import json
import math
from typing import Dict, List

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from swarm_interfaces.msg import RobotState


class RelayTreeEvaluator(Node):
    """
    Lightweight relay-tree evaluation node.

    Subscribes:
      /swarm/robot_states

    Publishes:
      /swarm/relay_tree_eval          std_msgs/String JSON summary
      /swarm/relay_tree_eval_text     std_msgs/String human-readable summary
      /swarm/relay_tree_eval_markers  visualization_msgs/MarkerArray RViz overlay
    """

    def __init__(self):
        super().__init__('relay_tree_evaluator')

        self._declare_parameters()
        self._read_parameters()

        self.states: Dict[str, RobotState] = {}
        self.last_seen_ns: Dict[str, int] = {}

        self.start_ns = self.get_clock().now().nanoseconds
        self.known_relays = set()
        self.known_groups = set()
        self.events: List[dict] = []

        self.create_subscription(RobotState, self.state_topic, self.state_callback, 10)

        self.eval_pub = self.create_publisher(String, self.eval_topic, 10)
        self.text_pub = self.create_publisher(String, self.text_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)

        self.create_timer(1.0 / self.publish_rate_hz, self.publish_evaluation)

        self.get_logger().info('[relay_tree_evaluator] started')

    def _declare_parameters(self):
        self.declare_parameter('state_topic', '/swarm/robot_states')
        self.declare_parameter('eval_topic', '/swarm/relay_tree_eval')
        self.declare_parameter('text_topic', '/swarm/relay_tree_eval_text')
        self.declare_parameter('marker_topic', '/swarm/relay_tree_eval_markers')

        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('state_timeout_sec', 3.0)

        self.declare_parameter('show_rviz_text', True)
        self.declare_parameter('rviz_text_x', -8.0)
        self.declare_parameter('rviz_text_y', 8.0)
        self.declare_parameter('rviz_text_z', 1.5)
        self.declare_parameter('rviz_text_height', 0.30)

        self.declare_parameter('max_recent_events_displayed', 5)
        self.declare_parameter('max_relay_link_distance_m', 30.0)

    def _read_parameters(self):
        self.state_topic = str(self.get_parameter('state_topic').value)
        self.eval_topic = str(self.get_parameter('eval_topic').value)
        self.text_topic = str(self.get_parameter('text_topic').value)
        self.marker_topic = str(self.get_parameter('marker_topic').value)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.state_timeout_sec = float(self.get_parameter('state_timeout_sec').value)

        self.show_rviz_text = bool(self.get_parameter('show_rviz_text').value)
        self.rviz_text_x = float(self.get_parameter('rviz_text_x').value)
        self.rviz_text_y = float(self.get_parameter('rviz_text_y').value)
        self.rviz_text_z = float(self.get_parameter('rviz_text_z').value)
        self.rviz_text_height = float(self.get_parameter('rviz_text_height').value)

        self.max_recent_events_displayed = int(
            self.get_parameter('max_recent_events_displayed').value
        )
        self.max_relay_link_distance_m = float(
            self.get_parameter('max_relay_link_distance_m').value
        )

    def state_callback(self, msg: RobotState):
        self.states[msg.robot_name] = msg
        self.last_seen_ns[msg.robot_name] = self.get_clock().now().nanoseconds

    def active_recent_states(self) -> Dict[str, RobotState]:
        now_ns = self.get_clock().now().nanoseconds
        timeout_ns = int(self.state_timeout_sec * 1e9)

        result = {}
        for name, state in self.states.items():
            if now_ns - self.last_seen_ns.get(name, 0) > timeout_ns:
                continue
            if not state.active:
                continue
            result[name] = state

        return result

    def publish_evaluation(self):
        states = self.active_recent_states()
        now = self.get_clock().now()
        elapsed_sec = (now.nanoseconds - self.start_ns) / 1e9

        summary = self.compute_summary(states, elapsed_sec)
        self.detect_events(states, elapsed_sec, summary)
        summary['recent_events'] = self.events[-self.max_recent_events_displayed:]

        json_msg = String()
        json_msg.data = json.dumps(summary, sort_keys=True)
        self.eval_pub.publish(json_msg)

        text = self.format_text(summary)
        text_msg = String()
        text_msg.data = text
        self.text_pub.publish(text_msg)

        if self.show_rviz_text:
            self.publish_text_marker(text, now)

    def compute_summary(self, states: Dict[str, RobotState], elapsed_sec: float) -> dict:
        relays = []
        root_relays = []
        active_groups: Dict[str, List[str]] = {}
        leaders = {}
        followers = {}

        for name, state in states.items():
            if state.role == 'root_relay':
                root_relays.append(name)
                relays.append(name)
            elif state.role == 'relay':
                relays.append(name)
            elif state.role == 'group_leader':
                active_groups.setdefault(state.group_id, []).append(name)
                leaders[state.group_id] = name
            elif state.role == 'group_follower':
                active_groups.setdefault(state.group_id, []).append(name)
                followers.setdefault(state.group_id, []).append(name)

        relay_links = self.compute_relay_links(states, leaders)

        max_depth = 0
        for state in states.values():
            max_depth = max(max_depth, int(state.branch_depth))

        return {
            'time_sec': round(elapsed_sec, 2),
            'robots_active': len(states),
            'root_relays': sorted(root_relays),
            'relays': sorted(relays, key=self.robot_number),
            'relay_count': len(relays),
            'active_group_count': len(active_groups),
            'active_groups': {
                gid: {
                    'robots': sorted(names, key=self.robot_number),
                    'size': len(names),
                    'leader': leaders.get(gid, ''),
                    'followers': sorted(followers.get(gid, []), key=self.robot_number),
                }
                for gid, names in sorted(active_groups.items())
            },
            'max_branch_depth': max_depth,
            'relay_links': relay_links,
            'worst_relay_link_m': max(
                [link['distance_m'] for link in relay_links], default=0.0
            ),
            'relay_link_ok': all(
                link['distance_m'] <= self.max_relay_link_distance_m
                for link in relay_links
            ),
            'max_relay_link_distance_m': self.max_relay_link_distance_m,
        }

    def compute_relay_links(self, states: Dict[str, RobotState], leaders: Dict[str, str]) -> List[dict]:
        links = []

        for group_id, leader_name in leaders.items():
            leader_state = states.get(leader_name)
            if leader_state is None:
                continue

            parent_relay_id = leader_state.parent_relay_id
            parent_relay_state = states.get(parent_relay_id)

            if parent_relay_state is None:
                continue

            distance = math.hypot(
                leader_state.x - parent_relay_state.x,
                leader_state.y - parent_relay_state.y,
            )

            links.append({
                'group_id': group_id,
                'leader': leader_name,
                'parent_relay': parent_relay_id,
                'distance_m': round(distance, 2),
                'ok': distance <= self.max_relay_link_distance_m,
            })

        links.sort(key=lambda x: x['group_id'])
        return links

    def detect_events(self, states: Dict[str, RobotState], elapsed_sec: float, summary: dict):
        current_relays = set(summary['relays'])
        new_relays = sorted(current_relays - self.known_relays, key=self.robot_number)

        for relay_name in new_relays:
            state = states.get(relay_name)
            if state is None:
                continue
            self.events.append({
                'time_sec': round(elapsed_sec, 2),
                'type': 'relay_created',
                'relay': relay_name,
                'role': state.role,
                'group_id': state.group_id,
                'parent_relay_id': state.parent_relay_id,
                'branch_depth': int(state.branch_depth),
                'x': round(state.x, 2),
                'y': round(state.y, 2),
            })

        self.known_relays = current_relays

        current_groups = set(summary['active_groups'].keys())
        new_groups = sorted(current_groups - self.known_groups)

        for group_id in new_groups:
            group = summary['active_groups'][group_id]
            self.events.append({
                'time_sec': round(elapsed_sec, 2),
                'type': 'group_created',
                'group_id': group_id,
                'leader': group['leader'],
                'size': group['size'],
                'robots': group['robots'],
            })

        self.known_groups = current_groups

        if len(self.events) > 80:
            self.events = self.events[-80:]

    def format_text(self, summary: dict) -> str:
        lines = []
        status = 'OK' if summary['relay_link_ok'] else 'WARN'

        lines.append(f"Relay eval t={summary['time_sec']:.1f}s [{status}]")
        lines.append(
            f"robots={summary['robots_active']} relays={summary['relay_count']} "
            f"groups={summary['active_group_count']} depth={summary['max_branch_depth']}"
        )
        lines.append(
            f"worst link={summary['worst_relay_link_m']:.1f}/"
            f"{summary['max_relay_link_distance_m']:.1f} m"
        )

        if summary['active_groups']:
            lines.append('Groups:')
            for gid, group in summary['active_groups'].items():
                robots = ','.join(group['robots'])
                lines.append(f"  {gid}: n={group['size']} lead={group['leader']} [{robots}]")

        if summary['relay_links']:
            lines.append('Links:')
            for link in summary['relay_links']:
                ok = 'OK' if link['ok'] else 'FAR'
                lines.append(
                    f"  {link['group_id']}: {link['parent_relay']} -> "
                    f"{link['leader']} {link['distance_m']:.1f}m {ok}"
                )

        events = summary.get('recent_events', [])
        if events:
            lines.append('Events:')
            for event in events[-self.max_recent_events_displayed:]:
                if event['type'] == 'relay_created':
                    lines.append(
                        f"  {event['time_sec']:.1f}s relay {event['relay']} "
                        f"d{event['branch_depth']}"
                    )
                elif event['type'] == 'group_created':
                    lines.append(
                        f"  {event['time_sec']:.1f}s group {event['group_id']} "
                        f"n={event['size']}"
                    )

        return '\n'.join(lines)

    def publish_text_marker(self, text: str, now):
        marker_array = MarkerArray()

        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = now.to_msg()
        marker.ns = 'relay_tree_eval_text'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = self.rviz_text_x
        marker.pose.position.y = self.rviz_text_y
        marker.pose.position.z = self.rviz_text_z
        marker.pose.orientation.w = 1.0

        marker.scale.z = self.rviz_text_height

        marker.color.r = 0.9
        marker.color.g = 1.0
        marker.color.b = 0.9
        marker.color.a = 1.0

        marker.text = text
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 900_000_000

        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def robot_number(self, name: str) -> int:
        digits = ''.join(ch for ch in name if ch.isdigit())
        return int(digits) if digits else 999


def main(args=None):
    rclpy.init(args=args)
    node = RelayTreeEvaluator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
