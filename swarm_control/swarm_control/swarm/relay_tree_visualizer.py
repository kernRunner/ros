import math
import re
from typing import Dict, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from swarm_interfaces.msg import RobotState


class RelayTreeVisualizer(Node):
    """
    RViz marker visualizer for the relay-tree exploration state.

    Subscribes:
      /swarm/robot_states

    Publishes:
      /swarm/relay_tree_markers

    Visualizes:
      - root relay
      - relay robots
      - group leaders
      - group followers
      - parent-relay links
      - leader-follower links
      - text labels with role/group info
    """

    def __init__(self):
        super().__init__('relay_tree_visualizer')

        self._declare_parameters()
        self._read_parameters()
        self._init_state()
        self._init_ros_interfaces()

        self.get_logger().info('[relay_tree_visualizer] started')

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------

    def _declare_parameters(self):
        self.declare_parameter('state_topic', '/swarm/robot_states')
        self.declare_parameter('marker_topic', '/swarm/relay_tree_markers')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('state_timeout_sec', 2.0)

        self.declare_parameter('robot_marker_scale', 0.38)
        self.declare_parameter('relay_marker_scale', 0.55)
        self.declare_parameter('text_height', 0.45)
        self.declare_parameter('line_width', 0.045)

        self.declare_parameter('show_all_follower_links', True)
        self.declare_parameter('show_parent_relay_links', True)
        self.declare_parameter('show_text_labels', True)

    def _read_parameters(self):
        self.state_topic = self.get_parameter('state_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.state_timeout_sec = float(self.get_parameter('state_timeout_sec').value)

        self.robot_marker_scale = float(
            self.get_parameter('robot_marker_scale').value
        )
        self.relay_marker_scale = float(
            self.get_parameter('relay_marker_scale').value
        )
        self.text_height = float(self.get_parameter('text_height').value)
        self.line_width = float(self.get_parameter('line_width').value)

        self.show_all_follower_links = bool(
            self.get_parameter('show_all_follower_links').value
        )
        self.show_parent_relay_links = bool(
            self.get_parameter('show_parent_relay_links').value
        )
        self.show_text_labels = bool(self.get_parameter('show_text_labels').value)

    # ------------------------------------------------------------------
    # Initialization
    # ------------------------------------------------------------------

    def _init_state(self):
        self.states: Dict[str, RobotState] = {}
        self.last_seen_ns: Dict[str, int] = {}

    def _init_ros_interfaces(self):
        self.create_subscription(
            RobotState,
            self.state_topic,
            self.state_callback,
            10,
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.marker_topic,
            10,
        )

        self.create_timer(1.0 / self.publish_rate_hz, self.publish_markers)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def state_callback(self, msg: RobotState):
        self.states[msg.robot_name] = msg
        self.last_seen_ns[msg.robot_name] = self.get_clock().now().nanoseconds

    # ------------------------------------------------------------------
    # Marker publishing
    # ------------------------------------------------------------------

    def publish_markers(self):
        now = self.get_clock().now()
        active_states = self._active_recent_states(now.nanoseconds)

        marker_array = MarkerArray()

        # Clear old markers first. This avoids stale labels/lines when roles change.
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        marker_id = 0

        for name in sorted(active_states.keys(), key=self.robot_number):
            state = active_states[name]

            marker_array.markers.append(
                self._robot_marker(marker_id, state, now)
            )
            marker_id += 1

            if self.show_text_labels:
                marker_array.markers.append(
                    self._text_marker(marker_id, state, now)
                )
                marker_id += 1

        if self.show_parent_relay_links:
            for name in sorted(active_states.keys(), key=self.robot_number):
                state = active_states[name]
                parent_name = state.parent_relay_id

                if not parent_name or parent_name not in active_states:
                    continue

                # Draw parent relay links for relays and group leaders.
                # This shows the relay-tree backbone without too much clutter.
                if state.role not in ('relay', 'group_leader'):
                    continue

                marker_array.markers.append(
                    self._line_marker(
                        marker_id,
                        active_states[parent_name],
                        state,
                        now,
                        namespace='parent_relay_links',
                        color=self._color_parent_link(),
                        z_offset=0.16,
                    )
                )
                marker_id += 1

        if self.show_all_follower_links:
            for name in sorted(active_states.keys(), key=self.robot_number):
                state = active_states[name]

                if state.role != 'group_follower':
                    continue

                leader_name = state.leader_id

                if not leader_name or leader_name not in active_states:
                    continue

                marker_array.markers.append(
                    self._line_marker(
                        marker_id,
                        active_states[leader_name],
                        state,
                        now,
                        namespace='leader_follower_links',
                        color=self._color_follower_link(),
                        z_offset=0.08,
                    )
                )
                marker_id += 1

        self.marker_pub.publish(marker_array)

    def _active_recent_states(self, now_ns: int) -> Dict[str, RobotState]:
        recent = {}

        timeout_ns = int(self.state_timeout_sec * 1e9)

        for name, state in self.states.items():
            last_seen = self.last_seen_ns.get(name, 0)

            if now_ns - last_seen > timeout_ns:
                continue

            if not state.active:
                continue

            recent[name] = state

        return recent

    # ------------------------------------------------------------------
    # Marker builders
    # ------------------------------------------------------------------

    def _base_marker(self, marker_id: int, namespace: str, now) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = now.to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 500_000_000
        return marker

    def _robot_marker(self, marker_id: int, state: RobotState, now) -> Marker:
        marker = self._base_marker(marker_id, 'relay_tree_robots', now)

        marker.type = Marker.SPHERE
        marker.pose.position.x = state.x
        marker.pose.position.y = state.y
        marker.pose.position.z = 0.25
        marker.pose.orientation.w = 1.0

        scale = self.relay_marker_scale if state.is_relay else self.robot_marker_scale
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        marker.color = self._role_color(state)
        return marker

    def _text_marker(self, marker_id: int, state: RobotState, now) -> Marker:
        marker = self._base_marker(marker_id, 'relay_tree_labels', now)

        marker.type = Marker.TEXT_VIEW_FACING
        marker.pose.position.x = state.x
        marker.pose.position.y = state.y
        marker.pose.position.z = 0.95
        marker.pose.orientation.w = 1.0

        marker.scale.z = self.text_height
        marker.color = self._color_white()

        marker.text = (
            f'{state.robot_name}\n'
            f'{state.role}\n'
            f'{state.group_id}'
        )

        return marker

    def _line_marker(
        self,
        marker_id: int,
        state_a: RobotState,
        state_b: RobotState,
        now,
        namespace: str,
        color: ColorRGBA,
        z_offset: float,
    ) -> Marker:
        marker = self._base_marker(marker_id, namespace, now)

        marker.type = Marker.LINE_LIST
        marker.scale.x = self.line_width
        marker.color = color

        point_a = Point()
        point_a.x = state_a.x
        point_a.y = state_a.y
        point_a.z = z_offset

        point_b = Point()
        point_b.x = state_b.x
        point_b.y = state_b.y
        point_b.z = z_offset

        marker.points.append(point_a)
        marker.points.append(point_b)

        return marker

    # ------------------------------------------------------------------
    # Colors
    # ------------------------------------------------------------------

    def _role_color(self, state: RobotState) -> ColorRGBA:
        if state.role == 'root_relay':
            return self._rgba(0.80, 0.15, 0.90, 1.0)  # purple

        if state.role == 'relay':
            return self._rgba(0.15, 0.55, 1.00, 1.0)  # blue

        if state.role in ('leader', 'group_leader'):
            return self._rgba(0.10, 0.90, 0.20, 1.0)  # green

        if state.role in ('follower', 'group_follower'):
            return self._rgba(1.00, 0.75, 0.10, 1.0)  # yellow/orange

        return self._rgba(0.80, 0.80, 0.80, 1.0)

    def _color_parent_link(self) -> ColorRGBA:
        return self._rgba(0.40, 0.70, 1.00, 0.90)

    def _color_follower_link(self) -> ColorRGBA:
        return self._rgba(0.90, 0.90, 0.90, 0.60)

    def _color_white(self) -> ColorRGBA:
        return self._rgba(1.0, 1.0, 1.0, 1.0)

    def _rgba(self, r: float, g: float, b: float, a: float) -> ColorRGBA:
        color = ColorRGBA()
        color.r = float(r)
        color.g = float(g)
        color.b = float(b)
        color.a = float(a)
        return color

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------

    def robot_number(self, name: str) -> int:
        match = re.search(r'\d+', name)
        return int(match.group()) if match else 999


def main(args=None):
    rclpy.init(args=args)
    node = RelayTreeVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
