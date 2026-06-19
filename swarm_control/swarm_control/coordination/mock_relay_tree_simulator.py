#!/usr/bin/env python3
import json
import math
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from swarm_interfaces.msg import RobotState


def wrap_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class MockRelayTreeSimulator(Node):
    """
    Lightweight RViz-only kinematic simulator for 3 relay-tree groups.

    Publishes:
      /swarm/robot_states

    Subscribes:
      /swarm/role_assignments_A
      /swarm/role_assignments_B
      /swarm/role_assignments_C

    This replaces Gazebo + swarm_member + path_follower + tree_explorer for RViz testing.

    Important:
      The initial robot positions are generated as compact chains already facing each
      group's assigned heading. The launch file should pass the same group heading
      values as the relay managers use.
    """

    def __init__(self):
        super().__init__('mock_relay_tree_simulator')

        self.declare_parameter('state_topic', '/swarm/robot_states')

        self.declare_parameter('role_assignment_topic_a', '/swarm/role_assignments_A')
        self.declare_parameter('role_assignment_topic_b', '/swarm/role_assignments_B')
        self.declare_parameter('role_assignment_topic_c', '/swarm/role_assignments_C')

        self.declare_parameter('publish_rate_hz', 10.0)

        self.declare_parameter('forward_speed', 0.08)
        self.declare_parameter('follow_speed', 0.13)
        self.declare_parameter('follow_distance_m', 0.95)
        self.declare_parameter('snap_distance_m', 0.08)

        # Compact chain formation.
        self.declare_parameter('initial_chain_spacing_m', 0.82)
        self.declare_parameter('initial_lateral_spacing_m', 0.30)

        # Start positions for the three groups.
        self.declare_parameter('group_a_center_x', 0.0)
        self.declare_parameter('group_a_center_y', -1.8)
        self.declare_parameter('group_a_heading_deg', -90.0)

        self.declare_parameter('group_b_center_x', -1.8)
        self.declare_parameter('group_b_center_y', 1.2)
        self.declare_parameter('group_b_heading_deg', 180.0)

        self.declare_parameter('group_c_center_x', 1.8)
        self.declare_parameter('group_c_center_y', 1.2)
        self.declare_parameter('group_c_heading_deg', 0.0)

        self.state_topic = str(self.get_parameter('state_topic').value)

        self.role_topic_a = str(self.get_parameter('role_assignment_topic_a').value)
        self.role_topic_b = str(self.get_parameter('role_assignment_topic_b').value)
        self.role_topic_c = str(self.get_parameter('role_assignment_topic_c').value)

        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.follow_speed = float(self.get_parameter('follow_speed').value)
        self.follow_distance_m = float(self.get_parameter('follow_distance_m').value)
        self.snap_distance_m = float(self.get_parameter('snap_distance_m').value)

        self.initial_chain_spacing_m = float(
            self.get_parameter('initial_chain_spacing_m').value
        )
        self.initial_lateral_spacing_m = float(
            self.get_parameter('initial_lateral_spacing_m').value
        )

        ax = float(self.get_parameter('group_a_center_x').value)
        ay = float(self.get_parameter('group_a_center_y').value)
        ah = float(self.get_parameter('group_a_heading_deg').value)

        bx = float(self.get_parameter('group_b_center_x').value)
        by = float(self.get_parameter('group_b_center_y').value)
        bh = float(self.get_parameter('group_b_heading_deg').value)

        cx = float(self.get_parameter('group_c_center_x').value)
        cy = float(self.get_parameter('group_c_center_y').value)
        ch = float(self.get_parameter('group_c_heading_deg').value)

        # 18 robots total, split into 3 relay trees of 6 robots each.
        self.group_a = [
            'robot1', 'robot2', 'robot3',
            'robot6', 'robot5', 'robot4',
        ]

        self.group_b = [
            'robot7', 'robot8', 'robot9',
            'robot12', 'robot11', 'robot10',
        ]

        self.group_c = [
            'robot13', 'robot14', 'robot15',
            'robot18', 'robot17', 'robot16',
        ]

        self.all_robots = self.group_a + self.group_b + self.group_c

        self.pose: Dict[str, Tuple[float, float, float]] = {}
        self.assignment: Dict[str, dict] = {}

        self._init_heading_chain(self.group_a, ax, ay, ah)
        self._init_heading_chain(self.group_b, bx, by, bh)
        self._init_heading_chain(self.group_c, cx, cy, ch)

        self.state_pub = self.create_publisher(RobotState, self.state_topic, 10)

        self.create_subscription(String, self.role_topic_a, self.assignment_cb, 10)
        self.create_subscription(String, self.role_topic_b, self.assignment_cb, 10)
        self.create_subscription(String, self.role_topic_c, self.assignment_cb, 10)

        self.last_update_ns = self.get_clock().now().nanoseconds
        self.create_timer(1.0 / self.publish_rate_hz, self.tick)

        self.get_logger().info(
            'mock_relay_tree_simulator started: '
            f'publishing {self.state_topic}, listening to '
            f'{self.role_topic_a}, {self.role_topic_b}, {self.role_topic_c}'
        )

    def _init_heading_chain(
        self,
        names: List[str],
        cx: float,
        cy: float,
        heading_deg: float,
    ):
        """
        Create a compact chain centered near (cx, cy), with the first robot in the
        list at the front and the last robot at the tail.

        This matches your relay-manager setup where the first robot is the initial
        leader and the tail becomes the root relay.
        """
        heading = math.radians(heading_deg)

        forward_x = math.cos(heading)
        forward_y = math.sin(heading)

        # Left-hand lateral vector.
        lateral_x = -math.sin(heading)
        lateral_y = math.cos(heading)

        n = len(names)
        center_offset = 0.5 * (n - 1) * self.initial_chain_spacing_m

        for i, name in enumerate(names):
            # Leader/front has positive forward offset.
            along = center_offset - i * self.initial_chain_spacing_m

            # Small zig-zag lateral offset so the chain is readable in RViz.
            if i == 0:
                lateral = 0.0
            elif i % 2 == 1:
                lateral = self.initial_lateral_spacing_m
            else:
                lateral = -self.initial_lateral_spacing_m

            x = cx + along * forward_x + lateral * lateral_x
            y = cy + along * forward_y + lateral * lateral_y
            yaw = heading

            self.pose[name] = (x, y, yaw)

    def assignment_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warn(f'bad assignment JSON: {exc}')
            return

        if not isinstance(data, dict):
            return

        for name, assignment in data.items():
            if name in self.pose and isinstance(assignment, dict):
                self.assignment[name] = assignment

    def tick(self):
        now = self.get_clock().now()
        now_ns = now.nanoseconds

        dt = max(0.001, min(0.2, (now_ns - self.last_update_ns) / 1e9))
        self.last_update_ns = now_ns

        self._update_positions(dt)

        for name in self.all_robots:
            self.state_pub.publish(self._make_state(name, now))

    def _update_positions(self, dt: float):
        # First move active group leaders.
        for name in self.all_robots:
            ass = self.assignment.get(name, {})
            role = str(ass.get('role', 'idle'))

            if role not in ('leader', 'group_leader'):
                continue

            active = bool(ass.get('active', True))
            if not active:
                continue

            heading_deg = float(ass.get('assigned_heading_deg', 0.0))
            heading = math.radians(heading_deg)

            x, y, yaw = self.pose[name]
            x += self.forward_speed * math.cos(heading) * dt
            y += self.forward_speed * math.sin(heading) * dt
            yaw = heading
            self.pose[name] = (x, y, yaw)

        # Then followers chase their assigned leader with desired spacing.
        for name in self.all_robots:
            ass = self.assignment.get(name, {})
            role = str(ass.get('role', 'idle'))

            if role not in ('follower', 'group_follower'):
                continue

            leader = str(ass.get('leader_id', ''))
            if leader not in self.pose:
                continue

            x, y, yaw = self.pose[name]
            lx, ly, lyaw = self.pose[leader]

            dx = lx - x
            dy = ly - y
            dist = math.hypot(dx, dy)

            if dist < 1e-6:
                continue

            err = dist - self.follow_distance_m

            if abs(err) <= self.snap_distance_m:
                yaw = math.atan2(dy, dx)
                self.pose[name] = (x, y, yaw)
                continue

            step = max(-self.follow_speed * dt, min(self.follow_speed * dt, err))
            x += step * dx / dist
            y += step * dy / dist
            yaw = math.atan2(dy, dx)
            self.pose[name] = (x, y, yaw)

        # Relays/root relays remain fixed, but yaw tracks assigned heading.
        for name in self.all_robots:
            ass = self.assignment.get(name, {})
            role = str(ass.get('role', 'idle'))

            if role in ('relay', 'root_relay'):
                x, y, yaw = self.pose[name]
                yaw = math.radians(float(ass.get('assigned_heading_deg', math.degrees(yaw))))
                self.pose[name] = (x, y, yaw)

    def _make_state(self, name: str, now) -> RobotState:
        ass = self.assignment.get(name, {})
        x, y, yaw = self.pose[name]

        msg = RobotState()
        msg.robot_name = name
        msg.stamp = now.to_msg()

        msg.x = float(x)
        msg.y = float(y)
        msg.yaw = float(wrap_angle(yaw))
        msg.linear_speed = float(self.forward_speed)

        msg.leader_score = 0.0
        msg.role = str(ass.get('role', 'idle'))
        msg.leader_id = str(ass.get('leader_id', ''))
        msg.active = bool(ass.get('active', True))

        msg.group_id = str(ass.get('group_id', ''))
        msg.parent_group_id = str(ass.get('parent_group_id', ''))
        msg.parent_relay_id = str(ass.get('parent_relay_id', ''))
        msg.assigned_heading_deg = float(ass.get('assigned_heading_deg', 0.0))
        msg.branch_depth = int(ass.get('branch_depth', 0))

        msg.is_relay = bool(ass.get('is_relay', msg.role in ('relay', 'root_relay')))
        msg.is_group_leader = bool(
            ass.get('is_group_leader', msg.role in ('leader', 'group_leader'))
        )

        return msg


def main(args=None):
    rclpy.init(args=args)
    node = MockRelayTreeSimulator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
