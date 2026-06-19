#!/usr/bin/env python3
import math
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node

from swarm_interfaces.msg import RobotState


def wrap_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class MockSingleRootFourBranches(Node):
    """
    RViz-only kinematic mock for one shared root relay and four branch groups.

    Publishes:
      /swarm/robot_states

    No Gazebo.
    No relay managers.
    No role-assignment topics.

    Layout:
      robot18 = shared root relay

      Group A leader robot1  heading -90 deg
      Group B leader robot5  heading 180 deg
      Group C leader robot9  heading 0 deg
      Group D leader robot13 heading 90 deg

    This is for low-CPU RViz visualization of one common relay root with
    four coverage branches: south, west, east, north.
    """

    def __init__(self):
        super().__init__('mock_single_root_four_branches')

        self.declare_parameter('state_topic', '/swarm/robot_states')
        self.declare_parameter('publish_rate_hz', 10.0)

        self.declare_parameter('forward_speed', 0.08)
        self.declare_parameter('follow_speed', 0.13)
        self.declare_parameter('follow_distance_m', 0.95)
        self.declare_parameter('snap_distance_m', 0.08)

        self.declare_parameter('initial_chain_spacing_m', 0.78)
        self.declare_parameter('initial_lateral_spacing_m', 0.28)

        # Four coverage headings.
        # Keeping your existing three:
        #   A = -90, B = 180, C = 0
        # Adding:
        #   D = 90
        self.declare_parameter('heading_a_deg', -90.0)
        self.declare_parameter('heading_b_deg', 180.0)
        self.declare_parameter('heading_c_deg', 0.0)
        self.declare_parameter('heading_d_deg', 90.0)

        # Shared root.
        self.declare_parameter('root_x', 0.0)
        self.declare_parameter('root_y', 0.0)

        # Compact initial branch centers around the root.
        self.declare_parameter('group_a_center_x', 0.0)
        self.declare_parameter('group_a_center_y', -1.7)

        self.declare_parameter('group_b_center_x', -1.7)
        self.declare_parameter('group_b_center_y', 0.0)

        self.declare_parameter('group_c_center_x', 1.7)
        self.declare_parameter('group_c_center_y', 0.0)

        self.declare_parameter('group_d_center_x', 0.0)
        self.declare_parameter('group_d_center_y', 1.7)

        self.state_topic = str(self.get_parameter('state_topic').value)
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

        self.heading_a_deg = float(self.get_parameter('heading_a_deg').value)
        self.heading_b_deg = float(self.get_parameter('heading_b_deg').value)
        self.heading_c_deg = float(self.get_parameter('heading_c_deg').value)
        self.heading_d_deg = float(self.get_parameter('heading_d_deg').value)

        root_x = float(self.get_parameter('root_x').value)
        root_y = float(self.get_parameter('root_y').value)

        ax = float(self.get_parameter('group_a_center_x').value)
        ay = float(self.get_parameter('group_a_center_y').value)

        bx = float(self.get_parameter('group_b_center_x').value)
        by = float(self.get_parameter('group_b_center_y').value)

        cx = float(self.get_parameter('group_c_center_x').value)
        cy = float(self.get_parameter('group_c_center_y').value)

        dx = float(self.get_parameter('group_d_center_x').value)
        dy = float(self.get_parameter('group_d_center_y').value)

        self.root_robot = 'robot18'

        # 17 branch robots + 1 root = 18 total.
        #
        # A/B/C get 4 robots each, D gets 5 robots.
        # This keeps all 18 robots active with one shared root.
        self.group_a = ['robot1', 'robot2', 'robot3', 'robot4']
        self.group_b = ['robot5', 'robot6', 'robot7', 'robot8']
        self.group_c = ['robot9', 'robot10', 'robot11', 'robot12']
        self.group_d = ['robot13', 'robot14', 'robot15', 'robot16', 'robot17']

        self.group_by_robot: Dict[str, str] = {}
        for name in self.group_a:
            self.group_by_robot[name] = 'A'
        for name in self.group_b:
            self.group_by_robot[name] = 'B'
        for name in self.group_c:
            self.group_by_robot[name] = 'C'
        for name in self.group_d:
            self.group_by_robot[name] = 'D'
        self.group_by_robot[self.root_robot] = 'ROOT'

        self.leader_by_group = {
            'A': 'robot1',
            'B': 'robot5',
            'C': 'robot9',
            'D': 'robot13',
        }

        self.heading_by_group = {
            'A': self.heading_a_deg,
            'B': self.heading_b_deg,
            'C': self.heading_c_deg,
            'D': self.heading_d_deg,
        }

        self.robots = (
            self.group_a
            + self.group_b
            + self.group_c
            + self.group_d
            + [self.root_robot]
        )

        self.pose: Dict[str, Tuple[float, float, float]] = {}

        self._init_heading_chain(self.group_a, ax, ay, self.heading_a_deg)
        self._init_heading_chain(self.group_b, bx, by, self.heading_b_deg)
        self._init_heading_chain(self.group_c, cx, cy, self.heading_c_deg)
        self._init_heading_chain(self.group_d, dx, dy, self.heading_d_deg)
        self.pose[self.root_robot] = (root_x, root_y, 0.0)

        self.state_pub = self.create_publisher(RobotState, self.state_topic, 10)

        self.last_update_ns = self.get_clock().now().nanoseconds
        self.create_timer(1.0 / self.publish_rate_hz, self.tick)

        self.get_logger().info(
            'mock_single_root_four_branches started: '
            f'root={self.root_robot}, publishing {self.state_topic}'
        )

    def _init_heading_chain(
        self,
        names: List[str],
        cx: float,
        cy: float,
        heading_deg: float,
    ):
        heading = math.radians(heading_deg)

        forward_x = math.cos(heading)
        forward_y = math.sin(heading)

        lateral_x = -math.sin(heading)
        lateral_y = math.cos(heading)

        n = len(names)
        center_offset = 0.5 * (n - 1) * self.initial_chain_spacing_m

        for i, name in enumerate(names):
            # Leader/front has positive forward offset.
            along = center_offset - i * self.initial_chain_spacing_m

            # Small zig-zag so labels/markers remain readable.
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

    def tick(self):
        now = self.get_clock().now()
        now_ns = now.nanoseconds

        dt = max(0.001, min(0.2, (now_ns - self.last_update_ns) / 1e9))
        self.last_update_ns = now_ns

        self._update_positions(dt)

        for name in self.robots:
            self.state_pub.publish(self._make_state(name, now))

    def _update_positions(self, dt: float):
        # Move four branch leaders.
        for group_id, leader in self.leader_by_group.items():
            heading_deg = self.heading_by_group[group_id]
            heading = math.radians(heading_deg)

            x, y, yaw = self.pose[leader]
            x += self.forward_speed * math.cos(heading) * dt
            y += self.forward_speed * math.sin(heading) * dt
            yaw = heading
            self.pose[leader] = (x, y, yaw)

        # Followers chase their branch leader.
        branch_robots = self.group_a + self.group_b + self.group_c + self.group_d

        for name in branch_robots:
            group_id = self.group_by_robot[name]
            leader = self.leader_by_group[group_id]

            if name == leader:
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

        # Root stays fixed.
        rx, ry, ryaw = self.pose[self.root_robot]
        self.pose[self.root_robot] = (rx, ry, 0.0)

    def _make_state(self, name: str, now) -> RobotState:
        x, y, yaw = self.pose[name]
        group_id = self.group_by_robot[name]

        msg = RobotState()
        msg.robot_name = name
        msg.stamp = now.to_msg()

        msg.x = float(x)
        msg.y = float(y)
        msg.yaw = float(wrap_angle(yaw))
        msg.linear_speed = float(self.forward_speed)
        msg.leader_score = 0.0
        msg.active = True

        msg.group_id = group_id
        msg.parent_group_id = 'ROOT'
        msg.parent_relay_id = self.root_robot
        msg.branch_depth = 1

        if name == self.root_robot:
            msg.role = 'root_relay'
            msg.leader_id = ''
            msg.group_id = 'ROOT'
            msg.parent_group_id = ''
            msg.parent_relay_id = ''
            msg.assigned_heading_deg = 0.0
            msg.branch_depth = 0
            msg.is_relay = True
            msg.is_group_leader = False
            return msg

        leader = self.leader_by_group[group_id]
        heading_deg = self.heading_by_group[group_id]

        msg.assigned_heading_deg = float(heading_deg)

        if name == leader:
            msg.role = 'group_leader'
            msg.leader_id = name
            msg.is_relay = False
            msg.is_group_leader = True
        else:
            msg.role = 'group_follower'
            msg.leader_id = leader
            msg.is_relay = False
            msg.is_group_leader = False

        return msg


def main(args=None):
    rclpy.init(args=args)
    node = MockSingleRootFourBranches()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
