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


class Branch:
    def __init__(
        self,
        group_id: str,
        robots: List[str],
        heading_deg: float,
        parent_relay_id: str,
        parent_group_id: str,
        depth: int,
        start_x: float,
        start_y: float,
        created_time_sec: float,
    ):
        self.group_id = group_id
        self.robots = list(robots)
        self.heading_deg = float(heading_deg)
        self.parent_relay_id = parent_relay_id
        self.parent_group_id = parent_group_id
        self.depth = int(depth)
        self.start_x = float(start_x)
        self.start_y = float(start_y)
        self.created_time_sec = float(created_time_sec)

    @property
    def leader(self) -> str:
        return self.robots[0] if self.robots else ''


class MockSingleRootFourBranchesSplitting(Node):
    """
    RViz-only kinematic mock for one shared root relay and four coverage branches.

    Publishes:
      /swarm/robot_states

    No Gazebo.
    No relay managers.
    No role-assignment topics.

    Unlike the simple mock, this version DOES split:
      - Each active branch leader drives forward.
      - Once a branch travels split_distance_m, its tail robot becomes a relay.
      - The remaining robots split into left/right child branches.
      - All branches still share one root relay: robot18.
    """

    def __init__(self):
        super().__init__('mock_single_root_four_branches_splitting')

        self.declare_parameter('state_topic', '/swarm/robot_states')
        self.declare_parameter('publish_rate_hz', 10.0)

        self.declare_parameter('forward_speed', 0.08)
        self.declare_parameter('follow_speed', 0.13)
        self.declare_parameter('follow_distance_m', 0.95)
        self.declare_parameter('snap_distance_m', 0.08)

        self.declare_parameter('initial_chain_spacing_m', 0.78)
        self.declare_parameter('initial_lateral_spacing_m', 0.28)

        # Your coverage headings.
        self.declare_parameter('heading_a_deg', -90.0)
        self.declare_parameter('heading_b_deg', 180.0)
        self.declare_parameter('heading_c_deg', 0.0)
        self.declare_parameter('heading_d_deg', 90.0)

        # Shared root.
        self.declare_parameter('root_x', 0.0)
        self.declare_parameter('root_y', 0.0)

        # Compact initial branch centers around root.
        self.declare_parameter('group_a_center_x', 0.0)
        self.declare_parameter('group_a_center_y', -1.7)

        self.declare_parameter('group_b_center_x', -1.7)
        self.declare_parameter('group_b_center_y', 0.0)

        self.declare_parameter('group_c_center_x', 1.7)
        self.declare_parameter('group_c_center_y', 0.0)

        self.declare_parameter('group_d_center_x', 0.0)
        self.declare_parameter('group_d_center_y', 1.7)

        # Splitting behavior.
        self.declare_parameter('split_distance_m', 2.5)
        self.declare_parameter('branch_angle_deg', 28.0)
        self.declare_parameter('min_group_size_to_split', 2)
        self.declare_parameter('max_branch_depth', 3)
        self.declare_parameter('min_group_age_before_split_sec', 3.0)

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

        self.split_distance_m = float(self.get_parameter('split_distance_m').value)
        self.branch_angle_deg = float(self.get_parameter('branch_angle_deg').value)
        self.min_group_size_to_split = int(self.get_parameter('min_group_size_to_split').value)
        self.max_branch_depth = int(self.get_parameter('max_branch_depth').value)
        self.min_group_age_before_split_sec = float(
            self.get_parameter('min_group_age_before_split_sec').value
        )

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
        self.initial_group_a = ['robot1', 'robot2', 'robot3', 'robot4']
        self.initial_group_b = ['robot5', 'robot6', 'robot7', 'robot8']
        self.initial_group_c = ['robot9', 'robot10', 'robot11', 'robot12']
        self.initial_group_d = ['robot13', 'robot14', 'robot15', 'robot16', 'robot17']

        self.robots = (
            self.initial_group_a
            + self.initial_group_b
            + self.initial_group_c
            + self.initial_group_d
            + [self.root_robot]
        )

        self.pose: Dict[str, Tuple[float, float, float]] = {}
        self.relay_robots = set()
        self.relay_parent_by_robot: Dict[str, str] = {}
        self.relay_depth_by_robot: Dict[str, int] = {}
        self.branches: Dict[str, Branch] = {}
        self.group_by_robot: Dict[str, str] = {}
        self.next_group_index = 0

        self._init_heading_chain(self.initial_group_a, ax, ay, self.heading_a_deg)
        self._init_heading_chain(self.initial_group_b, bx, by, self.heading_b_deg)
        self._init_heading_chain(self.initial_group_c, cx, cy, self.heading_c_deg)
        self._init_heading_chain(self.initial_group_d, dx, dy, self.heading_d_deg)
        self.pose[self.root_robot] = (root_x, root_y, 0.0)

        now_sec = 0.0
        self._add_initial_branch('A', self.initial_group_a, self.heading_a_deg, now_sec)
        self._add_initial_branch('B', self.initial_group_b, self.heading_b_deg, now_sec)
        self._add_initial_branch('C', self.initial_group_c, self.heading_c_deg, now_sec)
        self._add_initial_branch('D', self.initial_group_d, self.heading_d_deg, now_sec)

        self.state_pub = self.create_publisher(RobotState, self.state_topic, 10)

        self.start_ns = self.get_clock().now().nanoseconds
        self.last_update_ns = self.start_ns
        self.create_timer(1.0 / self.publish_rate_hz, self.tick)

        self.get_logger().info(
            'mock_single_root_four_branches_splitting started: '
            f'root={self.root_robot}, split_distance={self.split_distance_m:.1f}m'
        )

    def _elapsed_sec(self) -> float:
        return (self.get_clock().now().nanoseconds - self.start_ns) / 1e9

    def _add_initial_branch(
        self,
        group_id: str,
        robots: List[str],
        heading_deg: float,
        created_time_sec: float,
    ):
        leader = robots[0]
        lx, ly, _ = self.pose[leader]
        branch = Branch(
            group_id=group_id,
            robots=robots,
            heading_deg=heading_deg,
            parent_relay_id=self.root_robot,
            parent_group_id='ROOT',
            depth=1,
            start_x=lx,
            start_y=ly,
            created_time_sec=created_time_sec,
        )
        self.branches[group_id] = branch
        for r in robots:
            self.group_by_robot[r] = group_id

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
        self._maybe_split_branches()

        for name in self.robots:
            self.state_pub.publish(self._make_state(name, now))

    def _update_positions(self, dt: float):
        # Move active branch leaders.
        for branch in list(self.branches.values()):
            if not branch.robots:
                continue

            leader = branch.leader
            heading = math.radians(branch.heading_deg)

            x, y, yaw = self.pose[leader]
            x += self.forward_speed * math.cos(heading) * dt
            y += self.forward_speed * math.sin(heading) * dt
            yaw = heading
            self.pose[leader] = (x, y, yaw)

        # Followers chase their current branch leader.
        for branch in list(self.branches.values()):
            leader = branch.leader
            if not leader:
                continue

            for name in branch.robots:
                if name == leader:
                    continue

                x, y, yaw = self.pose[name]
                lx, ly, _ = self.pose[leader]

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

        # Relay robots and root stay fixed.
        rx, ry, _ = self.pose[self.root_robot]
        self.pose[self.root_robot] = (rx, ry, 0.0)

    def _maybe_split_branches(self):
        elapsed = self._elapsed_sec()
        split_candidates = []

        for group_id, branch in self.branches.items():
            if len(branch.robots) < self.min_group_size_to_split:
                continue

            if branch.depth >= self.max_branch_depth:
                continue

            if elapsed - branch.created_time_sec < self.min_group_age_before_split_sec:
                continue

            leader = branch.leader
            lx, ly, _ = self.pose[leader]

            progress = math.hypot(lx - branch.start_x, ly - branch.start_y)
            if progress < self.split_distance_m:
                continue

            split_candidates.append(group_id)

        for group_id in split_candidates:
            if group_id in self.branches:
                self._split_branch(group_id)

    def _split_branch(self, group_id: str):
        branch = self.branches.get(group_id)
        if branch is None:
            return

        if len(branch.robots) < self.min_group_size_to_split:
            return

        # Tail robot becomes relay.
        relay_robot = branch.robots[-1]
        remaining = branch.robots[:-1]

        if len(remaining) == 0:
            return

        self.relay_robots.add(relay_robot)
        self.group_by_robot[relay_robot] = f'{group_id}_relay'
        self.relay_parent_by_robot[relay_robot] = branch.parent_relay_id
        self.relay_depth_by_robot[relay_robot] = branch.depth

        relay_x, relay_y, _ = self.pose[relay_robot]

        # Split remaining robots into left/right child branches.
        split_index = max(1, len(remaining) // 2)
        left_robots = remaining[:split_index]
        right_robots = remaining[split_index:]

        # If only one child exists, just continue it straight.
        del self.branches[group_id]

        child_specs = []
        if left_robots and right_robots:
            child_specs.append((left_robots, branch.heading_deg - self.branch_angle_deg))
            child_specs.append((right_robots, branch.heading_deg + self.branch_angle_deg))
        elif left_robots:
            child_specs.append((left_robots, branch.heading_deg))
        elif right_robots:
            child_specs.append((right_robots, branch.heading_deg))

        for robots, heading_deg in child_specs:
            child_id = f'{group_id}_{self.next_group_index}'
            self.next_group_index += 1

            leader = robots[0]
            lx, ly, _ = self.pose[leader]

            child = Branch(
                group_id=child_id,
                robots=robots,
                heading_deg=heading_deg,
                parent_relay_id=relay_robot,
                parent_group_id=group_id,
                depth=branch.depth + 1,
                start_x=lx,
                start_y=ly,
                created_time_sec=self._elapsed_sec(),
            )

            self.branches[child_id] = child
            for r in robots:
                self.group_by_robot[r] = child_id

        self.get_logger().info(
            f'split {group_id}: relay={relay_robot}, '
            f'children={[gid for gid in self.branches.keys() if gid.startswith(group_id + "_")]}'
        )

    def _branch_for_robot(self, name: str):
        gid = self.group_by_robot.get(name, '')
        return self.branches.get(gid)

    def _make_state(self, name: str, now) -> RobotState:
        x, y, yaw = self.pose[name]

        msg = RobotState()
        msg.robot_name = name
        msg.stamp = now.to_msg()

        msg.x = float(x)
        msg.y = float(y)
        msg.yaw = float(wrap_angle(yaw))
        msg.linear_speed = float(self.forward_speed)
        msg.leader_score = 0.0
        msg.active = True

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

        if name in self.relay_robots:
            msg.role = 'relay'
            msg.leader_id = ''
            msg.group_id = self.group_by_robot.get(name, '')
            msg.parent_group_id = ''
            msg.parent_relay_id = self.relay_parent_by_robot.get(name, self.root_robot)
            msg.assigned_heading_deg = 0.0
            msg.branch_depth = int(self.relay_depth_by_robot.get(name, 1))
            msg.is_relay = True
            msg.is_group_leader = False
            return msg

        branch = self._branch_for_robot(name)

        if branch is None:
            msg.role = 'idle'
            msg.leader_id = ''
            msg.group_id = ''
            msg.parent_group_id = ''
            msg.parent_relay_id = ''
            msg.assigned_heading_deg = 0.0
            msg.branch_depth = 0
            msg.is_relay = False
            msg.is_group_leader = False
            return msg

        msg.group_id = branch.group_id
        msg.parent_group_id = branch.parent_group_id
        msg.parent_relay_id = branch.parent_relay_id
        msg.assigned_heading_deg = float(branch.heading_deg)
        msg.branch_depth = int(branch.depth)

        if name == branch.leader:
            msg.role = 'group_leader'
            msg.leader_id = name
            msg.is_relay = False
            msg.is_group_leader = True
        else:
            msg.role = 'group_follower'
            msg.leader_id = branch.leader
            msg.is_relay = False
            msg.is_group_leader = False

        return msg


def main(args=None):
    rclpy.init(args=args)
    node = MockSingleRootFourBranchesSplitting()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
