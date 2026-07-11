# Controls follower robots so each robot follows its predecessor in the assigned chain.

import json
import math
import re
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from swarm_interfaces.msg import RobotState

from swarm_control.core.math_utils import normalize_angle, quaternion_to_yaw


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        self._declare_parameters()
        self._read_parameters()
        self._init_state()
        self._init_ros_interfaces()

        self.get_logger().info(f'[{self.robot_name}] sequential path_follower started')

    def _declare_parameters(self):
        self.declare_parameter('robot_name', 'robot2')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel_raw')
        self.declare_parameter('mission_command_topic', '/swarm/mission_command')

        self.declare_parameter('path_topic', '/swarm/leader_path')

        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)

        self.declare_parameter('desired_follow_distance_m', 1.15)
        self.declare_parameter('follow_deadband_m', 0.18)

        self.declare_parameter('max_linear_speed', 0.22)
        self.declare_parameter('min_linear_speed_when_far', 0.07)
        self.declare_parameter('max_angular_speed', 1.60)
        self.declare_parameter('linear_gain', 0.75)
        self.declare_parameter('angular_gain', 1.80)

        self.declare_parameter('min_robot_distance_m', 0.50)
        self.declare_parameter('slow_robot_distance_m', 0.85)
        self.declare_parameter('too_close_reverse_speed', -0.02)

        self.declare_parameter('far_gap_m', 1.45)
        self.declare_parameter('very_far_gap_m', 2.00)
        self.declare_parameter('catchup_speed_boost', 1.50)

        self.declare_parameter('startup_gap_ratio', 0.60)

        self.declare_parameter('sequential_start_enabled', True)
        self.declare_parameter('sequential_slot_delay_sec', 0.8)
        self.declare_parameter('sequential_front_pair_gap_ratio', 0.80)
        self.declare_parameter('sequential_front_alignment_tolerance_rad', 0.70)
        self.declare_parameter('sequential_self_alignment_tolerance_rad', 0.45)
        self.declare_parameter('sequential_prealign_enabled', True)
        self.declare_parameter('sequential_prealign_angular_gain', 0.65)
        self.declare_parameter('sequential_predecessor_move_m', 0.25)

        self.declare_parameter('lock_chain_order', True)
        self.declare_parameter('chain_order_mode', 'robot_name')
        self.declare_parameter('explicit_chain_order', [''])

        self.declare_parameter('require_formation_ready', False)

        self.declare_parameter('lookahead_m', 0.25)
        self.declare_parameter('goal_tolerance_m', 0.06)
        self.declare_parameter('min_path_length_m', 0.25)
        self.declare_parameter('fallback_to_predecessor', True)
        self.declare_parameter('startup_delay_per_slot_sec', 0.0)
        self.declare_parameter('cross_track_gain', 1.10)
        self.declare_parameter('max_cross_track_correction_rad', 0.35)
        self.declare_parameter('line_hold_enabled', False)
        self.declare_parameter('line_hold_gain', 0.0)
        self.declare_parameter('line_hold_integral_gain', 0.0)
        self.declare_parameter('line_hold_integral_limit', 0.0)
        self.declare_parameter('line_hold_start_delay_sec', 0.0)
        self.declare_parameter('line_hold_max_correction_rad', 0.0)
        self.declare_parameter('resync_enabled', False)
        self.declare_parameter('resync_lateral_error_m', 0.0)
        self.declare_parameter('resync_release_error_m', 0.0)
        self.declare_parameter('resync_angular_boost', 1.0)
        self.declare_parameter('resync_speed_scale', 1.0)

        self.declare_parameter('hold_gap_deadband_m', 0.12)
        self.declare_parameter('hold_heading_deadband_rad', 0.20)

        self.declare_parameter('speed_match_enabled', True)
        self.declare_parameter('speed_match_gain', 0.85)
        self.declare_parameter('min_creep_speed', 0.025)
        self.declare_parameter('command_smoothing_alpha_linear', 0.35)
        self.declare_parameter('command_smoothing_alpha_angular', 0.30)

    def _read_parameters(self):
        self.robot_name = self.get_parameter('robot_name').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.mission_command_topic = self.get_parameter('mission_command_topic').value

        self.spawn_x = float(self.get_parameter('spawn_x').value)
        self.spawn_y = float(self.get_parameter('spawn_y').value)

        self.desired_follow_distance_m = float(
            self.get_parameter('desired_follow_distance_m').value
        )
        self.follow_deadband_m = float(self.get_parameter('follow_deadband_m').value)

        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.min_linear_speed_when_far = float(
            self.get_parameter('min_linear_speed_when_far').value
        )
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.linear_gain = float(self.get_parameter('linear_gain').value)
        self.angular_gain = float(self.get_parameter('angular_gain').value)

        self.min_robot_distance_m = float(self.get_parameter('min_robot_distance_m').value)
        self.slow_robot_distance_m = float(self.get_parameter('slow_robot_distance_m').value)
        self.too_close_reverse_speed = float(
            self.get_parameter('too_close_reverse_speed').value
        )

        self.far_gap_m = float(self.get_parameter('far_gap_m').value)
        self.very_far_gap_m = float(self.get_parameter('very_far_gap_m').value)
        self.catchup_speed_boost = float(self.get_parameter('catchup_speed_boost').value)

        self.startup_gap_ratio = float(self.get_parameter('startup_gap_ratio').value)

        self.sequential_start_enabled = bool(
            self.get_parameter('sequential_start_enabled').value
        )
        self.sequential_slot_delay_sec = float(
            self.get_parameter('sequential_slot_delay_sec').value
        )
        self.sequential_front_pair_gap_ratio = float(
            self.get_parameter('sequential_front_pair_gap_ratio').value
        )
        self.sequential_front_alignment_tolerance_rad = float(
            self.get_parameter('sequential_front_alignment_tolerance_rad').value
        )
        self.sequential_self_alignment_tolerance_rad = float(
            self.get_parameter('sequential_self_alignment_tolerance_rad').value
        )
        self.sequential_prealign_enabled = bool(
            self.get_parameter('sequential_prealign_enabled').value
        )
        self.sequential_prealign_angular_gain = float(
            self.get_parameter('sequential_prealign_angular_gain').value
        )
        self.sequential_predecessor_move_m = float(
            self.get_parameter('sequential_predecessor_move_m').value
        )

        self.lock_chain_order = bool(self.get_parameter('lock_chain_order').value)
        self.chain_order_mode = str(self.get_parameter('chain_order_mode').value)
        self.explicit_chain_order = [
            str(name)
            for name in list(self.get_parameter('explicit_chain_order').value)
            if str(name)
        ]
        self.require_formation_ready = bool(
            self.get_parameter('require_formation_ready').value
        )

        self.hold_gap_deadband_m = float(
            self.get_parameter('hold_gap_deadband_m').value
        )
        self.hold_heading_deadband_rad = float(
            self.get_parameter('hold_heading_deadband_rad').value
        )

        self.speed_match_enabled = bool(
            self.get_parameter('speed_match_enabled').value
        )
        self.speed_match_gain = float(self.get_parameter('speed_match_gain').value)
        self.min_creep_speed = float(self.get_parameter('min_creep_speed').value)
        self.command_smoothing_alpha_linear = float(
            self.get_parameter('command_smoothing_alpha_linear').value
        )
        self.command_smoothing_alpha_angular = float(
            self.get_parameter('command_smoothing_alpha_angular').value
        )

    def _init_state(self):
        self.world_x = 0.0
        self.world_y = 0.0
        self.yaw = 0.0
        self.have_self_state = False

        self.formation_done = False
        self.startup_released = False

        self.current_role = 'follower'
        self.current_leader_id = ''
        self.group_id = ''
        self.is_relay = False
        self.is_leader = False

        self.other_robots: Dict[str, RobotState] = {}
        self.initial_robot_positions: Dict[str, Tuple[float, float]] = {}
        self.locked_order: List[str] = []
        self.last_logged_order: List[str] = []

        self.last_chain_publish_ns = 0

        self.last_linear_cmd = 0.0
        self.last_angular_cmd = 0.0

        self.mission_mode = 'explore'
        self.last_mission_log_ns = 0

        self.group_join_ns = self.get_clock().now().nanoseconds
        self.last_startup_wait_log_ns = 0

    def _init_ros_interfaces(self):
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.chain_order_pub = self.create_publisher(String, '/swarm/chain_order', 10)

        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(RobotState, '/swarm/robot_states', self.state_callback, 10)
        self.create_subscription(Bool, 'formation_ready', self.formation_ready_cb, 10)
        self.create_subscription(String, self.mission_command_topic, self.mission_command_callback, 10)

        self.create_timer(0.1, self.control_loop)

    def mission_command_callback(self, msg: String):
        # Accepts plain text or JSON-like mission commands.
        raw = (msg.data or '').strip()

        if not raw:
            return

        mode = raw

        if raw.startswith('{'):
            lowered = raw.lower()
            if '"stop"' in lowered or "'stop'" in lowered:
                mode = 'stop'
            elif '"return_home"' in lowered or "'return_home'" in lowered:
                mode = 'return_home'
            elif '"explore"' in lowered or "'explore'" in lowered:
                mode = 'explore'

        mode = mode.strip().lower()

        if mode not in ('explore', 'stop', 'return_home'):
            self.get_logger().warn(
                f'[{self.robot_name}] ignoring unknown mission mode: {raw}'
            )
            return

        if mode == self.mission_mode:
            return

        self.mission_mode = mode

        if mode == 'stop':
            self.last_linear_cmd = 0.0
            self.last_angular_cmd = 0.0
            self.publish_cmd(0.0, 0.0)

        self.get_logger().warn(
            f'[{self.robot_name}] mission mode changed to {self.mission_mode}'
        )

    def odom_callback(self, msg: Odometry):
        # Uses odometry only until this robot's swarm state is available.
        if self.have_self_state:
            return

        self.world_x = self.spawn_x + msg.pose.pose.position.x
        self.world_y = self.spawn_y + msg.pose.pose.position.y
        self.yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def state_callback(self, msg: RobotState):
        self.other_robots[msg.robot_name] = msg

        if msg.robot_name not in self.initial_robot_positions:
            self.initial_robot_positions[msg.robot_name] = (msg.x, msg.y)

        if msg.robot_name != self.robot_name:
            return

        self.world_x = msg.x
        self.world_y = msg.y
        self.yaw = msg.yaw
        self.have_self_state = True

        old_leader = self.current_leader_id
        old_group = self.group_id
        old_role = self.current_role

        self.current_role = msg.role
        self.current_leader_id = msg.leader_id
        self.group_id = msg.group_id
        self.is_relay = msg.is_relay or msg.role in ('root_relay', 'relay')

        self.is_leader = (
            msg.role in ('leader', 'group_leader')
            and msg.leader_id == self.robot_name
            and not self.is_relay
        )

        if (
            (old_leader and self.current_leader_id != old_leader)
            or (old_group and self.group_id != old_group)
            or (old_role and self.current_role != old_role)
        ):
            self._reset_for_new_leader(old_leader, self.current_leader_id)

    def formation_ready_cb(self, msg: Bool):
        # Optional old formation gate; normally disabled in relay-tree mode.
        if not self.require_formation_ready:
            return

        if not msg.data:
            self.formation_done = False
            self.startup_released = False
            return

        if self.formation_done:
            return

        order = self.compute_physical_chain_order()

        if self.lock_chain_order and self.robot_name in order:
            self.locked_order = order
            self._log_order(order, locked=True)
            self._publish_chain_order(order)

        self.formation_done = True
        self.startup_released = False
        self.get_logger().info(f'[{self.robot_name}] chain following active')

    def _reset_for_new_leader(self, old_leader: str, new_leader: str):
        self.locked_order = []
        self.last_logged_order = []
        self.formation_done = False
        self.startup_released = False
        self.group_join_ns = self.get_clock().now().nanoseconds
        self.get_logger().info(
            f'[{self.robot_name}] leader changed: {old_leader} -> {new_leader}'
        )

    def control_loop(self):
        if not self.have_self_state:
            self.publish_cmd(0.0, 0.0)
            return

        if self.mission_mode == 'stop':
            self.last_linear_cmd = 0.0
            self.last_angular_cmd = 0.0
            self.publish_cmd(0.0, 0.0)
            self._log_mission_hold()
            return

        if self.is_relay or self.current_role in ('root_relay', 'relay'):
            self.publish_cmd(0.0, 0.0)
            return

        if self.require_formation_ready and not self.formation_done:
            return

        if (
            self.is_leader
            or self.current_role not in ('follower', 'group_follower')
            or not self.current_leader_id
        ):
            return

        order = self.get_chain_order()

        if not order or self.robot_name not in order:
            self.publish_cmd(0.0, 0.0)
            return

        self._publish_chain_order_periodically(order)

        predecessor = self.get_predecessor()

        if predecessor is None:
            self.publish_cmd(0.0, 0.0)
            return

        released, wait_cmd = self.startup_release_allowed(order, predecessor)
        if not released:
            self.publish_cmd(*wait_cmd)
            return

        guard = self.predecessor_guard(predecessor)
        if guard is not None:
            self.publish_cmd(*guard)
            return

        target = self.get_predecessor_target(predecessor)

        if target is None:
            linear, angular = self.compute_spacing_hold_command(predecessor)
        else:
            tx, ty, distance_to_predecessor = target
            linear, angular = self.compute_control(tx, ty)
            linear = self.apply_gap_speed_control(linear, distance_to_predecessor, predecessor)

        linear *= self.robot_collision_scale(predecessor)

        linear = max(self.too_close_reverse_speed, min(self.max_linear_speed, linear))
        angular = max(-self.max_angular_speed, min(self.max_angular_speed, angular))

        self.publish_cmd(linear, angular)

    def _log_mission_hold(self):
        now_ns = self.get_clock().now().nanoseconds

        if now_ns - self.last_mission_log_ns < 3_000_000_000:
            return

        self.last_mission_log_ns = now_ns
        self.get_logger().info(
            f'[{self.robot_name}] path follower holding due to mission mode: '
            f'{self.mission_mode}'
        )

    def get_chain_order(self) -> List[str]:
        # Returns the active chain order for this robot's current group.
        if self.lock_chain_order and self.locked_order:
            return self.locked_order

        if self.chain_order_mode == 'explicit':
            order = self.compute_explicit_chain_order()
        elif self.chain_order_mode == 'robot_name':
            order = self.compute_robot_name_chain_order()
        else:
            order = self.compute_physical_chain_order()

        if not order:
            return []

        self._log_order(order, locked=False)
        return order

    def compute_explicit_chain_order(self) -> List[str]:
        leader_name = self.current_leader_id

        if not leader_name:
            return []

        valid_names = set()
        for robot in self.other_robots.values():
            if (
                robot.active
                and robot.group_id == self.group_id
                and not robot.is_relay
                and robot.role not in ('root_relay', 'relay')
                and (
                    robot.leader_id == leader_name
                    or robot.robot_name == leader_name
                )
            ):
                valid_names.add(robot.robot_name)

        if not valid_names:
            return []

        ordered = [
            name for name in self.explicit_chain_order
            if name in valid_names
        ]

        missing = sorted(
            list(valid_names - set(ordered)),
            key=self.robot_number,
        )
        ordered.extend(missing)

        if leader_name in ordered:
            ordered.remove(leader_name)

        return [leader_name] + ordered

    def compute_robot_name_chain_order(self) -> List[str]:
        leader_name = self.current_leader_id

        if not leader_name:
            return []

        robots = [
            r for r in self.other_robots.values()
            if (
                r.active
                and r.group_id == self.group_id
                and not r.is_relay
                and r.role not in ('root_relay', 'relay')
                and (
                    r.leader_id == leader_name
                    or r.robot_name == leader_name
                )
            )
        ]

        if not robots:
            return []

        names = sorted(
            [r.robot_name for r in robots],
            key=self.robot_number,
        )

        if leader_name in names:
            names.remove(leader_name)

        return [leader_name] + names

    def compute_physical_chain_order(self) -> List[str]:
        leader_name = self.current_leader_id

        if not leader_name:
            return []

        leader = self.other_robots.get(leader_name)

        if leader is None:
            return []

        robots = [
            r for r in self.other_robots.values()
            if (
                r.active
                and r.group_id == self.group_id
                and not r.is_relay
                and r.role not in ('root_relay', 'relay')
                and (
                    r.leader_id == leader_name
                    or r.robot_name == leader_name
                )
            )
        ]

        if not robots:
            return []

        fx = math.cos(leader.yaw)
        fy = math.sin(leader.yaw)

        def along_leader_axis(robot: RobotState) -> float:
            dx = robot.x - leader.x
            dy = robot.y - leader.y
            return dx * fx + dy * fy

        ordered = sorted(
            robots,
            key=lambda r: (along_leader_axis(r), -self.robot_number(r.robot_name)),
            reverse=True,
        )
        names = [r.robot_name for r in ordered]

        if leader_name in names:
            names.remove(leader_name)

        return [leader_name] + names

    def get_predecessor(self) -> Optional[RobotState]:
        order = self.get_chain_order()

        if not order or self.robot_name not in order:
            return None

        index = order.index(self.robot_name)

        if index == 0:
            return None

        predecessor_name = order[index - 1]
        predecessor = self.other_robots.get(predecessor_name)

        if (
            predecessor is None
            or not predecessor.active
            or predecessor.is_relay
            or predecessor.role in ('root_relay', 'relay')
        ):
            return None

        return predecessor

    def _publish_chain_order(self, order: List[str]):
        msg = String()
        msg.data = json.dumps(order)
        self.chain_order_pub.publish(msg)
        self.last_chain_publish_ns = self.get_clock().now().nanoseconds

    def _publish_chain_order_periodically(self, order: List[str]):
        now_ns = self.get_clock().now().nanoseconds

        if now_ns - self.last_chain_publish_ns < 500_000_000:
            return

        self._publish_chain_order(order)

    def _log_order(self, order: List[str], locked: bool):
        if order == self.last_logged_order:
            return

        self.last_logged_order = list(order)
        label = 'locked chain order' if locked else 'chain order'
        self.get_logger().info(f'[{self.robot_name}] {label}: {" -> ".join(order)}')

    def robot_number(self, name: str) -> int:
        match = re.search(r'\d+', name)
        return int(match.group()) if match else 999

    def startup_release_allowed(
        self,
        order: List[str],
        predecessor: RobotState,
    ) -> Tuple[bool, Tuple[float, float]]:
        # Releases followers one by one after their predecessor has moved.
        if self.startup_released:
            return True, (0.0, 0.0)

        if not self.sequential_start_enabled:
            return self.startup_space_available(predecessor), (0.0, 0.0)

        if self.robot_name not in order:
            return False, (0.0, 0.0)

        my_index = order.index(self.robot_name)

        if my_index <= 0:
            self.startup_released = True
            return True, (0.0, 0.0)

        moved, moved_dist = self.predecessor_has_moved(predecessor)
        if not moved:
            self._log_startup_wait(
                f'waiting {predecessor.robot_name} moved '
                f'{moved_dist:.2f}/{self.sequential_predecessor_move_m:.2f}m'
            )
            return False, (0.0, 0.0)

        now_ns = self.get_clock().now().nanoseconds
        elapsed_sec = (now_ns - self.group_join_ns) / 1e9
        required_delay = min(my_index * self.sequential_slot_delay_sec, 1.5)

        if elapsed_sec < required_delay:
            self._log_startup_wait(
                f'waiting slot delay {elapsed_sec:.1f}/{required_delay:.1f}s'
            )
            return False, (0.0, 0.0)

        gap = math.hypot(
            predecessor.x - self.world_x,
            predecessor.y - self.world_y,
        )
        required_gap = self.desired_follow_distance_m * self.startup_gap_ratio

        if gap < required_gap:
            self._log_startup_wait(
                f'waiting gap to {predecessor.robot_name}: '
                f'{gap:.2f}/{required_gap:.2f}m'
            )
            return False, (0.0, 0.0)

        if self.sequential_prealign_enabled:
            aligned, angular = self.self_aligned_for_start(predecessor)
            if not aligned:
                self._log_startup_wait(f'pre-aligning behind {predecessor.robot_name}')
                return False, (0.0, angular)

        self.startup_released = True
        self.get_logger().info(
            f'[{self.robot_name}] sequential startup released: '
            f'slot={my_index}, predecessor={predecessor.robot_name}, '
            f'predecessor_moved={moved_dist:.2f}m, gap={gap:.2f}m'
        )
        return True, (0.0, 0.0)

    def predecessor_has_moved(self, predecessor: RobotState) -> Tuple[bool, float]:
        start = self.initial_robot_positions.get(predecessor.robot_name)

        if start is None:
            return False, 0.0

        sx, sy = start
        moved_dist = math.hypot(predecessor.x - sx, predecessor.y - sy)

        return moved_dist >= self.sequential_predecessor_move_m, moved_dist

    def self_aligned_for_start(
        self,
        predecessor: RobotState,
    ) -> Tuple[bool, float]:
        tx = predecessor.x - self.desired_follow_distance_m * math.cos(predecessor.yaw)
        ty = predecessor.y - self.desired_follow_distance_m * math.sin(predecessor.yaw)

        desired_heading = math.atan2(ty - self.world_y, tx - self.world_x)
        heading_error = normalize_angle(desired_heading - self.yaw)

        aligned = abs(heading_error) <= self.sequential_self_alignment_tolerance_rad
        angular = self.sequential_prealign_angular_gain * heading_error
        angular = max(-self.max_angular_speed, min(self.max_angular_speed, angular))

        return aligned, angular

    def _log_startup_wait(self, reason: str):
        now_ns = self.get_clock().now().nanoseconds

        if now_ns - self.last_startup_wait_log_ns < 2_000_000_000:
            return

        self.last_startup_wait_log_ns = now_ns
        self.get_logger().info(f'[{self.robot_name}] sequential startup: {reason}')

    def startup_space_available(self, predecessor: RobotState) -> bool:
        # Simple non-sequential startup gate based only on predecessor gap.
        if self.startup_released:
            return True

        gap = math.hypot(
            predecessor.x - self.world_x,
            predecessor.y - self.world_y,
        )

        required_gap = self.desired_follow_distance_m * self.startup_gap_ratio

        if gap >= required_gap:
            self.startup_released = True
            self.get_logger().info(
                f'[{self.robot_name}] startup released: '
                f'gap={gap:.2f}m required={required_gap:.2f}m'
            )
            return True

        return False

    def get_predecessor_target(
        self,
        predecessor: RobotState,
    ) -> Optional[Tuple[float, float, float]]:
        # Targets a slot behind the predecessor using the predecessor heading.
        dx = predecessor.x - self.world_x
        dy = predecessor.y - self.world_y
        distance = math.hypot(dx, dy)

        min_distance = self.desired_follow_distance_m - self.follow_deadband_m

        if distance < min_distance:
            return None

        tx = predecessor.x - self.desired_follow_distance_m * math.cos(predecessor.yaw)
        ty = predecessor.y - self.desired_follow_distance_m * math.sin(predecessor.yaw)

        return tx, ty, distance

    def compute_control(self, tx: float, ty: float) -> Tuple[float, float]:
        # Computes velocity toward the target slot.
        dx = tx - self.world_x
        dy = ty - self.world_y
        distance_to_target = math.hypot(dx, dy)

        desired_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(desired_heading - self.yaw)

        linear = min(self.linear_gain * distance_to_target, self.max_linear_speed)

        abs_err = abs(heading_error)

        if abs_err > 1.30:
            linear *= 0.35
        elif abs_err > 0.85:
            linear *= 0.60
        elif abs_err > 0.50:
            linear *= 0.85

        if distance_to_target < 0.04:
            linear = 0.0

        angular = self.angular_gain * heading_error

        return linear, angular

    def compute_spacing_hold_command(
        self,
        predecessor: RobotState,
    ) -> Tuple[float, float]:
        # Smoothly holds spacing instead of repeatedly stopping and starting.
        dx = predecessor.x - self.world_x
        dy = predecessor.y - self.world_y
        distance = math.hypot(dx, dy)
        gap_error = distance - self.desired_follow_distance_m

        predecessor_speed = max(0.0, float(predecessor.linear_speed))

        if abs(gap_error) <= self.hold_gap_deadband_m and predecessor_speed < 0.025:
            return 0.0, 0.0

        if self.speed_match_enabled and gap_error >= -self.follow_deadband_m:
            matched = predecessor_speed * self.speed_match_gain

            if predecessor_speed > 0.025:
                matched = max(self.min_creep_speed, matched)

            linear = min(self.max_linear_speed, matched)
        else:
            linear = 0.0

        angle_to_predecessor = math.atan2(dy, dx)
        heading_error = normalize_angle(angle_to_predecessor - self.yaw)

        if abs(heading_error) < self.hold_heading_deadband_rad:
            angular = 0.0
        else:
            angular = 0.45 * self.angular_gain * heading_error

        return linear, angular

    def apply_gap_speed_control(
        self,
        linear: float,
        distance_to_predecessor: float,
        predecessor: RobotState,
    ) -> float:
        # Adjusts forward speed based on the current gap to the predecessor.
        gap_error = distance_to_predecessor - self.desired_follow_distance_m
        predecessor_speed = max(0.0, float(predecessor.linear_speed))

        if gap_error < -self.follow_deadband_m:
            return 0.0

        if gap_error < 0.0:
            if self.speed_match_enabled and predecessor_speed > 0.025:
                return min(linear, max(self.min_creep_speed, predecessor_speed * 0.85))
            return min(linear, self.max_linear_speed * 0.25)

        if gap_error < self.follow_deadband_m:
            if self.speed_match_enabled and predecessor_speed > 0.025:
                return min(
                    self.max_linear_speed * 0.70,
                    max(self.min_creep_speed, predecessor_speed * self.speed_match_gain),
                )

            return min(linear, self.max_linear_speed * 0.35)

        if distance_to_predecessor > self.very_far_gap_m:
            return min(
                self.max_linear_speed,
                max(linear, self.min_linear_speed_when_far) * self.catchup_speed_boost,
            )

        if distance_to_predecessor > self.far_gap_m:
            return min(
                self.max_linear_speed,
                max(linear, self.min_linear_speed_when_far),
            )

        return linear

    def predecessor_guard(
        self,
        predecessor: Optional[RobotState],
    ) -> Optional[Tuple[float, float]]:
        # Prevents the follower from driving into its predecessor.
        if predecessor is None:
            return None

        dx = predecessor.x - self.world_x
        dy = predecessor.y - self.world_y
        distance = math.hypot(dx, dy)

        if distance >= self.min_robot_distance_m:
            return None

        heading_error = normalize_angle(math.atan2(dy, dx) - self.yaw)
        angular = self.angular_gain * heading_error
        angular = max(-self.max_angular_speed, min(self.max_angular_speed, angular))

        if abs(heading_error) < 0.8:
            return self.too_close_reverse_speed, angular

        return 0.0, angular

    def robot_collision_scale(self, predecessor: Optional[RobotState]) -> float:
        # Slows this robot down when another nearby robot is too close.
        predecessor_name = predecessor.robot_name if predecessor else None
        nearest = 999.0

        for name, robot in self.other_robots.items():
            if name == self.robot_name or not robot.active:
                continue

            if name == predecessor_name:
                continue

            d = math.hypot(robot.x - self.world_x, robot.y - self.world_y)
            nearest = min(nearest, d)

        if nearest < self.min_robot_distance_m:
            return 0.0

        if nearest < self.slow_robot_distance_m:
            span = self.slow_robot_distance_m - self.min_robot_distance_m

            if span <= 0.01:
                return 0.0

            return max(0.20, (nearest - self.min_robot_distance_m) / span)

        return 1.0

    def smooth_value(self, old: float, new: float, alpha: float) -> float:
        alpha = max(0.0, min(1.0, alpha))
        return (alpha * new) + ((1.0 - alpha) * old)

    def publish_cmd(self, linear: float, angular: float):
        # Smooths and publishes the final velocity command.
        self.last_linear_cmd = self.smooth_value(
            self.last_linear_cmd,
            linear,
            self.command_smoothing_alpha_linear,
        )
        self.last_angular_cmd = self.smooth_value(
            self.last_angular_cmd,
            angular,
            self.command_smoothing_alpha_angular,
        )

        if abs(linear) < 1e-4 and abs(self.last_linear_cmd) < 0.01:
            self.last_linear_cmd = 0.0
        if abs(angular) < 1e-4 and abs(self.last_angular_cmd) < 0.03:
            self.last_angular_cmd = 0.0

        msg = Twist()
        msg.linear.x = self.last_linear_cmd
        msg.angular.z = self.last_angular_cmd
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        msg = Twist()
        node.cmd_pub.publish(msg)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()