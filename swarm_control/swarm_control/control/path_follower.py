import json
import math
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
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

        self.get_logger().info(f'[{self.robot_name}] path_follower started')

    def _declare_parameters(self):
        self.declare_parameter('robot_name', 'robot2')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel_raw')
        self.declare_parameter('path_topic', '/swarm/leader_path')

        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)

        self.declare_parameter('lookahead_m', 0.25)
        self.declare_parameter('goal_tolerance_m', 0.06)
        self.declare_parameter('min_path_length_m', 0.25)

        self.declare_parameter('desired_follow_distance_m', 1.15)
        self.declare_parameter('follow_deadband_m', 0.25)

        self.declare_parameter('max_linear_speed', 0.12)
        self.declare_parameter('min_linear_speed_when_far', 0.035)
        self.declare_parameter('max_angular_speed', 1.10)
        self.declare_parameter('linear_gain', 0.55)
        self.declare_parameter('angular_gain', 1.55)

        self.declare_parameter('min_robot_distance_m', 0.50)
        self.declare_parameter('slow_robot_distance_m', 0.85)
        self.declare_parameter('too_close_reverse_speed', -0.02)

        self.declare_parameter('far_gap_m', 1.65)
        self.declare_parameter('very_far_gap_m', 2.30)
        self.declare_parameter('catchup_speed_boost', 1.15)

        self.declare_parameter('startup_delay_per_slot_sec', 0.40)
        self.declare_parameter('lock_chain_order', True)
        self.declare_parameter('fallback_to_predecessor', True)

        self.declare_parameter('cross_track_gain', 1.10)
        self.declare_parameter('max_cross_track_correction_rad', 0.35)

        self.declare_parameter('line_hold_enabled', True)
        self.declare_parameter('line_hold_gain', 0.90)
        self.declare_parameter('line_hold_integral_gain', 0.10)
        self.declare_parameter('line_hold_integral_limit', 0.35)
        self.declare_parameter('line_hold_start_delay_sec', 5.0)
        self.declare_parameter('line_hold_max_correction_rad', 0.60)

        self.declare_parameter('resync_enabled', True)
        self.declare_parameter('resync_lateral_error_m', 0.10)
        self.declare_parameter('resync_release_error_m', 0.04)
        self.declare_parameter('resync_angular_boost', 2.50)
        self.declare_parameter('resync_speed_scale', 0.40)

    def _read_parameters(self):
        self.robot_name = self.get_parameter('robot_name').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.path_topic = self.get_parameter('path_topic').value

        self.spawn_x = float(self.get_parameter('spawn_x').value)
        self.spawn_y = float(self.get_parameter('spawn_y').value)

        self.lookahead_m = float(self.get_parameter('lookahead_m').value)
        self.goal_tolerance_m = float(self.get_parameter('goal_tolerance_m').value)
        self.min_path_length_m = float(self.get_parameter('min_path_length_m').value)

        self.desired_follow_distance_m = float(self.get_parameter('desired_follow_distance_m').value)
        self.follow_deadband_m = float(self.get_parameter('follow_deadband_m').value)

        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.min_linear_speed_when_far = float(self.get_parameter('min_linear_speed_when_far').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.linear_gain = float(self.get_parameter('linear_gain').value)
        self.angular_gain = float(self.get_parameter('angular_gain').value)

        self.min_robot_distance_m = float(self.get_parameter('min_robot_distance_m').value)
        self.slow_robot_distance_m = float(self.get_parameter('slow_robot_distance_m').value)
        self.too_close_reverse_speed = float(self.get_parameter('too_close_reverse_speed').value)

        self.far_gap_m = float(self.get_parameter('far_gap_m').value)
        self.very_far_gap_m = float(self.get_parameter('very_far_gap_m').value)
        self.catchup_speed_boost = float(self.get_parameter('catchup_speed_boost').value)

        self.startup_delay_per_slot_sec = float(self.get_parameter('startup_delay_per_slot_sec').value)
        self.lock_chain_order = bool(self.get_parameter('lock_chain_order').value)
        self.fallback_to_predecessor = bool(self.get_parameter('fallback_to_predecessor').value)

        self.cross_track_gain = float(self.get_parameter('cross_track_gain').value)
        self.max_cross_track_correction_rad = float(self.get_parameter('max_cross_track_correction_rad').value)

        self.line_hold_enabled = bool(self.get_parameter('line_hold_enabled').value)
        self.line_hold_gain = float(self.get_parameter('line_hold_gain').value)
        self.line_hold_integral_gain = float(self.get_parameter('line_hold_integral_gain').value)
        self.line_hold_integral_limit = float(self.get_parameter('line_hold_integral_limit').value)
        self.line_hold_start_delay_sec = float(self.get_parameter('line_hold_start_delay_sec').value)
        self.line_hold_max_correction_rad = float(self.get_parameter('line_hold_max_correction_rad').value)

        self.resync_enabled = bool(self.get_parameter('resync_enabled').value)
        self.resync_lateral_error_m = float(self.get_parameter('resync_lateral_error_m').value)
        self.resync_release_error_m = float(self.get_parameter('resync_release_error_m').value)
        self.resync_angular_boost = float(self.get_parameter('resync_angular_boost').value)
        self.resync_speed_scale = float(self.get_parameter('resync_speed_scale').value)

    def _init_state(self):
        self.world_x = 0.0
        self.world_y = 0.0
        self.yaw = 0.0
        self.have_self_state = False

        self.path: Optional[Path] = None
        self._path_cache = None

        self.formation_done = False
        self.path_follow_start_time_ns = None

        self.current_role = 'follower'
        self.current_leader_id = ''
        self.is_leader = False

        self.other_robots: Dict[str, RobotState] = {}
        self.locked_order: List[str] = []
        self.last_logged_order: List[str] = []

        self.line_hold_integral = 0.0
        self.resync_active = False

    def _init_ros_interfaces(self):
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.chain_order_pub = self.create_publisher(String, '/swarm/chain_order', 10)

        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(Path, self.path_topic, self.path_callback, 10)
        self.create_subscription(RobotState, '/swarm/robot_states', self.state_callback, 10)
        self.create_subscription(Bool, 'formation_ready', self.formation_ready_cb, 10)

        self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg: Odometry):
        # Fallback only, before /swarm/robot_states for this robot arrives.
        if self.have_self_state:
            return

        self.world_x = self.spawn_x + msg.pose.pose.position.x
        self.world_y = self.spawn_y + msg.pose.pose.position.y
        self.yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def path_callback(self, msg: Path):
        self.path = msg
        self._path_cache = None

    def state_callback(self, msg: RobotState):
        self.other_robots[msg.robot_name] = msg

        if msg.robot_name != self.robot_name:
            return

        self.world_x = msg.x
        self.world_y = msg.y
        self.yaw = msg.yaw
        self.have_self_state = True

        old_leader = self.current_leader_id

        self.current_role = msg.role
        self.current_leader_id = msg.leader_id
        self.is_leader = msg.role == 'leader' and msg.leader_id == self.robot_name

        if old_leader and self.current_leader_id != old_leader:
            self.locked_order = []
            self.last_logged_order = []
            self.formation_done = False
            self.path_follow_start_time_ns = None
            self.line_hold_integral = 0.0
            self.resync_active = False
            self.get_logger().info(
                f'[{self.robot_name}] leader changed: {old_leader} -> {self.current_leader_id}'
            )

    def formation_ready_cb(self, msg: Bool):
        if msg.data and not self.formation_done:
            order = self.compute_physical_chain_order()

            if self.lock_chain_order and self.robot_name in order:
                self.locked_order = order
                self._log_order(order, locked=True)
                self._publish_chain_order(order)

            self.formation_done = True
            self.path_follow_start_time_ns = self.get_clock().now().nanoseconds
            self.line_hold_integral = 0.0
            self.resync_active = False
            self.get_logger().info(f'[{self.robot_name}] path following active')

    def _publish_chain_order(self, order: List[str]):
        msg = String()
        msg.data = json.dumps(order)
        self.chain_order_pub.publish(msg)

    def get_chain_order(self) -> List[str]:
        if self.lock_chain_order and self.locked_order:
            return self.locked_order

        order = self.compute_physical_chain_order()

        if order:
            self._log_order(order, locked=False)

        return order

    def compute_physical_chain_order(self) -> List[str]:
        leader_name = self.current_leader_id

        if not leader_name:
            return []

        leader = self.other_robots.get(leader_name)

        if leader is None:
            return []

        robots = [
            r for r in self.other_robots.values()
            if r.active and r.leader_id == leader_name
        ]

        if not robots:
            return []

        fx = math.cos(leader.yaw)
        fy = math.sin(leader.yaw)

        def along_leader_axis(robot: RobotState) -> float:
            dx = robot.x - leader.x
            dy = robot.y - leader.y
            return dx * fx + dy * fy

        ordered = sorted(robots, key=along_leader_axis, reverse=True)
        names = [r.robot_name for r in ordered]

        if leader_name in names:
            names.remove(leader_name)

        return [leader_name] + names

    def _log_order(self, order: List[str], locked: bool):
        if order == self.last_logged_order:
            return

        self.last_logged_order = list(order)
        label = 'locked chain order' if locked else 'chain order'
        # self.get_logger().info(f'[{self.robot_name}] {label}: {" -> ".join(order)}')

    def get_my_index(self) -> Optional[int]:
        order = self.get_chain_order()

        if self.robot_name not in order:
            return None

        return order.index(self.robot_name)

    def get_predecessor(self) -> Optional[RobotState]:
        order = self.get_chain_order()

        if self.robot_name not in order:
            return None

        index = order.index(self.robot_name)

        if index == 0:
            return None

        predecessor_name = order[index - 1]
        predecessor = self.other_robots.get(predecessor_name)

        if predecessor is None or not predecessor.active:
            return None

        return predecessor

    def _get_path_lengths(self):
        if self.path is None or len(self.path.poses) < 2:
            return None, None

        path_id = id(self.path)

        if self._path_cache is not None and self._path_cache[0] == path_id:
            _, total, cumulative = self._path_cache
            return total, cumulative

        cumulative = [0.0]
        total = 0.0

        for i in range(1, len(self.path.poses)):
            p0 = self.path.poses[i - 1].pose.position
            p1 = self.path.poses[i].pose.position
            seg = math.hypot(p1.x - p0.x, p1.y - p0.y)
            total += seg
            cumulative.append(total)

        self._path_cache = (path_id, total, cumulative)
        return total, cumulative

    def closest_path_s(self, cumulative: List[float], x: float, y: float) -> float:
        best_dist_sq = float('inf')
        best_s = 0.0

        for i in range(1, len(self.path.poses)):
            p0 = self.path.poses[i - 1].pose.position
            p1 = self.path.poses[i].pose.position

            vx = p1.x - p0.x
            vy = p1.y - p0.y
            wx = x - p0.x
            wy = y - p0.y

            seg_len_sq = vx * vx + vy * vy

            if seg_len_sq <= 1e-9:
                t = 0.0
            else:
                t = (wx * vx + wy * vy) / seg_len_sq
                t = max(0.0, min(1.0, t))

            px = p0.x + t * vx
            py = p0.y + t * vy
            dist_sq = (x - px) ** 2 + (y - py) ** 2

            if dist_sq < best_dist_sq:
                best_dist_sq = dist_sq
                best_s = cumulative[i - 1] + t * math.sqrt(seg_len_sq)

        return best_s

    def sample_path(self, cumulative: List[float], total: float, s: float):
        s = max(0.0, min(total, s))

        if s <= 0.0:
            p = self.path.poses[0].pose.position
            return p.x, p.y

        if s >= total:
            p = self.path.poses[-1].pose.position
            return p.x, p.y

        for i in range(1, len(cumulative)):
            if cumulative[i] >= s:
                p0 = self.path.poses[i - 1].pose.position
                p1 = self.path.poses[i].pose.position

                s0 = cumulative[i - 1]
                s1 = cumulative[i]
                ratio = 0.0 if s1 <= s0 else (s - s0) / (s1 - s0)

                return (
                    p0.x + ratio * (p1.x - p0.x),
                    p0.y + ratio * (p1.y - p0.y),
                )

        p = self.path.poses[-1].pose.position
        return p.x, p.y

    def sample_path_yaw(self, cumulative: List[float], total: float, s: float) -> float:
        s1 = max(0.0, s - 0.15)
        s2 = min(total, s + 0.15)

        p1 = self.sample_path(cumulative, total, s1)
        p2 = self.sample_path(cumulative, total, s2)

        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]

        if math.hypot(dx, dy) < 0.01:
            return self.yaw

        return math.atan2(dy, dx)

    def compute_lateral_error_to_path(self, cumulative: List[float], total: float, s_self: float):
        cx, cy = self.sample_path(cumulative, total, s_self)
        path_yaw = self.sample_path_yaw(cumulative, total, s_self)

        nx = -math.sin(path_yaw)
        ny = math.cos(path_yaw)

        lateral_error = (self.world_x - cx) * nx + (self.world_y - cy) * ny

        return lateral_error, path_yaw

    def get_arc_length_target(self, predecessor: RobotState):
        total, cumulative = self._get_path_lengths()

        if total is None or total < self.min_path_length_m:
            return None

        # Corrected: predecessor-based arc length.
        # This keeps the visible chain together instead of each robot independently
        # chasing a delayed leader-path slot.
        s_pred = self.closest_path_s(cumulative, predecessor.x, predecessor.y)
        s_target = s_pred - self.desired_follow_distance_m

        if s_target < 0.0:
            return None

        s_self = self.closest_path_s(cumulative, self.world_x, self.world_y)

        lateral_error, path_yaw_at_self = self.compute_lateral_error_to_path(
            cumulative,
            total,
            s_self,
        )

        s_lookahead = max(0.0, min(total, s_target + self.lookahead_m))

        tx, ty = self.sample_path(cumulative, total, s_lookahead)
        path_yaw = self.sample_path_yaw(cumulative, total, s_lookahead)

        arc_gap = s_target - s_self

        return (
            tx,
            ty,
            path_yaw,
            arc_gap,
            s_pred,
            s_target,
            lateral_error,
            path_yaw_at_self,
        )

    def get_predecessor_target(self, predecessor: RobotState):
        dx = predecessor.x - self.world_x
        dy = predecessor.y - self.world_y
        distance = math.hypot(dx, dy)

        min_distance = self.desired_follow_distance_m - self.follow_deadband_m

        if distance < min_distance:
            return None

        angle = math.atan2(dy, dx)

        tx = predecessor.x - self.desired_follow_distance_m * math.cos(angle)
        ty = predecessor.y - self.desired_follow_distance_m * math.sin(angle)

        return (
            tx,
            ty,
            angle,
            distance - self.desired_follow_distance_m,
            None,
            None,
            0.0,
            angle,
        )

    def control_loop(self):
        if not self.formation_done:
            return

        if self.is_leader or self.current_role != 'follower' or not self.current_leader_id:
            return

        if not self.startup_delay_done():
            self.publish_cmd(0.0, 0.0)
            return

        order = self.get_chain_order()

        if self.robot_name not in order:
            self.publish_cmd(0.0, 0.0)
            return

        index = order.index(self.robot_name)

        if index == 0:
            return

        predecessor = self.get_predecessor()

        guard = self.predecessor_guard(predecessor)

        if guard is not None:
            self.publish_cmd(*guard)
            return

        result = None

        if predecessor is not None:
            result = self.get_arc_length_target(predecessor)

        if result is None and self.fallback_to_predecessor and predecessor is not None:
            result = self.get_predecessor_target(predecessor)

        if result is None:
            self.publish_cmd(0.0, 0.0)
            return

        (
            tx,
            ty,
            path_yaw,
            arc_gap,
            s_pred,
            s_target,
            lateral_error,
            path_yaw_at_self,
        ) = result

        linear, angular = self.compute_control(tx, ty, path_yaw, lateral_error)

        angular = self.apply_line_hold_correction(
            angular,
            lateral_error,
            path_yaw_at_self,
        )

        linear, angular = self.apply_resync_correction(
            linear,
            angular,
            lateral_error,
        )

        linear = self.apply_arc_gap_speed_control(linear, arc_gap, predecessor)
        linear *= self.robot_collision_scale(predecessor)

        linear = max(self.too_close_reverse_speed, min(self.max_linear_speed, linear))
        angular = max(-self.max_angular_speed, min(self.max_angular_speed, angular))

        self.publish_cmd(linear, angular)

    def startup_delay_done(self) -> bool:
        if self.startup_delay_per_slot_sec <= 0.0:
            return True

        if self.path_follow_start_time_ns is None:
            return False

        index = self.get_my_index()

        if index is None:
            return False

        follower_index = max(0, index - 1)
        required_delay = follower_index * self.startup_delay_per_slot_sec

        elapsed = (self.get_clock().now().nanoseconds - self.path_follow_start_time_ns) / 1e9

        return elapsed >= required_delay

    def compute_control(self, tx: float, ty: float, path_yaw: float, lateral_error: float):
        dx = tx - self.world_x
        dy = ty - self.world_y
        distance = math.hypot(dx, dy)

        if distance < self.goal_tolerance_m:
            return 0.0, 0.0

        pursuit_heading = math.atan2(dy, dx)

        correction = math.atan2(
            self.cross_track_gain * lateral_error,
            max(self.lookahead_m, 0.05),
        )

        correction = max(
            -self.max_cross_track_correction_rad,
            min(self.max_cross_track_correction_rad, correction),
        )

        corrected_path_heading = normalize_angle(path_yaw - correction)

        x = 0.45 * math.cos(pursuit_heading) + 0.55 * math.cos(corrected_path_heading)
        y = 0.45 * math.sin(pursuit_heading) + 0.55 * math.sin(corrected_path_heading)

        desired_heading = math.atan2(y, x)
        heading_error = normalize_angle(desired_heading - self.yaw)

        linear = min(self.linear_gain * distance, self.max_linear_speed)

        abs_err = abs(heading_error)

        if abs_err > 1.30:
            linear *= 0.20
        elif abs_err > 0.85:
            linear *= 0.45
        elif abs_err > 0.50:
            linear *= 0.70

        angular = self.angular_gain * heading_error

        return linear, angular

    def apply_line_hold_correction(self, angular: float, lateral_error: float, path_yaw: float):
        if not self.line_hold_enabled:
            return angular

        if self.path_follow_start_time_ns is None:
            return angular

        elapsed = (self.get_clock().now().nanoseconds - self.path_follow_start_time_ns) / 1e9

        if elapsed < self.line_hold_start_delay_sec:
            return angular

        self.line_hold_integral += lateral_error * 0.1
        self.line_hold_integral = max(
            -self.line_hold_integral_limit,
            min(self.line_hold_integral_limit, self.line_hold_integral),
        )

        correction = (
            self.line_hold_gain * lateral_error
            + self.line_hold_integral_gain * self.line_hold_integral
        )

        correction = max(
            -self.line_hold_max_correction_rad,
            min(self.line_hold_max_correction_rad, correction),
        )

        return angular - correction

    def apply_resync_correction(self, linear: float, angular: float, lateral_error: float):
        if not self.resync_enabled:
            return linear, angular

        abs_error = abs(lateral_error)

        if not self.resync_active and abs_error >= self.resync_lateral_error_m:
            self.resync_active = True
            self.get_logger().warn(
                f'[{self.robot_name}] resync active: lateral_error={lateral_error:.2f}m'
            )

        if self.resync_active and abs_error <= self.resync_release_error_m:
            self.resync_active = False
            self.get_logger().info(f'[{self.robot_name}] resync released')

        if not self.resync_active:
            return linear, angular

        linear *= self.resync_speed_scale
        angular *= self.resync_angular_boost

        return linear, angular

    def apply_arc_gap_speed_control(
        self,
        linear: float,
        arc_gap: float,
        predecessor: Optional[RobotState],
    ) -> float:
        if predecessor is not None:
            euclid_gap = math.hypot(
                predecessor.x - self.world_x,
                predecessor.y - self.world_y,
            )

            target_gap = self.desired_follow_distance_m
            gap_error = euclid_gap - target_gap

            if gap_error < -self.follow_deadband_m:
                return max(0.0, min(linear, self.max_linear_speed * 0.20))

            if gap_error < 0.0:
                return max(0.0, min(linear, self.max_linear_speed * 0.40))

        if arc_gap is None:
            return linear

        deadband = self.follow_deadband_m

        if arc_gap < -deadband:
            return max(0.0, min(linear, self.max_linear_speed * 0.25))

        if arc_gap < deadband:
            return max(
                self.min_linear_speed_when_far,
                min(linear, self.max_linear_speed * 0.65),
            )

        if arc_gap > self.very_far_gap_m:
            return min(
                self.max_linear_speed,
                max(linear, self.min_linear_speed_when_far) * self.catchup_speed_boost,
            )

        if arc_gap > self.far_gap_m:
            return min(
                self.max_linear_speed,
                max(linear, self.min_linear_speed_when_far),
            )

        return linear

    def predecessor_guard(self, predecessor: Optional[RobotState]):
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

    def publish_cmd(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()