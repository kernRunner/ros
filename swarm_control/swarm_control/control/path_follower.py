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
    """
    Simplified chain follower.

    Main responsibility:
      - Followers maintain a chain behind their predecessor.
      - robot2 follows the leader.
      - robot3 follows robot2.
      - robot4 follows robot3.
      - No leader-path arc-length sampling.
      - No line-hold controller.
      - No resync controller.

    Why this version is simpler:
      - Formation is based on predecessor spacing, not historical path slots.
      - There is only one steering controller.
      - Followers start earlier using a configurable startup gap ratio.
      - The file still publishes /swarm/chain_order for debug/safety-filter use.
    """

    def __init__(self):
        super().__init__('path_follower')

        self._declare_parameters()
        self._read_parameters()
        self._init_state()
        self._init_ros_interfaces()

        self.get_logger().info(f'[{self.robot_name}] simplified path_follower started')

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------

    def _declare_parameters(self):
        self.declare_parameter('robot_name', 'robot2')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel_raw')

        # Kept for launch-file compatibility. This simplified follower does not
        # subscribe to /swarm/leader_path.
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

        # Followers are released once their predecessor opens enough space.
        # Old code used desired_follow_distance_m * 0.95, which made startup slow.
        self.declare_parameter('startup_gap_ratio', 0.60)

        self.declare_parameter('lock_chain_order', True)

        # Relay-tree mode: assignments from relay_tree_manager are enough to start.
        # Keep this False unless you explicitly want formation_manager to gate movement.
        self.declare_parameter('require_formation_ready', False)

        # Kept for launch-file compatibility, but unused in this simplified file.
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

        # Smooth following instead of start/stop behavior.
        # If the predecessor is moving slowly, followers try to match that speed.
        self.declare_parameter('speed_match_enabled', True)
        self.declare_parameter('speed_match_gain', 0.85)
        self.declare_parameter('min_creep_speed', 0.025)
        self.declare_parameter('command_smoothing_alpha_linear', 0.35)
        self.declare_parameter('command_smoothing_alpha_angular', 0.30)

    def _read_parameters(self):
        self.robot_name = self.get_parameter('robot_name').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

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
        self.lock_chain_order = bool(self.get_parameter('lock_chain_order').value)
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

    # ------------------------------------------------------------------
    # Initialization
    # ------------------------------------------------------------------

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
        self.locked_order: List[str] = []
        self.last_logged_order: List[str] = []

        self.last_chain_publish_ns = 0

        self.last_linear_cmd = 0.0
        self.last_angular_cmd = 0.0

    def _init_ros_interfaces(self):
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.chain_order_pub = self.create_publisher(String, '/swarm/chain_order', 10)

        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(RobotState, '/swarm/robot_states', self.state_callback, 10)
        self.create_subscription(Bool, 'formation_ready', self.formation_ready_cb, 10)

        self.create_timer(0.1, self.control_loop)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def odom_callback(self, msg: Odometry):
        # Fallback only, before /swarm/robot_states for this robot arrives.
        if self.have_self_state:
            return

        self.world_x = self.spawn_x + msg.pose.pose.position.x
        self.world_y = self.spawn_y + msg.pose.pose.position.y
        self.yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def state_callback(self, msg: RobotState):
        self.other_robots[msg.robot_name] = msg

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

        # Relay-tree mode:
        # - group_leader is a leader.
        # - root_relay / relay must never be treated as a moving leader.
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
        # In relay-tree mode the relay_tree_manager assignment is the movement gate.
        # If require_formation_ready is False, ignore formation_manager completely.
        # Otherwise an early/stale formation_ready message can lock a bad chain order
        # while the robots are still side-by-side at spawn.
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
        self.get_logger().info(
            f'[{self.robot_name}] leader changed: {old_leader} -> {new_leader}'
        )

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def control_loop(self):
        if not self.have_self_state:
            self.publish_cmd(0.0, 0.0)
            return

        # Relays are physical breadcrumbs: always stay stopped.
        if self.is_relay or self.current_role in ('root_relay', 'relay'):
            self.publish_cmd(0.0, 0.0)
            return

        # In relay-tree mode the role assignment is the formation gate.
        # The old formation_manager can still be used by setting
        # require_formation_ready:=True.
        if self.require_formation_ready and not self.formation_done:
            return

        if (
            self.is_leader
            or self.current_role not in ('follower', 'group_follower')
            or not self.current_leader_id
        ):
            return

        order = self.get_chain_order()

        if self.robot_name not in order:
            self.publish_cmd(0.0, 0.0)
            return

        self._publish_chain_order_periodically(order)

        predecessor = self.get_predecessor()

        if predecessor is None:
            self.publish_cmd(0.0, 0.0)
            return

        if not self.startup_space_available(predecessor):
            self.publish_cmd(0.0, 0.0)
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

    # ------------------------------------------------------------------
    # Chain order
    # ------------------------------------------------------------------

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

        # Sort robots along the current leader heading.
        # Robots in front of the leader heading axis come first.
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

        if self.robot_name not in order:
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

    # ------------------------------------------------------------------
    # Startup release
    # ------------------------------------------------------------------

    def startup_space_available(self, predecessor: RobotState) -> bool:
        """
        Followers start once the predecessor opened enough space.

        This is intentionally less strict than the old 0.95 ratio.
        A lower ratio makes line formation much faster.
        """
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

    # ------------------------------------------------------------------
    # Target generation
    # ------------------------------------------------------------------

    def should_hold_position(self, predecessor: RobotState) -> bool:
        """
        If spacing is already good and the predecessor is slow/stopped,
        do not keep rotating to perfectly aim at the target point.

        This prevents follower wiggle during launch, waiting, and obstacle slowdown.
        """
        dx = predecessor.x - self.world_x
        dy = predecessor.y - self.world_y
        distance = math.hypot(dx, dy)

        gap_error = distance - self.desired_follow_distance_m

        if abs(gap_error) > self.hold_gap_deadband_m:
            return False

        predecessor_speed = abs(predecessor.linear_speed)

        if predecessor_speed > 0.03:
            return False

        angle_to_predecessor = math.atan2(dy, dx)
        heading_error = normalize_angle(angle_to_predecessor - self.yaw)

        if abs(heading_error) > self.hold_heading_deadband_rad:
            return False

        return True

    def get_predecessor_target(
        self,
        predecessor: RobotState,
    ) -> Optional[Tuple[float, float, float]]:
        dx = predecessor.x - self.world_x
        dy = predecessor.y - self.world_y
        distance = math.hypot(dx, dy)

        min_distance = self.desired_follow_distance_m - self.follow_deadband_m

        if distance < min_distance:
            return None

        # Do not use a hard upper deadband here.
        # A hard "return None" near the target creates start/stop motion when
        # the predecessor is moving slowly. Smooth hold/speed matching is handled
        # in compute_spacing_hold_command() and apply_gap_speed_control().
        # Target a slot behind the predecessor along the predecessor heading.
        # This is much more stable for the 2-row spawn layout than targeting
        # the radial line between follower and predecessor. The old radial target
        # could pull one robot sideways out of the chain during startup.
        tx = predecessor.x - self.desired_follow_distance_m * math.cos(predecessor.yaw)
        ty = predecessor.y - self.desired_follow_distance_m * math.sin(predecessor.yaw)

        return tx, ty, distance

    # ------------------------------------------------------------------
    # Control
    # ------------------------------------------------------------------

    def compute_control(self, tx: float, ty: float) -> Tuple[float, float]:
        dx = tx - self.world_x
        dy = ty - self.world_y
        distance_to_target = math.hypot(dx, dy)

        desired_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(desired_heading - self.yaw)

        linear = min(self.linear_gain * distance_to_target, self.max_linear_speed)

        # Less conservative than the old version, so robots keep moving while
        # turning into formation.
        abs_err = abs(heading_error)

        if abs_err > 1.30:
            linear *= 0.35
        elif abs_err > 0.85:
            linear *= 0.60
        elif abs_err > 0.50:
            linear *= 0.85

        # If almost at target, stop instead of jittering.
        if distance_to_target < 0.04:
            linear = 0.0

        angular = self.angular_gain * heading_error

        return linear, angular

    def compute_spacing_hold_command(
        self,
        predecessor: RobotState,
    ) -> Tuple[float, float]:
        """
        Smooth command for the spacing band.

        Old behavior published exactly zero when the follower was close to the
        desired distance. That caused stop/go motion when the leader crawled
        around obstacles. This method lets the follower creep at approximately
        the predecessor speed instead of repeatedly stopping and restarting.
        """
        dx = predecessor.x - self.world_x
        dy = predecessor.y - self.world_y
        distance = math.hypot(dx, dy)
        gap_error = distance - self.desired_follow_distance_m

        predecessor_speed = max(0.0, float(predecessor.linear_speed))

        # If predecessor is basically stopped and spacing is good, hold still.
        if abs(gap_error) <= self.hold_gap_deadband_m and predecessor_speed < 0.025:
            return 0.0, 0.0

        # If predecessor is moving slowly, match it instead of pulsing.
        if self.speed_match_enabled and gap_error >= -self.follow_deadband_m:
            matched = predecessor_speed * self.speed_match_gain

            if predecessor_speed > 0.025:
                matched = max(self.min_creep_speed, matched)

            linear = min(self.max_linear_speed, matched)
        else:
            linear = 0.0

        # Only rotate if we are badly misaligned. Small angular corrections while
        # waiting are the source of visible wiggle.
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
        gap_error = distance_to_predecessor - self.desired_follow_distance_m
        predecessor_speed = max(0.0, float(predecessor.linear_speed))

        # Too close: stop moving forward.
        if gap_error < -self.follow_deadband_m:
            return 0.0

        # Slightly close: move no faster than predecessor, so we do not close the
        # gap and then stop again.
        if gap_error < 0.0:
            if self.speed_match_enabled and predecessor_speed > 0.025:
                return min(linear, max(self.min_creep_speed, predecessor_speed * 0.85))
            return min(linear, self.max_linear_speed * 0.25)

        # Inside the good spacing band: speed match instead of hard stop/start.
        if gap_error < self.follow_deadband_m:
            if self.speed_match_enabled and predecessor_speed > 0.025:
                return min(
                    self.max_linear_speed * 0.70,
                    max(self.min_creep_speed, predecessor_speed * self.speed_match_gain),
                )

            return min(linear, self.max_linear_speed * 0.35)

        # Far away: maintain at least a useful catch-up speed.
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

    # ------------------------------------------------------------------
    # Safety around other robots
    # ------------------------------------------------------------------

    def predecessor_guard(
        self,
        predecessor: Optional[RobotState],
    ) -> Optional[Tuple[float, float]]:
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

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------

    def smooth_value(self, old: float, new: float, alpha: float) -> float:
        alpha = max(0.0, min(1.0, alpha))
        return (alpha * new) + ((1.0 - alpha) * old)

    def publish_cmd(self, linear: float, angular: float):
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

        # Avoid tiny residual commands after smoothing.
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
