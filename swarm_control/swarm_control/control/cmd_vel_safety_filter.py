import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from swarm_interfaces.msg import RobotState

from swarm_control.core.cmd_utils import make_twist, smooth_value
from swarm_control.core.scan_utils import sector_min, sector_avg


class CmdVelSafetyFilter(Node):
    """
    Safety filter that sits between cmd_vel_raw and cmd_vel.

    Fix changelog:
      - Subscribes to /swarm/chain_order (published by path_follower) so
        predecessor identification is always consistent with the follower
        logic. The old grid-based _get_robot_ahead_name() sorted by world-X,
        which broke after the first turn and caused the safety filter to
        treat the predecessor as an obstacle and hard-stop the robot.
      - Side-wall angular corrections now CLAMP instead of ADD for followers.
        The old code added ±0.35 rad on top of path_follower's already-
        corrected heading, stacking corrections and causing lateral spread.
        The leader keeps additive wall avoidance because it has no path follower.
      - Smoothing alphas reduced for followers (0.30/0.25 vs 0.65/0.55) to
        cut ~200-300 ms of lag that was stacking down the chain.
    """

    def __init__(self):
        super().__init__('cmd_vel_safety_filter')

        self._declare_parameters()
        self._read_parameters()
        self._init_state()
        self._init_ros_interfaces()

        self.get_logger().info(f'[{self.robot_name}] cmd_vel_safety_filter started')

    def _declare_parameters(self):
        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('enabled', True)

        self.declare_parameter('raw_cmd_topic', 'cmd_vel_raw')
        self.declare_parameter('safe_cmd_topic', 'cmd_vel')

        self.declare_parameter('hard_stop_distance', 0.18)
        self.declare_parameter('slowdown_distance', 0.32)
        self.declare_parameter('side_stop_distance', 0.18)
        self.declare_parameter('side_slow_distance', 0.35)

        self.declare_parameter('max_safe_linear_speed', 0.12)
        self.declare_parameter('wall_avoid_gain', 0.05)

    def _read_parameters(self):
        self.robot_name = self.get_parameter('robot_name').value
        self.enabled = bool(self.get_parameter('enabled').value)

        self.raw_cmd_topic = self.get_parameter('raw_cmd_topic').value
        self.safe_cmd_topic = self.get_parameter('safe_cmd_topic').value

        self.hard_stop_distance = float(self.get_parameter('hard_stop_distance').value)
        self.slowdown_distance = float(self.get_parameter('slowdown_distance').value)
        self.side_stop_distance = float(self.get_parameter('side_stop_distance').value)
        self.side_slow_distance = float(self.get_parameter('side_slow_distance').value)

        self.max_safe_linear_speed = float(
            self.get_parameter('max_safe_linear_speed').value
        )
        self.wall_avoid_gain = float(self.get_parameter('wall_avoid_gain').value)

    def _init_state(self):
        self.raw_linear = 0.0
        self.raw_angular = 0.0

        self.front = float('inf')
        self.front_left = float('inf')
        self.front_right = float('inf')
        self.left = float('inf')
        self.right = float('inf')

        self.current_role = 'follower'
        self.current_leader_id = ''
        self.is_leader = False

        self.last_linear = 0.0
        self.last_angular = 0.0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.other_robots = {}

        # FIX: authoritative chain order from path_follower.
        # Replaces the old grid-based _get_robot_ahead_name() which broke
        # after the leader turned because it sorted by world-X axis.
        self.chain_order: list = []

    def _init_ros_interfaces(self):
        self.cmd_pub = self.create_publisher(Twist, self.safe_cmd_topic, 10)

        self.create_subscription(
            Twist,
            self.raw_cmd_topic,
            self.cmd_callback,
            10,
        )

        self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            RobotState,
            '/swarm/robot_states',
            self.state_callback,
            10,
        )

        # FIX: subscribe to the chain order published by path_follower.
        self.create_subscription(
            String,
            '/swarm/chain_order',
            self._chain_order_cb,
            10,
        )

        self.create_timer(0.1, self.control_loop)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def cmd_callback(self, msg: Twist):
        self.raw_linear = msg.linear.x
        self.raw_angular = msg.angular.z

    def state_callback(self, msg: RobotState):
        self.other_robots[msg.robot_name] = msg

        if msg.robot_name != self.robot_name:
            return

        self.current_role = msg.role
        self.current_leader_id = msg.leader_id
        self.is_leader = msg.role == 'leader' and msg.leader_id == self.robot_name

        self.x = msg.x
        self.y = msg.y
        self.yaw = msg.yaw

    def _chain_order_cb(self, msg: String):
        """
        Receive the locked chain order from path_follower.

        This is the single source of truth for who is the predecessor of
        each robot. Using this prevents the safety filter from ever
        disagreeing with path_follower about chain membership.
        """
        try:
            self.chain_order = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warn(
                f'[{self.robot_name}] failed to parse chain_order: {exc}'
            )

    def scan_callback(self, msg: LaserScan):
        if not msg.ranges:
            return

        self.front = self._smooth_scan_value(
            self.front,
            sector_min(msg, 0.0, 0.35),
            0.65,
        )

        self.front_left = self._smooth_scan_value(
            self.front_left,
            sector_avg(msg, 0.65, 0.45),
            0.65,
        )

        self.front_right = self._smooth_scan_value(
            self.front_right,
            sector_avg(msg, -0.65, 0.45),
            0.65,
        )

        self.left = self._smooth_scan_value(
            self.left,
            sector_min(msg, math.pi / 2.0, 0.55),
            0.65,
        )

        self.right = self._smooth_scan_value(
            self.right,
            sector_min(msg, -math.pi / 2.0, 0.55),
            0.65,
        )

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def control_loop(self):
        if not self.enabled:
            return

        linear, angular = self._safe_command(self.raw_linear, self.raw_angular)
        self._publish_cmd(linear, angular)

    def _safe_command(self, linear: float, angular: float):
        linear = min(linear, self.max_safe_linear_speed)

        linear, angular = self._apply_robot_collision_avoidance(linear, angular)

        side_bias = self.front_left - self.front_right

        if self.is_leader:
            # Emergency safety only.
            # tree_explorer owns normal leader obstacle avoidance and heading.
            # This layer may reduce speed or stop, but it should not fight the
            # planner's angular command except as a last resort.
            if self.front < self.hard_stop_distance:
                linear = 0.0

                # Only choose an emergency turn if tree_explorer did not already
                # command a meaningful turn direction.
                if abs(angular) < 0.05:
                    angular = 0.45 if side_bias >= 0.0 else -0.45

            elif self.front < self.slowdown_distance:
                linear = min(linear, 0.045)

            # Side obstacles reduce speed only. Do not overwrite angular here;
            # tree_explorer's state machine owns wall/obstacle steering.
            if self.left < self.side_stop_distance:
                linear = min(linear, 0.025)

            elif self.left < self.side_slow_distance:
                linear = min(linear, 0.040)

            if self.right < self.side_stop_distance:
                linear = min(linear, 0.025)

            elif self.right < self.side_slow_distance:
                linear = min(linear, 0.040)

        else:
            # Followers: path_follower owns the desired heading.
            # Safety should prevent crashes, not replace or flip the path command.
            predecessor_ahead = self._predecessor_is_in_front()

            if predecessor_ahead:
                # The object ahead is probably the robot we follow.
                if self.front < 0.18:
                    linear = 0.0
            else:
                # Front obstacle: strict protection.
                # This is the only place where follower safety may override angular.
                if self.front < self.hard_stop_distance:
                    linear = 0.0
                    angular = 0.55 if side_bias >= 0.0 else -0.55

                elif self.front < self.slowdown_distance:
                    linear = min(linear, 0.045)

                    # Small front-avoidance nudge only.
                    # This can still slightly modify path_follower, but it should not dominate.
                    angular += 0.10 if side_bias >= 0.0 else -0.10

            # Side obstacles for followers:
            # IMPORTANT:
            # Do NOT change angular here.
            # The old code flipped angular direction and caused drift.
            if self.left < self.side_stop_distance:
                linear = min(linear, 0.035)

            elif self.left < self.side_slow_distance:
                linear = min(linear, 0.060)

            if self.right < self.side_stop_distance:
                linear = min(linear, 0.035)

            elif self.right < self.side_slow_distance:
                linear = min(linear, 0.060)

            # No wall_error centering for followers.
            # The path follower's cross-track and line-hold correction owns alignment.

        angular = max(-1.00, min(1.00, angular))
        return linear, angular
    # ------------------------------------------------------------------
    # Predecessor identification  (FIX: use shared chain order)
    # ------------------------------------------------------------------

    def _get_robot_ahead_name(self) -> str | None:
        """
        Return the name of this robot's predecessor in the chain.

        FIX: uses the chain_order list broadcast by path_follower instead
        of recomputing from world-X position. The old approach sorted by
        world X-axis and broke after the first turn, causing the safety
        filter to misidentify the predecessor and hard-stop the robot.
        """
        order = self.chain_order
        if not order or self.robot_name not in order:
            return None
        idx = order.index(self.robot_name)
        if idx == 0:
            return None
        return order[idx - 1]

    def _smooth_scan_value(self, old: float, new: float, alpha: float) -> float:
        if not math.isfinite(old):
            return new
        if not math.isfinite(new):
            return old
        return smooth_value(old, new, alpha)

    def _predecessor_is_in_front(self) -> bool:
        predecessor_name = self._get_robot_ahead_name()

        if predecessor_name is None:
            return False

        predecessor = self.other_robots.get(predecessor_name)

        if predecessor is None:
            return False

        dx = predecessor.x - self.x
        dy = predecessor.y - self.y

        distance = math.hypot(dx, dy)

        if distance > 1.2:
            return False

        angle_to_predecessor = math.atan2(dy, dx)
        heading_error = abs(angle_to_predecessor - self.yaw)

        while heading_error > math.pi:
            heading_error -= 2.0 * math.pi
        while heading_error < -math.pi:
            heading_error += 2.0 * math.pi

        return abs(heading_error) < 0.45

    def _apply_robot_collision_avoidance(self, linear: float, angular: float):
        robot_ahead_name = self._get_robot_ahead_name()

        for name, robot in self.other_robots.items():
            if name == self.robot_name:
                continue

            dx = robot.x - self.x
            dy = robot.y - self.y
            dist = math.hypot(dx, dy)

            # Emergency stop only.
            if dist < 0.18:
                return 0.0, angular

            # The robot directly ahead is allowed to be close during chain following.
            if name == robot_ahead_name:
                if dist < 0.35:
                    linear = min(linear, 0.04)
                continue

            # Other robots: slow only if very close, do not block normal chain merge.
            if dist < 0.22:
                linear = min(linear, 0.035)

        return linear, angular

    def _publish_cmd(self, linear: float, angular: float):
        # FIX: reduced smoothing alpha for followers to cut lag that was
        # stacking down the chain (~200-300 ms per robot at the old values).
        # Leader keeps stronger smoothing for gentle wall-avoidance steering.
        if self.is_leader:
            alpha_lin = 0.65
            alpha_ang = 0.55
        else:
            alpha_lin = 0.25
            alpha_ang = 0.10

        self.last_linear = smooth_value(self.last_linear, linear, alpha=alpha_lin)
        self.last_angular = smooth_value(self.last_angular, angular, alpha=alpha_ang)

        self.cmd_pub.publish(make_twist(self.last_linear, self.last_angular))


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSafetyFilter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_cmd(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()