import math
import re

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from swarm_interfaces.msg import RobotState

from swarm_control.core.cmd_utils import make_twist
from swarm_control.core.math_utils import quaternion_to_yaw, normalize_angle
from swarm_control.core.scan_utils import sector_min, sector_avg


class TreeExplorer(Node):
    def __init__(self):
        super().__init__('tree_explorer')

        self._declare_parameters()
        self._read_parameters()
        self._init_state()
        self._init_ros_interfaces()

        self.get_logger().info(f'[{self.robot_name}] tree_explorer started')

    def _declare_parameters(self):
        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel_raw')

        self.declare_parameter('forward_speed', 0.060)
        self.declare_parameter('turn_speed', 0.55)
        self.declare_parameter('front_blocked_distance', 1.25)

        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)

        self.declare_parameter('chain_spacing_m', 0.9)
        self.declare_parameter('formation_tolerance_m', 0.35)

        self.declare_parameter('leader_start_delay_sec', 15.0)

        # Keep this false until basic movement is stable.
        self.declare_parameter('leader_wait_for_chain', False)
        self.declare_parameter('leader_slow_chain_gap_m', 2.00)
        self.declare_parameter('leader_max_chain_gap_m', 2.60)
        self.declare_parameter('leader_wait_turn_allowed', True)

        self.declare_parameter('side_clearance_distance', 0.75)
        self.declare_parameter('side_avoid_turn_gain', 0.30)

        self.declare_parameter('preferred_heading_deg', 90.0)
        self.declare_parameter('heading_gain', 0.65)
        self.declare_parameter('max_heading_turn', 0.35)

        self.declare_parameter('obstacle_escape_enabled', True)
        self.declare_parameter('escape_front_clear_distance', 1.60)
        self.declare_parameter('escape_rejoin_heading_error_deg', 25.0)
        self.declare_parameter('escape_min_time_sec', 2.0)

    def _read_parameters(self):
        self.robot_name = self.get_parameter('robot_name').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.front_blocked_distance = float(
            self.get_parameter('front_blocked_distance').value
        )

        self.spawn_x = float(self.get_parameter('spawn_x').value)
        self.spawn_y = float(self.get_parameter('spawn_y').value)

        self.chain_spacing_m = float(self.get_parameter('chain_spacing_m').value)
        self.formation_tolerance_m = float(
            self.get_parameter('formation_tolerance_m').value
        )

        self.leader_start_delay_sec = float(
            self.get_parameter('leader_start_delay_sec').value
        )

        self.leader_wait_for_chain = bool(
            self.get_parameter('leader_wait_for_chain').value
        )
        self.leader_slow_chain_gap_m = float(
            self.get_parameter('leader_slow_chain_gap_m').value
        )
        self.leader_max_chain_gap_m = float(
            self.get_parameter('leader_max_chain_gap_m').value
        )
        self.leader_wait_turn_allowed = bool(
            self.get_parameter('leader_wait_turn_allowed').value
        )

        self.side_clearance_distance = float(
            self.get_parameter('side_clearance_distance').value
        )
        self.side_avoid_turn_gain = float(
            self.get_parameter('side_avoid_turn_gain').value
        )

        self.preferred_heading_deg = float(
            self.get_parameter('preferred_heading_deg').value
        )
        self.preferred_heading = math.radians(self.preferred_heading_deg)

        self.heading_gain = float(self.get_parameter('heading_gain').value)
        self.max_heading_turn = float(self.get_parameter('max_heading_turn').value)

        self.obstacle_escape_enabled = bool(
            self.get_parameter('obstacle_escape_enabled').value
        )
        self.escape_front_clear_distance = float(
            self.get_parameter('escape_front_clear_distance').value
        )
        self.escape_rejoin_heading_error_deg = float(
            self.get_parameter('escape_rejoin_heading_error_deg').value
        )
        self.escape_min_time_sec = float(
            self.get_parameter('escape_min_time_sec').value
        )

    def _init_state(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.have_self_state = False

        self.is_leader = False
        self.current_role = 'follower'
        self.current_leader_id = ''

        self.other_robots = {}

        self.front = float('inf')
        self.front_left = float('inf')
        self.front_right = float('inf')
        self.left = float('inf')
        self.right = float('inf')

        self.became_leader_time_ns = None
        self.last_chain_status_log_ns = 0
        self.locked_chain_order = []

        # Leader exploration state machine.
        # CRUISE: follow preferred heading.
        # AVOID_OBSTACLE: keep one selected avoidance direction until clear.
        # REJOIN_HEADING: return to preferred heading only after obstacle is passed.
        self.mode = 'CRUISE'
        self.avoid_turn_sign = 0.0
        self.escape_start_ns = 0

    def _init_ros_interfaces(self):
        self.cmd_pub = self.create_publisher(
            make_twist().__class__,
            self.cmd_vel_topic,
            10,
        )

        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

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

        self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        # Fallback only, before /swarm/robot_states for this robot arrives.
        if self.have_self_state:
            return

        self.x = self.spawn_x + msg.pose.pose.position.x
        self.y = self.spawn_y + msg.pose.pose.position.y
        self.yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def state_callback(self, msg):
        self.other_robots[msg.robot_name] = msg

        if msg.robot_name != self.robot_name:
            return

        self.x = msg.x
        self.y = msg.y
        self.yaw = msg.yaw
        self.have_self_state = True

        was_leader = self.is_leader

        self.current_role = msg.role
        self.current_leader_id = msg.leader_id
        self.is_leader = msg.role == 'leader' and msg.leader_id == self.robot_name

        if self.is_leader and not was_leader:
            self.became_leader_time_ns = self.get_clock().now().nanoseconds
            self.get_logger().info(
                f'[{self.robot_name}] leader waiting '
                f'{self.leader_start_delay_sec:.1f}s before exploring'
            )

    def scan_callback(self, msg):
        if not msg.ranges:
            return

        self.front = sector_min(msg, 0.0, 0.35)
        self.front_left = sector_avg(msg, 0.65, 0.45)
        self.front_right = sector_avg(msg, -0.65, 0.45)

        self.left = sector_min(msg, math.pi / 2.0, 0.55)
        self.right = sector_min(msg, -math.pi / 2.0, 0.55)

    def control_loop(self):
        if not self.have_self_state:
            self._publish_stop()
            return

        if not self.is_leader:
            return

        if not self._leader_start_delay_done():
            self._publish_stop()
            return

        linear, angular = self._compute_exploration_command()

        if self.leader_wait_for_chain:
            scale = self._chain_speed_scale()
            linear *= scale

            if scale <= 0.01 and not self.leader_wait_turn_allowed:
                angular = 0.0

        self.cmd_pub.publish(make_twist(linear, angular))

    def _leader_start_delay_done(self):
        if self.became_leader_time_ns is None:
            return False

        elapsed = (
            self.get_clock().now().nanoseconds - self.became_leader_time_ns
        ) / 1e9

        return elapsed >= self.leader_start_delay_sec

    def _compute_exploration_command(self):
        """
        Main leader behavior.

        The important design rule is that tree_explorer owns normal obstacle
        avoidance. The safety filter should only be the emergency layer after
        this command is published.
        """
        if not self.obstacle_escape_enabled:
            return self._cruise_command()

        # Any new front blockage immediately enters/continues avoidance.
        if self.front < self.front_blocked_distance:
            if self.mode != 'AVOID_OBSTACLE':
                self._enter_avoid_mode()
            return self._avoid_obstacle_command()

        if self.mode == 'AVOID_OBSTACLE':
            return self._avoid_obstacle_command()

        if self.mode == 'REJOIN_HEADING':
            return self._rejoin_heading_command()

        return self._cruise_command()

    def _cruise_command(self):
        """Drive along the preferred exploration heading."""
        heading_error = normalize_angle(self.preferred_heading - self.yaw)

        angular = self.heading_gain * heading_error
        angular = max(-self.max_heading_turn, min(self.max_heading_turn, angular))

        linear = self.forward_speed

        # Side clearances are only gentle nudges in cruise mode.
        if self.left < self.side_clearance_distance:
            linear = min(linear, 0.07)
            angular -= self.side_avoid_turn_gain

        if self.right < self.side_clearance_distance:
            linear = min(linear, 0.07)
            angular += self.side_avoid_turn_gain

        return linear, angular

    def _enter_avoid_mode(self):
        """
        Lock one avoidance direction.

        This prevents left/right oscillation when front_left and front_right
        alternate by small amounts from scan noise or partial obstacle views.
        """
        self.mode = 'AVOID_OBSTACLE'
        self.escape_start_ns = self.get_clock().now().nanoseconds

        if self.front_left >= self.front_right:
            self.avoid_turn_sign = 1.0   # turn left
        else:
            self.avoid_turn_sign = -1.0  # turn right

        self.get_logger().info(
            f'[{self.robot_name}] obstacle avoidance started; '
            f'turn_sign={self.avoid_turn_sign:+.0f}'
        )

    def _avoid_obstacle_command(self):
        """
        Persistent obstacle escape behavior.

        While avoiding, do not try to rejoin the preferred heading. That was
        the source of the old fight: cruise wanted the preferred heading while
        safety/avoidance wanted to turn away.
        """
        elapsed = (
            self.get_clock().now().nanoseconds - self.escape_start_ns
        ) / 1e9

        # Still blocked in front: keep turning in the chosen direction.
        if self.front < self.front_blocked_distance:
            return 0.015, self.avoid_turn_sign * self.turn_speed

        # If we turn left, the obstacle is normally on the right.
        # If we turn right, the obstacle is normally on the left.
        obstacle_side_distance = (
            self.right if self.avoid_turn_sign > 0.0 else self.left
        )

        # Only leave avoidance after the robot has moved past the obstacle,
        # not merely after the front sector flickers clear for one cycle.
        obstacle_side_clear = (
            obstacle_side_distance > self.side_clearance_distance + 0.30
        )
        front_clear = self.front > self.escape_front_clear_distance

        if elapsed >= self.escape_min_time_sec and front_clear and obstacle_side_clear:
            self.mode = 'REJOIN_HEADING'
            return self._rejoin_heading_command()

        linear = min(self.forward_speed, 0.07)

        if obstacle_side_distance < self.side_clearance_distance:
            # Too close to the obstacle side: steer away.
            angular = self.avoid_turn_sign * self.side_avoid_turn_gain

        elif obstacle_side_distance > self.side_clearance_distance + 0.40:
            # Getting far enough around the obstacle: gently curve back around it.
            angular = -self.avoid_turn_sign * 0.18

        else:
            # Good clearance: continue forward around the obstacle.
            angular = 0.0

        return linear, angular

    def _rejoin_heading_command(self):
        """Return to preferred heading after obstacle escape is complete."""
        if self.front < self.front_blocked_distance:
            self._enter_avoid_mode()
            return self._avoid_obstacle_command()

        heading_error = normalize_angle(self.preferred_heading - self.yaw)

        if abs(heading_error) < math.radians(self.escape_rejoin_heading_error_deg):
            self.mode = 'CRUISE'

        angular = self.heading_gain * heading_error
        angular = max(-self.max_heading_turn, min(self.max_heading_turn, angular))

        linear = min(self.forward_speed, 0.075)
        return linear, angular

    def _chain_speed_scale(self):
        order = self._get_chain_order()

        if len(order) <= 1:
            return 1.0

        max_gap = self._max_chain_gap(order)
        self._log_chain_status(order, max_gap)

        if max_gap >= self.leader_max_chain_gap_m:
            return 0.0

        if max_gap >= self.leader_slow_chain_gap_m:
            span = self.leader_max_chain_gap_m - self.leader_slow_chain_gap_m

            if span <= 0.01:
                return 0.0

            scale = 1.0 - ((max_gap - self.leader_slow_chain_gap_m) / span)
            return max(0.25, min(1.0, scale))

        return 1.0

    def _get_chain_order(self):
        if self.locked_chain_order:
            return self.locked_chain_order

        robots = [
            r for r in self.other_robots.values()
            if r.active and r.leader_id == self.robot_name
        ]

        names = [r.robot_name for r in robots]

        if self.robot_name not in names:
            names.append(self.robot_name)

        ordered = sorted(names, key=self.robot_number)

        if self.robot_name in ordered:
            ordered.remove(self.robot_name)

        order = [self.robot_name] + ordered

        if len(order) >= len(self.other_robots):
            self.locked_chain_order = order
            self.get_logger().info(
                f'[{self.robot_name}] locked leader chain: {" -> ".join(order)}'
            )

        return order

    def _max_chain_gap(self, order):
        max_gap = 0.0

        for a, b in zip(order[:-1], order[1:]):
            pa = self._get_robot_position(a)
            pb = self._get_robot_position(b)

            if pa is None or pb is None:
                continue

            gap = math.hypot(pa[0] - pb[0], pa[1] - pb[1])
            max_gap = max(max_gap, gap)

        return max_gap

    def _get_robot_position(self, name):
        if name == self.robot_name:
            return self.x, self.y

        state = self.other_robots.get(name)

        if state is None or not state.active:
            return None

        return state.x, state.y

    def _log_chain_status(self, order, max_gap):
        now_ns = self.get_clock().now().nanoseconds

        if now_ns - self.last_chain_status_log_ns < 2_000_000_000:
            return

        self.last_chain_status_log_ns = now_ns

        # self.get_logger().info(
        #     f'[{self.robot_name}] chain={" -> ".join(order)} '
        #     f'max_gap={max_gap:.2f}m'
        # )

    def robot_number(self, name):
        match = re.search(r'\d+', name)
        return int(match.group()) if match else 999

    def _publish_stop(self):
        self.cmd_pub.publish(make_twist())


def main(args=None):
    rclpy.init(args=args)
    node = TreeExplorer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()