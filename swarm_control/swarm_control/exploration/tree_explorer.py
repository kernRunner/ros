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

        self.escape_mode = False
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
        heading_error = normalize_angle(self.preferred_heading - self.yaw)

        if self.front < self.front_blocked_distance:
            if self.obstacle_escape_enabled:
                self.escape_mode = True
                self.escape_start_ns = self.get_clock().now().nanoseconds

            return self._turn_toward_open_space()

        if self.obstacle_escape_enabled and self.escape_mode:
            elapsed = (
                self.get_clock().now().nanoseconds - self.escape_start_ns
            ) / 1e9

            heading_error_ok = abs(heading_error) < math.radians(
                self.escape_rejoin_heading_error_deg
            )
            front_clear = self.front > self.escape_front_clear_distance

            if elapsed >= self.escape_min_time_sec and front_clear and heading_error_ok:
                self.escape_mode = False
            else:
                linear = min(self.forward_speed, 0.045)

                if self.left < self.side_clearance_distance:
                    return linear, -self.side_avoid_turn_gain

                if self.right < self.side_clearance_distance:
                    return linear, self.side_avoid_turn_gain

                angular = self.heading_gain * heading_error
                angular = max(-self.max_heading_turn, min(self.max_heading_turn, angular))
                return linear, angular

        angular = self.heading_gain * heading_error
        angular = max(-self.max_heading_turn, min(self.max_heading_turn, angular))

        linear = self.forward_speed

        if self.left < self.side_clearance_distance:
            linear = min(linear, 0.045)
            angular -= self.side_avoid_turn_gain

        if self.right < self.side_clearance_distance:
            linear = min(linear, 0.045)
            angular += self.side_avoid_turn_gain

        return linear, angular

    def _turn_toward_open_space(self):
        if self.front_left >= self.front_right:
            return 0.02, self.turn_speed

        return 0.02, -self.turn_speed

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

        if len(order) >= 4:
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