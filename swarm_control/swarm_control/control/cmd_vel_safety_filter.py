import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from swarm_interfaces.msg import RobotState

from swarm_control.core.cmd_utils import make_twist, smooth_value
from swarm_control.core.scan_utils import sector_min, sector_avg


class CmdVelSafetyFilter(Node):
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

        self.create_timer(0.1, self.control_loop)

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

    def scan_callback(self, msg: LaserScan):
        if not msg.ranges:
            return

        self.front = smooth_value(
            self.front,
            sector_min(msg, 0.0, 0.25),
            0.65,
        )

        self.front_left = smooth_value(
            self.front_left,
            sector_avg(msg, 0.65, 0.35),
            0.65,
        )

        self.front_right = smooth_value(
            self.front_right,
            sector_avg(msg, -0.65, 0.35),
            0.65,
        )

        self.left = smooth_value(
            self.left,
            sector_min(msg, math.pi / 2.0, 0.45),
            0.65,
        )

        self.right = smooth_value(
            self.right,
            sector_min(msg, -math.pi / 2.0, 0.45),
            0.65,
        )

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
            if self.front < self.hard_stop_distance:
                linear = 0.0
                angular = 0.55 if side_bias >= 0.0 else -0.55

            elif self.front < self.slowdown_distance:
                linear = min(linear, 0.045)
                angular += 0.15 if side_bias >= 0.0 else -0.15

        else:
            # Followers may see the robot ahead. Only emergency-stop very close.
            if self.front < 0.12:
                linear = 0.0

        if self.left < self.side_stop_distance:
            linear = min(linear, 0.03)
            angular -= 0.25

        if self.right < self.side_stop_distance:
            linear = min(linear, 0.03)
            angular += 0.25

        if self.left < self.side_slow_distance or self.right < self.side_slow_distance:
            wall_error = self.left - self.right

            if abs(wall_error) < 10.0:
                angular -= self.wall_avoid_gain * wall_error
                linear = min(linear, 0.05)

        angular = max(-0.70, min(0.70, angular))
        return linear, angular

    def _get_robot_ahead_name(self):
        if not self.current_leader_id:
            return None

        robots = [
            r for r in self.other_robots.values()
            if r.active and r.leader_id == self.current_leader_id
        ]

        leader = None
        for r in robots:
            if r.robot_name == r.leader_id:
                leader = r
                break

        if leader is None:
            return None

        followers = [
            r for r in robots
            if r.robot_name != leader.robot_name
        ]

        same_row = []
        behind_rows = []

        for r in followers:
            if abs(r.x - leader.x) <= 0.35:
                same_row.append(r)
            else:
                behind_rows.append(r)

        behind_same_lane = [
            r for r in behind_rows
            if abs(r.y - leader.y) <= 0.7
        ]

        behind_other_lane = [
            r for r in behind_rows
            if abs(r.y - leader.y) > 0.7
        ]

        behind_same_lane.sort(
            key=lambda r: (-r.x, abs(r.y - leader.y), r.robot_name)
        )
        same_row.sort(
            key=lambda r: (abs(r.y - leader.y), r.robot_name)
        )
        behind_other_lane.sort(
            key=lambda r: (-r.x, abs(r.y - leader.y), r.robot_name)
        )

        order = [leader.robot_name] + [
            r.robot_name for r in behind_same_lane + same_row + behind_other_lane
        ]

        if self.robot_name not in order:
            return None

        index = order.index(self.robot_name)

        if index == 0:
            return None

        return order[index - 1]

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
        self.last_linear = smooth_value(self.last_linear, linear, alpha=0.65)
        self.last_angular = smooth_value(self.last_angular, angular, alpha=0.55)

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