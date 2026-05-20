import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from swarm_interfaces.msg import RobotState

from swarm_control.core.cmd_utils import make_twist, smooth_value
from swarm_control.core.math_utils import normalize_angle, quaternion_to_yaw


class LeaderGoalController(Node):
    def __init__(self):
        super().__init__('leader_goal_controller')

        self._declare_parameters()
        self._read_parameters()
        self._init_state()
        self._init_ros_interfaces()

        self.get_logger().info(f'[{self.robot_name}] leader_goal_controller started')

    # -------------------------
    # Setup
    # -------------------------

    def _declare_parameters(self):
        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel_raw')  # IMPORTANT
        self.declare_parameter('goal_topic', '/goal_pose')

        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)

        self.declare_parameter('goal_tolerance_m', 0.20)
        self.declare_parameter('yaw_tolerance_rad', 0.15)

        self.declare_parameter('max_linear_speed', 0.12)
        self.declare_parameter('max_angular_speed', 0.90)

        self.declare_parameter('linear_gain', 0.45)
        self.declare_parameter('angular_gain', 1.60)

    def _read_parameters(self):
        self.robot_name = self.get_parameter('robot_name').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.goal_topic = self.get_parameter('goal_topic').value

        self.spawn_x = float(self.get_parameter('spawn_x').value)
        self.spawn_y = float(self.get_parameter('spawn_y').value)

        self.goal_tolerance_m = float(self.get_parameter('goal_tolerance_m').value)
        self.yaw_tolerance_rad = float(self.get_parameter('yaw_tolerance_rad').value)

        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)

        self.linear_gain = float(self.get_parameter('linear_gain').value)
        self.angular_gain = float(self.get_parameter('angular_gain').value)

    def _init_state(self):
        self.world_x = 0.0
        self.world_y = 0.0
        self.yaw = 0.0

        self.is_leader = False
        self.current_role = 'follower'
        self.current_leader_id = ''

        self.goal_x: Optional[float] = None
        self.goal_y: Optional[float] = None
        self.goal_yaw: Optional[float] = None

        self.last_linear = 0.0
        self.last_angular = 0.0

    def _init_ros_interfaces(self):
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.create_subscription(
            RobotState,
            '/swarm/robot_states',
            self.state_callback,
            10,
        )

        self.create_subscription(
            PoseStamped,
            self.goal_topic,
            self.goal_callback,
            10,
        )

        self.create_timer(0.1, self.control_loop)

    # -------------------------
    # Callbacks
    # -------------------------

    def odom_callback(self, msg: Odometry):
        self.world_x = self.spawn_x + msg.pose.pose.position.x
        self.world_y = self.spawn_y + msg.pose.pose.position.y
        self.yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def state_callback(self, msg: RobotState):
        if msg.robot_name != self.robot_name:
            return

        self.current_role = msg.role
        self.current_leader_id = msg.leader_id
        self.is_leader = msg.role == 'leader' and msg.leader_id == self.robot_name

    def goal_callback(self, msg: PoseStamped):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_yaw = quaternion_to_yaw(msg.pose.orientation)

        self.get_logger().info(
            f'[{self.robot_name}] received goal: '
            f'x={self.goal_x:.2f}, y={self.goal_y:.2f}'
        )

    # -------------------------
    # Control
    # -------------------------

    def control_loop(self):
        if not self._should_run():
            return

        control = self._compute_control_to_goal()

        if control is None:
            self.publish_stop()
            return

        linear, angular = control
        self.publish_cmd(linear, angular)

    def _should_run(self) -> bool:
        if not self.is_leader:
            return False

        if self.goal_x is None or self.goal_y is None:
            return False

        return True

    def _compute_control_to_goal(self) -> Optional[Tuple[float, float]]:
        dx = self.goal_x - self.world_x
        dy = self.goal_y - self.world_y

        distance = math.hypot(dx, dy)

        if distance < self.goal_tolerance_m:
            return None

        target_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(target_heading - self.yaw)

        linear = min(self.linear_gain * distance, self.max_linear_speed)

        if abs(heading_error) > 0.9:
            linear *= 0.15
        elif abs(heading_error) > 0.5:
            linear *= 0.45

        angular = self.angular_gain * heading_error
        angular = max(-self.max_angular_speed, min(self.max_angular_speed, angular))

        return linear, angular

    def publish_cmd(self, linear: float, angular: float):
        self.last_linear = smooth_value(self.last_linear, linear, alpha=0.55)
        self.last_angular = smooth_value(self.last_angular, angular, alpha=0.45)

        self.cmd_pub.publish(make_twist(self.last_linear, self.last_angular))

    def publish_stop(self):
        self.last_linear = 0.0
        self.last_angular = 0.0
        self.cmd_pub.publish(make_twist())


def main(args=None):
    rclpy.init(args=args)
    node = LeaderGoalController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()