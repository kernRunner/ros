import math
import re

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from swarm_control.core.math_utils import normalize_angle, quaternion_to_yaw


class SimpleChainFollower(Node):
    def __init__(self):
        super().__init__('simple_chain_follower')

        self.declare_parameter('robot_name', 'robot2')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel_raw')

        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)

        self.declare_parameter('line_start_x', 1.2)
        self.declare_parameter('line_start_y', 0.6)
        self.declare_parameter('spacing', 1.8)

        self.declare_parameter('max_linear', 0.08)
        self.declare_parameter('max_angular', 0.60)

        self.robot_name = self.get_parameter('robot_name').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.spawn_x = float(self.get_parameter('spawn_x').value)
        self.spawn_y = float(self.get_parameter('spawn_y').value)

        self.line_start_x = float(self.get_parameter('line_start_x').value)
        self.line_start_y = float(self.get_parameter('line_start_y').value)
        self.spacing = float(self.get_parameter('spacing').value)

        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)

        self.x = None
        self.y = None
        self.yaw = 0.0

        self.slot = self.robot_number(self.robot_name) - 1

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)

        self.create_timer(0.1, self.loop)

        self.get_logger().info(
            f'{self.robot_name} target slot={self.slot}'
        )

    def robot_number(self, name):
        match = re.search(r'\d+', name)
        return int(match.group()) if match else 1

    def odom_cb(self, msg):
        self.x = self.spawn_x + msg.pose.pose.position.x
        self.y = self.spawn_y + msg.pose.pose.position.y
        self.yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def loop(self):
        if self.x is None:
            self.stop()
            return

        # Fixed line:
        # robot1 stays at line_start.
        # robot2 goes behind it.
        # robot3 behind robot2.
        # robot4 behind robot3.
        target_x = self.line_start_x - self.slot * self.spacing
        target_y = self.line_start_y
        target_yaw = 0.0

        dx = target_x - self.x
        dy = target_y - self.y
        dist = math.hypot(dx, dy)

        if dist < 0.12:
            yaw_error = normalize_angle(target_yaw - self.yaw)
            if abs(yaw_error) < 0.25:
                self.stop()
                return

        heading = math.atan2(dy, dx)
        heading_error = normalize_angle(heading - self.yaw)

        linear = min(0.35 * dist, self.max_linear)
        angular = 1.2 * heading_error

        if abs(heading_error) > 1.20:
            linear = 0.0
        elif abs(heading_error) > 0.70:
            linear *= 0.25

        angular = max(-self.max_angular, min(self.max_angular, angular))

        self.cmd(linear, angular)

    def cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    def stop(self):
        self.cmd(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleChainFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()