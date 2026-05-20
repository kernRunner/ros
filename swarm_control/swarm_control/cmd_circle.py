import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdCircle(Node):
    def __init__(self):
        super().__init__('cmd_circle')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_cmd)

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = 0.3
        msg.angular.z = 0.4
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdCircle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()