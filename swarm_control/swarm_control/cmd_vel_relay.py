import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')

        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/model/robot1/cmd_vel')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.pub = self.create_publisher(Twist, output_topic, 10)
        self.sub = self.create_subscription(Twist, input_topic, self.callback, 10)

        self.get_logger().info(f'Relaying {input_topic} -> {output_topic}')

    def callback(self, msg: Twist):
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()