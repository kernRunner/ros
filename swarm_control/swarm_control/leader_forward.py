import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class LeaderForward(Node):
    def __init__(self):
        super().__init__('leader_forward')

        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('speed', 0.15)

        robot_name = self.get_parameter('robot_name').value
        speed = float(self.get_parameter('speed').value)

        self.speed = speed
        self.pub = self.create_publisher(
            Twist,
            f'/model/{robot_name}/cmd_vel',
            10
        )

        self.timer = self.create_timer(0.1, self.loop)

        self.get_logger().info(
            f'Publishing forward motion to /model/{robot_name}/cmd_vel at {self.speed:.2f} m/s'
        )

    def loop(self):
        msg = Twist()
        msg.linear.x = self.speed
        msg.angular.z = 0.0
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LeaderForward()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            stop = Twist()
            node.pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()