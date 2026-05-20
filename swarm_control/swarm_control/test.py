import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SwarmManager1(Node):
    def __init__(self):
        super().__init__('swarm_manager')

        self.robot_names = ['robot1', 'robot2', 'robot3']
        self.cmd_pubs = {}

        for name in self.robot_names:
            topic = f'/model/{name}/cmd_vel'
            self.cmd_pubs[name] = self.create_publisher(Twist, topic, 10)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Swarm manager started')

    def control_loop(self):
        commands = {
            'robot1': (0.20, 0.00),
            'robot2': (0.20, 0.15),
            'robot3': (0.20, -0.15),
        }

        for name, (vx, wz) in commands.items():
            msg = Twist()
            msg.linear.x = vx
            msg.angular.z = wz
            self.cmd_pubs[name].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SwarmManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()