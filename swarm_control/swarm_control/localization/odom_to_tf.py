import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')

        self._declare_parameters()
        self._read_parameters()
        self._init_ros_interfaces()

        self.get_logger().info(
            f'[{self.robot_name}] TF: {self.odom_frame} -> {self.base_frame}'
        )

    def _declare_parameters(self):
        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('base_frame', 'chassis')
        self.declare_parameter('odom_frame', 'odom')

    def _read_parameters(self):
        self.robot_name = self.get_parameter('robot_name').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value

    def _init_ros_interfaces(self):
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )

    def odom_callback(self, msg: Odometry):
        transform = self._odom_to_transform(msg)
        self.tf_broadcaster.sendTransform(transform)

    def _odom_to_transform(self, msg: Odometry) -> TransformStamped:
        transform = TransformStamped()

        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame

        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation

        return transform


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()