import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


class GroundTruthToTF(Node):
    def __init__(self):
        super().__init__('ground_truth_to_tf')

        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('pose_topic', 'ground_truth_pose')
        self.declare_parameter('parent_frame', 'world')
        self.declare_parameter('child_frame', 'chassis')

        self.robot_name = self.get_parameter('robot_name').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value

        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            10,
        )

        self.get_logger().info(
            f'[{self.robot_name}] ground_truth_to_tf started: '
            f'{self.parent_frame} -> {self.child_frame}'
        )

    def pose_callback(self, msg: PoseStamped):
        tf = TransformStamped()

        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.parent_frame
        tf.child_frame_id = self.child_frame

        tf.transform.translation.x = msg.pose.position.x
        tf.transform.translation.y = msg.pose.position.y
        tf.transform.translation.z = msg.pose.position.z

        tf.transform.rotation = msg.pose.orientation

        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthToTF()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()