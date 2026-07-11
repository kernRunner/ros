# Reads robot poses from Gazebo and republishes each one as a /robotX/ground_truth_pose topic for the swarm nodes.
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage


class GazeboPoseBridge(Node):
    def __init__(self):
        super().__init__('gazebo_pose_bridge')

        # Gazebo topic that contains all dynamic model poses.
        self.declare_parameter(
            'gazebo_pose_topic',
            '/world/tree_exploration_test/dynamic_pose/info',
        )

        # Robots that should receive ground-truth pose topics.
        self.declare_parameter(
            'robot_names',
            ['robot1', 'robot2', 'robot3', 'robot4'],
        )

        self.gazebo_pose_topic = self.get_parameter('gazebo_pose_topic').value
        self.robot_names = list(self.get_parameter('robot_names').value)

        # One publisher per robot.
        self.pose_pubs = {
            name: self.create_publisher(
                PoseStamped,
                f'/{name}/ground_truth_pose',
                10,
            )
            for name in self.robot_names
        }

        self.create_subscription(
            TFMessage,
            self.gazebo_pose_topic,
            self.pose_callback,
            10,
        )

        self.get_logger().info(
            f'gazebo_pose_bridge listening to {self.gazebo_pose_topic}'
        )

    def pose_callback(self, msg: TFMessage):
        now = self.get_clock().now().to_msg()

        for transform in msg.transforms:
            name = transform.child_frame_id

            if name not in self.pose_pubs:
                continue

            out = PoseStamped()
            out.header.stamp = now
            out.header.frame_id = 'world'

            out.pose.position.x = transform.transform.translation.x
            out.pose.position.y = transform.transform.translation.y
            out.pose.position.z = transform.transform.translation.z

            out.pose.orientation = transform.transform.rotation

            self.pose_pubs[name].publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboPoseBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()