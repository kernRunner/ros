import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from swarm_interfaces.msg import RobotState

from swarm_control.core.math_utils import quaternion_to_yaw


class BreadcrumbManager(Node):
    def __init__(self):
        super().__init__('breadcrumb_manager')

        self._declare_parameters()
        self._read_parameters()
        self._init_state()
        self._init_ros_interfaces()

        self.get_logger().info(f'[{self.robot_name}] breadcrumb_manager started')

    def _declare_parameters(self):
        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)
        self.declare_parameter('breadcrumb_distance', 1.0)
        self.declare_parameter('marker_topic', '/swarm/breadcrumb_markers')

    def _read_parameters(self):
        self.robot_name = self.get_parameter('robot_name').value
        self.frame_id = self.get_parameter('frame_id').value
        self.spawn_x = float(self.get_parameter('spawn_x').value)
        self.spawn_y = float(self.get_parameter('spawn_y').value)
        self.breadcrumb_distance = float(
            self.get_parameter('breadcrumb_distance').value
        )
        self.marker_topic = self.get_parameter('marker_topic').value

    def _init_state(self):
        self.is_leader = False
        self.current_role = 'follower'
        self.current_leader_id = ''

        self.world_x = 0.0
        self.world_y = 0.0
        self.yaw = 0.0

        self.last_breadcrumb_x = None
        self.last_breadcrumb_y = None

        self.breadcrumbs = []

    def _init_ros_interfaces(self):
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.create_subscription(
            RobotState,
            '/swarm/robot_states',
            self.state_callback,
            10,
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.marker_topic,
            10,
        )

        self.create_timer(0.5, self.publish_markers)

    def odom_callback(self, msg: Odometry):
        self.world_x = self.spawn_x + msg.pose.pose.position.x
        self.world_y = self.spawn_y + msg.pose.pose.position.y
        self.yaw = quaternion_to_yaw(msg.pose.pose.orientation)

        if self.is_leader:
            self._maybe_add_breadcrumb()

    def state_callback(self, msg: RobotState):
        if msg.robot_name != self.robot_name:
            return

        self.current_role = msg.role
        self.current_leader_id = msg.leader_id
        self.is_leader = msg.role == 'leader' and msg.leader_id == self.robot_name

    def _maybe_add_breadcrumb(self):
        if self.last_breadcrumb_x is None:
            self._add_breadcrumb()
            return

        distance = math.hypot(
            self.world_x - self.last_breadcrumb_x,
            self.world_y - self.last_breadcrumb_y,
        )

        if distance >= self.breadcrumb_distance:
            self._add_breadcrumb()

    def _add_breadcrumb(self):
        breadcrumb_id = len(self.breadcrumbs)

        self.breadcrumbs.append(
            {
                'id': breadcrumb_id,
                'robot_name': self.robot_name,
                'x': self.world_x,
                'y': self.world_y,
                'yaw': self.yaw,
            }
        )

        self.last_breadcrumb_x = self.world_x
        self.last_breadcrumb_y = self.world_y

        # self.get_logger().info(
        #     f'[{self.robot_name}] breadcrumb {breadcrumb_id}: '
        #     f'x={self.world_x:.2f}, y={self.world_y:.2f}'
        # )

    def publish_markers(self):
        markers = MarkerArray()

        for breadcrumb in self.breadcrumbs:
            markers.markers.append(self._make_breadcrumb_marker(breadcrumb))
            markers.markers.append(self._make_text_marker(breadcrumb))

        self.marker_pub.publish(markers)

    def _make_breadcrumb_marker(self, breadcrumb):
        marker = Marker()

        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = f'{self.robot_name}_breadcrumbs'
        marker.id = breadcrumb['id']
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = breadcrumb['x']
        marker.pose.position.y = breadcrumb['y']
        marker.pose.position.z = 0.08

        marker.scale.x = 0.18
        marker.scale.y = 0.18
        marker.scale.z = 0.18

        marker.color.r = 1.0
        marker.color.g = 0.7
        marker.color.b = 0.1
        marker.color.a = 1.0

        return marker

    def _make_text_marker(self, breadcrumb):
        marker = Marker()

        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = f'{self.robot_name}_breadcrumb_labels'
        marker.id = 10000 + breadcrumb['id']
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = breadcrumb['x']
        marker.pose.position.y = breadcrumb['y']
        marker.pose.position.z = 0.35

        marker.scale.z = 0.18

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.text = f"B{breadcrumb['id']}"

        return marker


def main(args=None):
    rclpy.init(args=args)
    node = BreadcrumbManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()