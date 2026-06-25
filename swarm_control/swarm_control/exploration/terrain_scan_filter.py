import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from swarm_interfaces.msg import RobotState


class TerrainScanFilter(Node):
    """
    Converts 3D lidar PointCloud2 into a filtered 2D LaserScan.

    Important:
      - Every robot may have lidar.
      - But this node only processes point clouds when this robot is currently
        leader / group_leader.
      - Followers and relays do not spend CPU filtering point clouds.
    """

    def __init__(self):
        super().__init__('terrain_scan_filter')

        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('active_only_when_leader', True)

        self.declare_parameter('pointcloud_topic', 'points')
        self.declare_parameter('scan_topic', 'scan')

        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('num_ranges', 120)

        self.declare_parameter('range_min', 0.25)
        self.declare_parameter('range_max', 7.0)

        self.declare_parameter('min_obstacle_height', 0.05)
        self.declare_parameter('max_obstacle_height', 0.70)

        self.robot_name = self.get_parameter('robot_name').value
        self.active_only_when_leader = bool(
            self.get_parameter('active_only_when_leader').value
        )

        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value

        self.angle_min = float(self.get_parameter('angle_min').value)
        self.angle_max = float(self.get_parameter('angle_max').value)
        self.num_ranges = int(self.get_parameter('num_ranges').value)

        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)

        self.min_obstacle_height = float(
            self.get_parameter('min_obstacle_height').value
        )
        self.max_obstacle_height = float(
            self.get_parameter('max_obstacle_height').value
        )

        self.angle_increment = (
            self.angle_max - self.angle_min
        ) / float(self.num_ranges)

        self.active = not self.active_only_when_leader
        self.last_role_log_ns = 0

        self.scan_pub = self.create_publisher(
            LaserScan,
            self.scan_topic,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self.pointcloud_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            RobotState,
            '/swarm/robot_states',
            self.robot_state_callback,
            10,
        )

        self.get_logger().info(
            f'[{self.robot_name}] terrain_scan_filter started: '
            f'{self.pointcloud_topic} -> {self.scan_topic}'
        )

    def robot_state_callback(self, msg: RobotState):
        if msg.robot_name != self.robot_name:
            return

        was_active = self.active

        self.active = (
            msg.role in ('leader', 'group_leader')
            and msg.leader_id == self.robot_name
            and not msg.is_relay
        )

        if self.active != was_active:
            state = 'ACTIVE' if self.active else 'inactive'
            self.get_logger().info(
                f'[{self.robot_name}] terrain scan filter is now {state} '
                f'for role={msg.role}, leader_id={msg.leader_id}, relay={msg.is_relay}'
            )

    def pointcloud_callback(self, msg: PointCloud2):
        if not self.active:
            return

        ranges = [float('inf')] * self.num_ranges

        for point in point_cloud2.read_points(
            msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True,
        ):
            x = float(point[0])
            y = float(point[1])
            z = float(point[2])

            if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(z):
                continue

            # Ignore ground / slope points.
            if z < self.min_obstacle_height:
                continue

            # Ignore very high points.
            if z > self.max_obstacle_height:
                continue

            distance = math.hypot(x, y)

            if distance < self.range_min or distance > self.range_max:
                continue

            angle = math.atan2(y, x)

            if angle < self.angle_min or angle >= self.angle_max:
                continue

            index = int((angle - self.angle_min) / self.angle_increment)

            if index < 0 or index >= self.num_ranges:
                continue

            if distance < ranges[index]:
                ranges[index] = distance

        scan = LaserScan()
        scan.header = msg.header
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.0
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges
        scan.intensities = []

        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = TerrainScanFilter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()