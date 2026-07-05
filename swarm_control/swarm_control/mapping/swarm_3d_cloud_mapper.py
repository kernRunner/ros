import math
from typing import Dict, List, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

import tf2_ros
from tf2_ros import TransformException


class Swarm3DCloudMapper(Node):
    """
    Accumulates /robotX/points into one global 3D PointCloud2 map.

    Output:
      /swarm/map_3d

    RViz:
      Fixed Frame: world
      Add -> PointCloud2 -> /swarm/map_3d
    """

    def __init__(self):
        super().__init__('swarm_3d_cloud_mapper')

        self.declare_parameter(
            'robot_names',
            [
                'robot1', 'robot2', 'robot3',
                'robot4', 'robot5', 'robot6',
                'robot7', 'robot8', 'robot9',
            ],
        )
        self.declare_parameter('cloud_topic_suffix', 'points')
        self.declare_parameter('fixed_frame', 'world')
        self.declare_parameter('map_topic', '/swarm/map_3d')

        # Voxel size controls map density.
        # Smaller = prettier but slower/heavier.
        self.declare_parameter('voxel_size', 0.15)

        # Keep map bounded so RViz and ROS do not get overloaded.
        self.declare_parameter('max_voxels', 250000)

        # Publish accumulated map at this rate.
        self.declare_parameter('publish_rate_hz', 1.0)

        # Point filtering in sensor frame before transform.
        self.declare_parameter('range_min', 0.35)
        self.declare_parameter('range_max', 8.0)

        # Point filtering after transform into world.
        # Useful to remove crazy points under/above the terrain.
        self.declare_parameter('world_z_min', -3.0)
        self.declare_parameter('world_z_max', 8.0)

        # Process at most this many points from each incoming cloud.
        # 0 means process all.
        self.declare_parameter('max_points_per_cloud', 2500)

        self.robot_names: List[str] = list(self.get_parameter('robot_names').value)
        self.cloud_topic_suffix = str(self.get_parameter('cloud_topic_suffix').value)
        self.fixed_frame = str(self.get_parameter('fixed_frame').value)
        self.map_topic = str(self.get_parameter('map_topic').value)

        self.voxel_size = float(self.get_parameter('voxel_size').value)
        self.max_voxels = int(self.get_parameter('max_voxels').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)
        self.world_z_min = float(self.get_parameter('world_z_min').value)
        self.world_z_max = float(self.get_parameter('world_z_max').value)
        self.max_points_per_cloud = int(self.get_parameter('max_points_per_cloud').value)

        if self.voxel_size <= 0.01:
            self.get_logger().warn('voxel_size too small; forcing to 0.05')
            self.voxel_size = 0.05

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # voxel key -> world point at voxel center / first observed point
        self.voxels: Dict[Tuple[int, int, int], Tuple[float, float, float]] = {}

        self.map_pub = self.create_publisher(
            PointCloud2,
            self.map_topic,
            qos_profile_sensor_data,
        )

        for robot_name in self.robot_names:
            topic = f'/{robot_name}/{self.cloud_topic_suffix}'
            self.create_subscription(
                PointCloud2,
                topic,
                lambda msg, rn=robot_name: self.cloud_callback(msg, rn),
                qos_profile_sensor_data,
            )

        timer_period = 1.0 / max(self.publish_rate_hz, 0.1)
        self.create_timer(timer_period, self.publish_map)

        self.get_logger().info(
            f'[swarm_3d_cloud_mapper] started: robots={self.robot_names}, '
            f'output={self.map_topic}, fixed_frame={self.fixed_frame}, '
            f'voxel_size={self.voxel_size}'
        )

    def cloud_callback(self, msg: PointCloud2, robot_name: str):
        source_frame = msg.header.frame_id

        if not source_frame:
            source_frame = f'{robot_name}/chassis/lidar_3d'

        try:
            tf = self.tf_buffer.lookup_transform(
                self.fixed_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as ex:
            self.get_logger().debug(
                f'No TF {self.fixed_frame} <- {source_frame}: {ex}'
            )
            return

        t = tf.transform.translation
        q = tf.transform.rotation

        rot = self.quaternion_to_matrix(q.x, q.y, q.z, q.w)

        added = 0
        seen = 0
        step = 1

        # If cloud is huge, stride through it instead of processing all points.
        if self.max_points_per_cloud > 0 and msg.width * msg.height > self.max_points_per_cloud:
            total = max(1, msg.width * msg.height)
            step = max(1, total // self.max_points_per_cloud)

        for point in point_cloud2.read_points(
            msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True,
        ):
            seen += 1

            if step > 1 and (seen % step) != 0:
                continue

            x = float(point[0])
            y = float(point[1])
            z = float(point[2])

            if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(z):
                continue

            r_xy = math.hypot(x, y)

            if r_xy < self.range_min or r_xy > self.range_max:
                continue

            wx = rot[0][0] * x + rot[0][1] * y + rot[0][2] * z + t.x
            wy = rot[1][0] * x + rot[1][1] * y + rot[1][2] * z + t.y
            wz = rot[2][0] * x + rot[2][1] * y + rot[2][2] * z + t.z

            if wz < self.world_z_min or wz > self.world_z_max:
                continue

            key = self.voxel_key(wx, wy, wz)

            if key not in self.voxels:
                self.voxels[key] = (wx, wy, wz)
                added += 1

        if added > 0 and len(self.voxels) > self.max_voxels:
            self.trim_voxels()

    def voxel_key(self, x: float, y: float, z: float) -> Tuple[int, int, int]:
        return (
            int(math.floor(x / self.voxel_size)),
            int(math.floor(y / self.voxel_size)),
            int(math.floor(z / self.voxel_size)),
        )

    def trim_voxels(self):
        # Simple trim: keep the newest-ish insertion order tail.
        # Python dict preserves insertion order.
        overflow = len(self.voxels) - self.max_voxels

        if overflow <= 0:
            return

        for key in list(self.voxels.keys())[:overflow]:
            self.voxels.pop(key, None)

        self.get_logger().warn(
            f'[swarm_3d_cloud_mapper] trimmed map to {len(self.voxels)} voxels'
        )

    def publish_map(self):
        if not self.voxels:
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.fixed_frame

        points = list(self.voxels.values())

        msg = point_cloud2.create_cloud(
            header,
            [
                PointField(
                    name='x',
                    offset=0,
                    datatype=PointField.FLOAT32,
                    count=1,
                ),
                PointField(
                    name='y',
                    offset=4,
                    datatype=PointField.FLOAT32,
                    count=1,
                ),
                PointField(
                    name='z',
                    offset=8,
                    datatype=PointField.FLOAT32,
                    count=1,
                ),
            ],
            points,
        )

        self.map_pub.publish(msg)

    def quaternion_to_matrix(self, x: float, y: float, z: float, w: float):
        xx = x * x
        yy = y * y
        zz = z * z
        xy = x * y
        xz = x * z
        yz = y * z
        wx = w * x
        wy = w * y
        wz = w * z

        return [
            [
                1.0 - 2.0 * (yy + zz),
                2.0 * (xy - wz),
                2.0 * (xz + wy),
            ],
            [
                2.0 * (xy + wz),
                1.0 - 2.0 * (xx + zz),
                2.0 * (yz - wx),
            ],
            [
                2.0 * (xz - wy),
                2.0 * (yz + wx),
                1.0 - 2.0 * (xx + yy),
            ],
        ]


def main(args=None):
    rclpy.init(args=args)
    node = Swarm3DCloudMapper()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()