import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from swarm_interfaces.msg import RobotState

from swarm_control.core.math_utils import normalize_angle, quaternion_to_yaw


class LeaderPathPublisher(Node):
    def __init__(self):
        super().__init__('leader_path_publisher')

        self._declare_parameters()
        self._read_parameters()
        self._init_state()
        self._init_ros_interfaces()

        self.get_logger().info(f'[{self.robot_name}] leader_path_publisher started')

    def _declare_parameters(self):
        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('path_topic', '/swarm/leader_path')
        self.declare_parameter('append_distance', 0.08)
        self.declare_parameter('append_yaw_delta', 0.20)
        self.declare_parameter('max_path_points', 2000)
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)

    def _read_parameters(self):
        self.robot_name = self.get_parameter('robot_name').value
        self.path_topic = self.get_parameter('path_topic').value
        self.append_distance = float(self.get_parameter('append_distance').value)
        self.append_yaw_delta = float(self.get_parameter('append_yaw_delta').value)
        self.max_path_points = int(self.get_parameter('max_path_points').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.spawn_x = float(self.get_parameter('spawn_x').value)
        self.spawn_y = float(self.get_parameter('spawn_y').value)

    def _init_state(self):
        self.is_leader = False
        self.current_leader_id = ''

        self.last_x = None
        self.last_y = None
        self.last_yaw = None

        self.have_self_state = False
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.path_msg = self._new_path_msg()

    def _init_ros_interfaces(self):
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.create_subscription(
            RobotState,
            '/swarm/robot_states',
            self.state_callback,
            10,
        )

        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.timer = self.create_timer(0.1, self.publish_path)

    def state_callback(self, msg: RobotState):
        if msg.robot_name != self.robot_name:
            return

        was_leader = self.is_leader

        self.current_leader_id = msg.leader_id
        self.is_leader = msg.role == 'leader' and msg.leader_id == self.robot_name

        self.current_x = msg.x
        self.current_y = msg.y
        self.current_yaw = msg.yaw
        self.have_self_state = True

        if self.is_leader and not was_leader:
            self._reset_path()
            self.get_logger().info(f'[{self.robot_name}] became leader, resetting path')

    def odom_callback(self, msg: Odometry):
        # Fallback only if RobotState has not arrived yet.
        if self.have_self_state:
            return

        if not self.is_leader:
            return

        x, y, yaw = self._world_pose_from_odom(msg)
        self._maybe_append_pose(msg.header.stamp, x, y, yaw)


    def _maybe_append_pose(self, stamp, x: float, y: float, yaw: float):
        if self._should_append_pose(x, y, yaw):
            self._append_interpolated_pose(stamp, x, y, yaw)
            self._remember_last_pose(x, y, yaw)


    def _append_interpolated_pose(self, stamp, x: float, y: float, yaw: float):
        # First point: append directly.
        if self.last_x is None:
            self._append_pose(stamp, x, y, yaw)
            return

        dx = x - self.last_x
        dy = y - self.last_y
        distance = math.hypot(dx, dy)

        yaw_delta = normalize_angle(yaw - self.last_yaw)

        # Keep path points dense enough for smooth corner following.
        max_step = max(0.02, self.append_distance)
        steps_by_distance = max(1, int(math.ceil(distance / max_step)))
        steps_by_yaw = max(1, int(math.ceil(abs(yaw_delta) / max(0.02, self.append_yaw_delta))))
        steps = max(steps_by_distance, steps_by_yaw)

        for i in range(1, steps + 1):
            ratio = i / steps

            ix = self.last_x + ratio * dx
            iy = self.last_y + ratio * dy
            iyaw = normalize_angle(self.last_yaw + ratio * yaw_delta)

            self._append_pose(stamp, ix, iy, iyaw)

    def publish_path(self):
        if not self.is_leader:
            return

        if self.have_self_state:
            self._maybe_append_pose(
                self.get_clock().now().to_msg(),
                self.current_x,
                self.current_y,
                self.current_yaw,
            )

        if not self.path_msg.poses:
            return

        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path_msg)

    def _world_pose_from_odom(self, msg: Odometry):
        x = self.spawn_x + msg.pose.pose.position.x
        y = self.spawn_y + msg.pose.pose.position.y
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        return x, y, yaw

    def _should_append_pose(self, x: float, y: float, yaw: float) -> bool:
        if self.last_x is None:
            return True

        distance = math.hypot(x - self.last_x, y - self.last_y)
        yaw_delta = abs(normalize_angle(yaw - self.last_yaw))

        return distance >= self.append_distance or yaw_delta >= self.append_yaw_delta

    def _append_pose(self, stamp, x: float, y: float, yaw: float):
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.frame_id

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.path_msg.poses.append(pose)
        self.path_msg.poses = self.path_msg.poses[-self.max_path_points:]

    def _remember_last_pose(self, x: float, y: float, yaw: float):
        self.last_x = x
        self.last_y = y
        self.last_yaw = yaw

    def _reset_path(self):
        self.path_msg = self._new_path_msg()
        self.last_x = None
        self.last_y = None
        self.last_yaw = None

    def _new_path_msg(self):
        path = Path()
        path.header.frame_id = self.frame_id
        return path


def main(args=None):
    rclpy.init(args=args)
    node = LeaderPathPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()