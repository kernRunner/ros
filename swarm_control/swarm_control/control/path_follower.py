import math
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool
from swarm_interfaces.msg import RobotState

from swarm_control.core.math_utils import normalize_angle, quaternion_to_yaw


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        self.declare_parameter('robot_name', 'robot2')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel_raw')
        self.declare_parameter('path_topic', '/swarm/leader_path')

        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)

        self.declare_parameter('chain_spacing_m', 1.8)
        self.declare_parameter('lookahead_m', 0.15)
        self.declare_parameter('goal_tolerance_m', 0.10)

        self.declare_parameter('max_linear_speed', 0.10)
        self.declare_parameter('max_angular_speed', 0.80)
        self.declare_parameter('linear_gain', 0.45)
        self.declare_parameter('angular_gain', 1.50)

        self.declare_parameter('collision_stop_m', 0.70)
        self.declare_parameter('collision_slow_m', 1.15)

        self.robot_name = self.get_parameter('robot_name').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.path_topic = self.get_parameter('path_topic').value

        self.spawn_x = float(self.get_parameter('spawn_x').value)
        self.spawn_y = float(self.get_parameter('spawn_y').value)

        self.chain_spacing_m = float(self.get_parameter('chain_spacing_m').value)
        self.lookahead_m = float(self.get_parameter('lookahead_m').value)
        self.goal_tolerance_m = float(self.get_parameter('goal_tolerance_m').value)

        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.linear_gain = float(self.get_parameter('linear_gain').value)
        self.angular_gain = float(self.get_parameter('angular_gain').value)

        self.collision_stop_m = float(self.get_parameter('collision_stop_m').value)
        self.collision_slow_m = float(self.get_parameter('collision_slow_m').value)

        self.world_x = 0.0
        self.world_y = 0.0
        self.yaw = 0.0

        self.path: Optional[Path] = None
        self.formation_done = False

        self.current_role = 'follower'
        self.current_leader_id = ''
        self.is_leader = False

        self.other_robots: Dict[str, RobotState] = {}
        self.locked_order: List[str] = []

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(Path, self.path_topic, self.path_callback, 10)
        self.create_subscription(RobotState, '/swarm/robot_states', self.state_callback, 10)
        self.create_subscription(Bool, 'formation_ready', self.formation_ready_cb, 10)

        self.create_timer(0.1, self.control_loop)

        self.get_logger().info(f'[{self.robot_name}] path_follower waiting for formation_ready')

    def odom_callback(self, msg: Odometry):
        self.world_x = self.spawn_x + msg.pose.pose.position.x
        self.world_y = self.spawn_y + msg.pose.pose.position.y
        self.yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def path_callback(self, msg: Path):
        self.path = msg

    def state_callback(self, msg: RobotState):
        self.other_robots[msg.robot_name] = msg

        if msg.robot_name == self.robot_name:
            self.current_role = msg.role
            self.current_leader_id = msg.leader_id
            self.is_leader = msg.role == 'leader' and msg.leader_id == self.robot_name

    def formation_ready_cb(self, msg: Bool):
        if msg.data and not self.formation_done:
            self.formation_done = True
            self.get_logger().info(f'[{self.robot_name}] path following active')

    def control_loop(self):
        if not self.formation_done:
            self.publish_cmd(0.0, 0.0)
            return

        if self.is_leader or self.current_role != 'follower' or not self.current_leader_id:
            self.publish_cmd(0.0, 0.0)
            return

        target = self.get_path_target()

        if target is None:
            self.publish_cmd(0.0, 0.0)
            return

        linear, angular = self.compute_control(target)

        scale = self.collision_speed_scale()
        linear *= scale

        if scale <= 0.01:
            linear = 0.0

        self.publish_cmd(linear, angular)

    def get_chain_order(self):
        leader_name = self.current_leader_id

        if not leader_name:
            return []

        robots = [
            r for r in self.other_robots.values()
            if r.active and r.leader_id == leader_name
        ]

        def robot_number(name):
            digits = ''.join(c for c in name if c.isdigit())
            return int(digits) if digits else 999

        names = sorted([r.robot_name for r in robots], key=robot_number)

        if leader_name in names:
            names.remove(leader_name)

        return [leader_name] + names

    def get_my_index(self):
        order = self.get_chain_order()

        if self.robot_name not in order:
            return None

        return order.index(self.robot_name)

    def get_path_target(self):
        if self.path is None or len(self.path.poses) < 2:
            return None

        index = self.get_my_index()

        if index is None or index == 0:
            return None

        total, cumulative = self.compute_path_lengths(self.path)

        if total <= 0.01:
            return None

        s = total - index * self.chain_spacing_m + self.lookahead_m
        s = max(0.0, min(total, s))

        x, y = self.sample_path(self.path, cumulative, s)
        yaw = self.sample_path_yaw(self.path, cumulative, s)

        return x, y, yaw

    def compute_path_lengths(self, path: Path):
        cumulative = [0.0]
        total = 0.0

        for i in range(1, len(path.poses)):
            p0 = path.poses[i - 1].pose.position
            p1 = path.poses[i].pose.position
            seg = math.hypot(p1.x - p0.x, p1.y - p0.y)
            total += seg
            cumulative.append(total)

        return total, cumulative

    def sample_path(self, path: Path, cumulative: List[float], s: float):
        if s <= 0.0:
            p = path.poses[0].pose.position
            return p.x, p.y

        if s >= cumulative[-1]:
            p = path.poses[-1].pose.position
            return p.x, p.y

        for i in range(1, len(cumulative)):
            if cumulative[i] >= s:
                p0 = path.poses[i - 1].pose.position
                p1 = path.poses[i].pose.position

                s0 = cumulative[i - 1]
                s1 = cumulative[i]

                ratio = 0.0 if s1 <= s0 else (s - s0) / (s1 - s0)

                return (
                    p0.x + ratio * (p1.x - p0.x),
                    p0.y + ratio * (p1.y - p0.y),
                )

        p = path.poses[-1].pose.position
        return p.x, p.y

    def sample_path_yaw(self, path: Path, cumulative: List[float], s: float):
        s1 = max(0.0, s - 0.15)
        s2 = min(cumulative[-1], s + 0.15)

        p1 = self.sample_path(path, cumulative, s1)
        p2 = self.sample_path(path, cumulative, s2)

        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]

        if math.hypot(dx, dy) < 0.01:
            return self.yaw

        return math.atan2(dy, dx)

    def compute_control(self, target):
        tx, ty, target_yaw = target

        dx = tx - self.world_x
        dy = ty - self.world_y

        distance = math.hypot(dx, dy)

        if distance < self.goal_tolerance_m:
            heading_error = normalize_angle(target_yaw - self.yaw)
            if abs(heading_error) < 0.25:
                return 0.0, 0.0

        point_heading = math.atan2(dy, dx)
        point_error = normalize_angle(point_heading - self.yaw)
        path_error = normalize_angle(target_yaw - self.yaw)

        heading_error = normalize_angle(0.75 * point_error + 0.25 * path_error)

        linear = min(self.linear_gain * distance, self.max_linear_speed)

        if abs(heading_error) > 1.25:
            linear = 0.0
        elif abs(heading_error) > 0.85:
            linear *= 0.25
        elif abs(heading_error) > 0.50:
            linear *= 0.60

        angular = self.angular_gain * heading_error
        angular = max(-self.max_angular_speed, min(self.max_angular_speed, angular))

        return linear, angular

    def collision_speed_scale(self):
        nearest = 999.0

        for name, robot in self.other_robots.items():
            if name == self.robot_name or not robot.active:
                continue

            d = math.hypot(robot.x - self.world_x, robot.y - self.world_y)
            nearest = min(nearest, d)

        if nearest < self.collision_stop_m:
            return 0.0

        if nearest < self.collision_slow_m:
            return (nearest - self.collision_stop_m) / (
                self.collision_slow_m - self.collision_stop_m
            )

        return 1.0

    def publish_cmd(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()