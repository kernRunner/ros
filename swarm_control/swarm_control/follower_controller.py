import math
from typing import Dict, Optional, List, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from swarm_interfaces.msg import RobotState


class FollowerController(Node):
    def __init__(self):
        super().__init__('follower_controller')

        # ---- Parameters ----
        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('follow_distance', 1.1)
        self.declare_parameter('stop_distance', 0.75)
        self.declare_parameter('slow_distance', 1.5)

        self.declare_parameter('max_linear_speed', 0.09)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('angular_gain', 0.8)
        self.declare_parameter('linear_gain', 0.45)
        self.declare_parameter('lateral_gain', 0.50)

        self.declare_parameter('state_timeout_sec', 1.0)
        self.declare_parameter('enabled', True)

        self.declare_parameter('repulsion_radius', 0.8)
        self.declare_parameter('repulsion_gain', 0.35)

        self.declare_parameter('parent_filter_alpha', 0.88)

        # ---- Load params ----
        self.robot_name = self.get_parameter('robot_name').value
        self.follow_distance = float(self.get_parameter('follow_distance').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.slow_distance = float(self.get_parameter('slow_distance').value)

        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.angular_gain = float(self.get_parameter('angular_gain').value)
        self.linear_gain = float(self.get_parameter('linear_gain').value)
        self.lateral_gain = float(self.get_parameter('lateral_gain').value)

        self.state_timeout_sec = float(self.get_parameter('state_timeout_sec').value)
        self.enabled = bool(self.get_parameter('enabled').value)

        self.repulsion_radius = float(self.get_parameter('repulsion_radius').value)
        self.repulsion_gain = float(self.get_parameter('repulsion_gain').value)

        self.parent_filter_alpha = float(self.get_parameter('parent_filter_alpha').value)

        # ---- Robot state ----
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.current_role = 'follower'
        self.current_leader_id = ''
        self.current_parent_id = ''

        self.last_states: Dict[str, RobotState] = {}
        self.last_status_log = ''

        self.last_cmd_linear = 0.0
        self.last_cmd_angular = 0.0

        # Smoothed parent pose
        self.filtered_parent_x = 0.0
        self.filtered_parent_y = 0.0
        self.filtered_parent_yaw = 0.0
        self.parent_filter_initialized = False
        self.last_parent_id = ''

        # ---- ROS interfaces ----
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        self.state_sub = self.create_subscription(
            RobotState,
            '/swarm/robot_states',
            self.state_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(f'[{self.robot_name}] follower_controller started')

    # --------------------------------------------------

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def state_callback(self, msg: RobotState):
        self.last_states[msg.robot_name] = msg

        if msg.robot_name == self.robot_name:
            self.current_role = msg.role
            self.current_leader_id = msg.leader_id

    # --------------------------------------------------

    def get_valid_state(self, robot_name: str) -> Optional[RobotState]:
        if robot_name not in self.last_states:
            return None

        state = self.last_states[robot_name]
        now = self.get_clock().now().nanoseconds
        stamp_ns = int(state.stamp.sec) * 1_000_000_000 + int(state.stamp.nanosec)
        age = (now - stamp_ns) / 1e9

        if age > self.state_timeout_sec or not state.active:
            return None

        return state

    def get_valid_states(self) -> Dict[str, RobotState]:
        valid = {}
        for name in self.last_states:
            s = self.get_valid_state(name)
            if s is not None:
                valid[name] = s
        return valid

    # --------------------------------------------------

    def get_chain_order(self, valid_states: Dict[str, RobotState]) -> List[str]:
        """
        Returns chain order starting with leader, then followers.
        Example: ['robot3', 'robot1', 'robot2']
        """
        if not self.current_leader_id:
            return []

        if self.current_leader_id not in valid_states:
            return []

        followers = []
        for name, state in valid_states.items():
            if name == self.current_leader_id:
                continue
            if state.role == 'follower' and state.leader_id == self.current_leader_id:
                followers.append(name)

        followers.sort()
        return [self.current_leader_id] + followers

    def get_parent_id(self, chain_order: List[str]) -> str:
        if self.robot_name not in chain_order:
            return ''

        idx = chain_order.index(self.robot_name)
        if idx <= 0:
            return ''

        return chain_order[idx - 1]

    # --------------------------------------------------

    def reset_parent_filter_if_needed(self):
        if self.current_parent_id != self.last_parent_id:
            self.parent_filter_initialized = False
            self.last_parent_id = self.current_parent_id
            self.last_cmd_linear = 0.0
            self.last_cmd_angular = 0.0

    def update_filtered_parent_pose(self, parent_state: RobotState):
        alpha = self.parent_filter_alpha

        if not self.parent_filter_initialized:
            self.filtered_parent_x = parent_state.x
            self.filtered_parent_y = parent_state.y
            self.filtered_parent_yaw = parent_state.yaw
            self.parent_filter_initialized = True
            return

        self.filtered_parent_x = alpha * self.filtered_parent_x + (1.0 - alpha) * parent_state.x
        self.filtered_parent_y = alpha * self.filtered_parent_y + (1.0 - alpha) * parent_state.y

        dyaw = self.normalize_angle(parent_state.yaw - self.filtered_parent_yaw)
        self.filtered_parent_yaw += (1.0 - alpha) * dyaw

    def compute_follow_target(self) -> Tuple[float, float]:
        """
        Target point directly behind parent.
        """
        tx = self.filtered_parent_x - self.follow_distance * math.cos(self.filtered_parent_yaw)
        ty = self.filtered_parent_y - self.follow_distance * math.sin(self.filtered_parent_yaw)
        return tx, ty

    # --------------------------------------------------

    def compute_repulsion(self, valid_states: Dict[str, RobotState]) -> Tuple[float, float]:
        rep_x = 0.0
        rep_y = 0.0

        for name, state in valid_states.items():
            if name == self.robot_name:
                continue

            dx = self.x - state.x
            dy = self.y - state.y
            dist = math.hypot(dx, dy)

            if dist < self.repulsion_radius and dist > 0.01:
                strength = (self.repulsion_radius - dist) / self.repulsion_radius
                rep_x += (dx / dist) * strength
                rep_y += (dy / dist) * strength

        return rep_x, rep_y

    # --------------------------------------------------

    def publish_cmd(self, linear_x: float, angular_z: float):
        alpha_lin = 0.72
        alpha_ang = 0.62

        self.last_cmd_linear = alpha_lin * self.last_cmd_linear + (1.0 - alpha_lin) * linear_x
        self.last_cmd_angular = alpha_ang * self.last_cmd_angular + (1.0 - alpha_ang) * angular_z

        cmd = Twist()
        cmd.linear.x = self.last_cmd_linear
        cmd.angular.z = self.last_cmd_angular
        self.cmd_pub.publish(cmd)

    # --------------------------------------------------

    def control_loop(self):
        if not self.enabled:
            return

        if self.current_role != 'follower':
            return

        if not self.current_leader_id or self.current_leader_id == self.robot_name:
            return

        valid_states = self.get_valid_states()
        chain_order = self.get_chain_order(valid_states)

        self.current_parent_id = self.get_parent_id(chain_order)
        if not self.current_parent_id:
            return

        parent_state = self.get_valid_state(self.current_parent_id)
        if parent_state is None:
            return

        self.reset_parent_filter_if_needed()
        self.update_filtered_parent_pose(parent_state)

        tx, ty = self.compute_follow_target()

        # Attraction toward parent trail target
        dx = tx - self.x
        dy = ty - self.y

        # Repulsion away from nearby robots
        rep_x, rep_y = self.compute_repulsion(valid_states)
        dx += self.repulsion_gain * rep_x
        dy += self.repulsion_gain * rep_y

        distance = math.hypot(dx, dy)

        # Target in follower local frame
        local_x = math.cos(self.yaw) * dx + math.sin(self.yaw) * dy
        local_y = -math.sin(self.yaw) * dx + math.cos(self.yaw) * dy

        target_heading = math.atan2(dy, dx)
        heading_error = self.normalize_angle(target_heading - self.yaw)

        # Angular control: heading + explicit lateral correction
        heading_term = self.angular_gain * heading_error
        lateral_term = self.lateral_gain * local_y
        angular_z = heading_term + lateral_term
        angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_z))

        # Linear control
        if distance <= self.stop_distance:
            linear_x = 0.0
        else:
            distance_error = max(0.0, distance - self.follow_distance)
            linear_x = self.linear_gain * distance_error
            linear_x = min(self.max_linear_speed, linear_x)

            if distance < self.slow_distance:
                ratio = (distance - self.stop_distance) / max(0.01, (self.slow_distance - self.stop_distance))
                ratio = max(0.0, min(1.0, ratio))
                linear_x *= ratio

        # Penalize forward motion when sideways error is large
        side_error = abs(local_y)
        if side_error > 0.35:
            linear_x *= 0.35
        elif side_error > 0.20:
            linear_x *= 0.65

        # Penalize forward motion when facing away
        abs_err = abs(heading_error)
        if abs_err > 1.2:
            linear_x *= 0.0
        elif abs_err > 0.8:
            linear_x *= 0.3
        elif abs_err > 0.4:
            linear_x *= 0.65

        self.publish_cmd(linear_x, angular_z)
        self.log_status(distance, heading_error, local_y, linear_x, angular_z)

    # --------------------------------------------------

    def log_status(self, d: float, err: float, local_y: float, v: float, w: float):
        msg = (
            f'parent={self.current_parent_id}, d={d:.2f}, err={err:.2f}, '
            f'lat={local_y:.2f}, v={v:.2f}, w={w:.2f}'
        )
        if msg != self.last_status_log:
            self.get_logger().info(f'[{self.robot_name}] {msg}')
            self.last_status_log = msg

    # --------------------------------------------------

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        return math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z)
        )

    @staticmethod
    def normalize_angle(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a


def main(args=None):
    rclpy.init(args=args)
    node = FollowerController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()