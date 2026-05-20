import math
import re
from typing import Dict, Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from swarm_interfaces.msg import RobotState

from swarm_control.core.math_utils import normalize_angle, quaternion_to_yaw


class FormationManager(Node):
    def __init__(self):
        super().__init__('formation_manager')

        self.declare_parameter('robot_name', 'robot2')
        self.declare_parameter('leader_name', 'robot1')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel_raw')

        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)

        self.declare_parameter('chain_spacing_m', 1.8)
        self.declare_parameter('slot_index', -1)

        self.declare_parameter('goal_tolerance_m', 0.12)
        self.declare_parameter('yaw_tolerance_rad', 0.25)

        self.declare_parameter('max_linear_speed', 0.10)
        self.declare_parameter('max_angular_speed', 0.75)

        self.declare_parameter('linear_gain', 0.45)
        self.declare_parameter('angular_gain', 1.60)

        self.declare_parameter('collision_stop_m', 0.70)
        self.declare_parameter('collision_slow_m', 1.10)

        self.robot_name = self.get_parameter('robot_name').value
        self.leader_name = self.get_parameter('leader_name').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.spawn_x = float(self.get_parameter('spawn_x').value)
        self.spawn_y = float(self.get_parameter('spawn_y').value)

        self.chain_spacing_m = float(self.get_parameter('chain_spacing_m').value)
        self.slot_index = int(self.get_parameter('slot_index').value)

        self.goal_tolerance_m = float(self.get_parameter('goal_tolerance_m').value)
        self.yaw_tolerance_rad = float(self.get_parameter('yaw_tolerance_rad').value)

        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)

        self.linear_gain = float(self.get_parameter('linear_gain').value)
        self.angular_gain = float(self.get_parameter('angular_gain').value)

        self.collision_stop_m = float(self.get_parameter('collision_stop_m').value)
        self.collision_slow_m = float(self.get_parameter('collision_slow_m').value)

        self.world_x = 0.0
        self.world_y = 0.0
        self.yaw = 0.0

        self.ready = False
        self.states: Dict[str, RobotState] = {}

        if self.slot_index < 0:
            self.slot_index = self._index_from_name(self.robot_name)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.ready_pub = self.create_publisher(Bool, 'formation_ready', 10)

        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(RobotState, '/swarm/robot_states', self.state_callback, 10)

        self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f'[{self.robot_name}] formation_manager started, slot_index={self.slot_index}'
        )

    def _index_from_name(self, name: str) -> int:
        match = re.search(r'(\d+)$', name)
        if not match:
            return 1
        return max(0, int(match.group(1)) - 1)

    def odom_callback(self, msg: Odometry):
        self.world_x = self.spawn_x + msg.pose.pose.position.x
        self.world_y = self.spawn_y + msg.pose.pose.position.y
        self.yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def state_callback(self, msg: RobotState):
        self.states[msg.robot_name] = msg

    def control_loop(self):
        if self.robot_name == self.leader_name:
            self._set_ready()
            self._publish_cmd(0.0, 0.0)
            return

        leader = self.states.get(self.leader_name)
        if leader is None or not leader.active:
            self._publish_cmd(0.0, 0.0)
            return

        tx, ty, tyaw = self._slot_pose(leader)

        dx = tx - self.world_x
        dy = ty - self.world_y
        distance = math.hypot(dx, dy)

        yaw_error = normalize_angle(tyaw - self.yaw)

        if distance < self.goal_tolerance_m and abs(yaw_error) < self.yaw_tolerance_rad:
            self._set_ready()
            self._publish_cmd(0.0, 0.0)
            return

        linear, angular = self._go_to(tx, ty, tyaw)

        scale = self._collision_speed_scale()
        linear *= scale

        if scale <= 0.01:
            linear = 0.0

        self._publish_cmd(linear, angular)

    def _slot_pose(self, leader: RobotState):
        # Line behind leader along leader yaw.
        # slot_index: robot2=1, robot3=2, ...
        yaw = getattr(leader, 'yaw', 0.0)

        tx = leader.x - self.slot_index * self.chain_spacing_m * math.cos(yaw)
        ty = leader.y - self.slot_index * self.chain_spacing_m * math.sin(yaw)

        return tx, ty, yaw

    def _go_to(self, tx: float, ty: float, target_yaw: float):
        dx = tx - self.world_x
        dy = ty - self.world_y

        distance = math.hypot(dx, dy)

        if distance > self.goal_tolerance_m:
            target_heading = math.atan2(dy, dx)
            heading_error = normalize_angle(target_heading - self.yaw)

            linear = min(self.linear_gain * distance, self.max_linear_speed)

            if abs(heading_error) > 1.2:
                linear = 0.0
            elif abs(heading_error) > 0.8:
                linear *= 0.25
            elif abs(heading_error) > 0.45:
                linear *= 0.55

            angular = self.angular_gain * heading_error
        else:
            linear = 0.0
            angular = self.angular_gain * normalize_angle(target_yaw - self.yaw)

        angular = max(-self.max_angular_speed, min(self.max_angular_speed, angular))
        return linear, angular

    def _collision_speed_scale(self):
        nearest = 999.0

        for name, robot in self.states.items():
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

    def _set_ready(self):
        if not self.ready:
            self.ready = True
            self.get_logger().info(f'[{self.robot_name}] formation_ready=True')

        msg = Bool()
        msg.data = True
        self.ready_pub.publish(msg)

    def _publish_cmd(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FormationManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_cmd(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()