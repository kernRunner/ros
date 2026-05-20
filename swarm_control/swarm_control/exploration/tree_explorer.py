import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from swarm_interfaces.msg import RobotState

from swarm_control.core.cmd_utils import make_twist
from swarm_control.core.math_utils import quaternion_to_yaw
from swarm_control.core.scan_utils import sector_min, sector_avg


class TreeExplorer(Node):
    def __init__(self):
        super().__init__('tree_explorer')

        self._declare_parameters()
        self._read_parameters()
        self._init_state()
        self._init_ros_interfaces()

        self.get_logger().info(f'[{self.robot_name}] tree_explorer started')

    def _declare_parameters(self):
        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel_raw')

        self.declare_parameter('forward_speed', 0.18)
        self.declare_parameter('turn_speed', 0.75)
        self.declare_parameter('front_blocked_distance', 0.85)

        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)

        self.declare_parameter('chain_spacing_m', 1.3)
        self.declare_parameter('formation_tolerance_m', 0.35)

    def _read_parameters(self):
        self.robot_name = self.get_parameter('robot_name').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.front_blocked_distance = float(
            self.get_parameter('front_blocked_distance').value
        )

        self.spawn_x = float(self.get_parameter('spawn_x').value)
        self.spawn_y = float(self.get_parameter('spawn_y').value)

        self.chain_spacing_m = float(self.get_parameter('chain_spacing_m').value)
        self.formation_tolerance_m = float(
            self.get_parameter('formation_tolerance_m').value
        )

    def _init_state(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.is_leader = False
        self.current_role = 'follower'
        self.current_leader_id = ''

        self.other_robots = {}

        self.front = float('inf')
        self.front_left = float('inf')
        self.front_right = float('inf')

    def _init_ros_interfaces(self):
        self.cmd_pub = self.create_publisher(
            make_twist().__class__,
            self.cmd_vel_topic,
            10,
        )

        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            RobotState,
            '/swarm/robot_states',
            self.state_callback,
            10,
        )

        self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg: Odometry):
        self.x = self.spawn_x + msg.pose.pose.position.x
        self.y = self.spawn_y + msg.pose.pose.position.y
        self.yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def state_callback(self, msg: RobotState):
        self.other_robots[msg.robot_name] = msg

        if msg.robot_name != self.robot_name:
            return

        self.current_role = msg.role
        self.current_leader_id = msg.leader_id
        self.is_leader = msg.role == 'leader' and msg.leader_id == self.robot_name

    def scan_callback(self, msg: LaserScan):
        if not msg.ranges:
            return

        self.front = sector_min(msg, 0.0, 0.25)
        self.front_left = sector_avg(msg, 0.65, 0.35)
        self.front_right = sector_avg(msg, -0.65, 0.35)

    def control_loop(self):
        if not self.is_leader:
            return

        linear, angular = self._compute_exploration_command()
        self.cmd_pub.publish(make_twist(linear, angular))


    def _compute_exploration_command(self):
        if self.front < self.front_blocked_distance:
            return self._turn_toward_open_space()

        return self.forward_speed, 0.0

    def _turn_toward_open_space(self):
        if self.front_left >= self.front_right:
            return 0.02, self.turn_speed

        return 0.02, -self.turn_speed

    def _publish_stop(self):
        self.cmd_pub.publish(make_twist())


def main(args=None):
    rclpy.init(args=args)
    node = TreeExplorer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()