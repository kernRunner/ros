import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from swarm_interfaces.msg import RobotState

from swarm_control.core.math_utils import quaternion_to_yaw


class FormationManager(Node):
    """
    Startup gate for the swarm.

    This version intentionally does NOT drive followers into an exact line.
    Your earlier formation controller made robots cross through each other
    from the square start layout. Instead, this node waits until the elected
    leader is known, optionally waits a short time, then publishes
    /<robot>/formation_ready=True.

    Actual follower motion is handled by path_follower.py.
    """

    def __init__(self):
        super().__init__('formation_manager')

        self._declare_parameters()
        self._read_parameters()
        self._init_state()
        self._init_ros_interfaces()

        self.get_logger().info(f'[{self.robot_name}] formation_manager started')

    def _declare_parameters(self):
        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel_raw')

        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)

        # Compatibility parameters kept so your launch file can still pass them.
        self.declare_parameter('slot_spacing_m', 1.4)
        self.declare_parameter('chain_spacing_m', 1.4)
        self.declare_parameter('arrival_tolerance_m', 0.25)
        self.declare_parameter('goal_tolerance_m', 0.25)
        self.declare_parameter('yaw_tolerance_rad', 0.35)
        self.declare_parameter('max_linear_speed', 0.06)
        self.declare_parameter('max_angular_speed', 0.55)
        self.declare_parameter('linear_gain', 0.35)
        self.declare_parameter('angular_gain', 1.10)
        self.declare_parameter('collision_stop_m', 0.35)
        self.declare_parameter('collision_slow_m', 0.80)
        self.declare_parameter('staged_formation', True)
        self.declare_parameter('slot_start_delay_sec', 5.0)

        # This is the only behavior this simplified manager uses.
        self.declare_parameter('post_election_wait_sec', 2.0)

    def _read_parameters(self):
        self.robot_name = self.get_parameter('robot_name').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.spawn_x = float(self.get_parameter('spawn_x').value)
        self.spawn_y = float(self.get_parameter('spawn_y').value)

        self.post_election_wait_sec = float(
            self.get_parameter('post_election_wait_sec').value
        )

    def _init_state(self):
        self.world_x = 0.0
        self.world_y = 0.0
        self.yaw = 0.0

        self.current_role = 'follower'
        self.current_leader_id = ''
        self.leader_detected_time_ns = None

        self.ready = False
        self.states = {}

    def _init_ros_interfaces(self):
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.ready_pub = self.create_publisher(Bool, 'formation_ready', 10)

        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(
            RobotState,
            '/swarm/robot_states',
            self.state_callback,
            10,
        )

        self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg: Odometry):
        self.world_x = self.spawn_x + msg.pose.pose.position.x
        self.world_y = self.spawn_y + msg.pose.pose.position.y
        self.yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def state_callback(self, msg: RobotState):
        self.states[msg.robot_name] = msg

        if msg.robot_name != self.robot_name:
            return

        old_leader = self.current_leader_id

        self.current_role = msg.role
        self.current_leader_id = msg.leader_id

        if self.current_leader_id and self.current_leader_id != old_leader:
            self.leader_detected_time_ns = self.get_clock().now().nanoseconds
            self.ready = False
            self.get_logger().info(
                f'[{self.robot_name}] detected leader: {self.current_leader_id}'
            )

    def control_loop(self):
        """
        Do not drive the robot here.

        This prevents command fighting with path_follower/tree_explorer.
        It only publishes formation_ready after a leader exists and the
        short startup delay is finished.
        """
        if not self.current_leader_id:
            self._publish_ready(False)
            return

        if not self._post_election_wait_done():
            self._publish_ready(False)
            return

        self._set_ready()

    def _post_election_wait_done(self) -> bool:
        if self.leader_detected_time_ns is None:
            return False

        elapsed = (
            self.get_clock().now().nanoseconds - self.leader_detected_time_ns
        ) / 1e9

        return elapsed >= self.post_election_wait_sec

    def _set_ready(self):
        if not self.ready:
            self.ready = True
            self.get_logger().info(f'[{self.robot_name}] formation_ready=True')

        self._publish_ready(True)

    def _publish_ready(self, value: bool):
        msg = Bool()
        msg.data = value
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
        # One final stop is okay during shutdown.
        node._publish_cmd(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
