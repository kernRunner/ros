import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from swarm_interfaces.msg import RobotState


class FormationManager(Node):
    """
    Startup gate only.

    This node does not move robots.
    It waits until a leader is elected, waits a short delay,
    then publishes formation_ready=True.

    The leader movement is handled by tree_explorer.py.
    Follower joining/spacing is handled by path_follower.py.
    """

    def __init__(self):
        super().__init__('formation_manager')

        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('post_election_wait_sec', 1.0)

        self.robot_name = self.get_parameter('robot_name').value
        self.post_election_wait_sec = float(
            self.get_parameter('post_election_wait_sec').value
        )

        self.current_leader_id = ''
        self.leader_detected_time_ns = None
        self.released = False

        self.ready_pub = self.create_publisher(Bool, 'formation_ready', 10)

        self.create_subscription(
            RobotState,
            '/swarm/robot_states',
            self.state_callback,
            10,
        )

        self.create_timer(0.1, self.control_loop)

        self.get_logger().info(f'[{self.robot_name}] formation_manager started')

    def state_callback(self, msg: RobotState):
        if msg.robot_name != self.robot_name:
            return

        old_leader = self.current_leader_id
        self.current_leader_id = msg.leader_id

        if self.current_leader_id and self.current_leader_id != old_leader:
            self.released = False
            self.leader_detected_time_ns = self.get_clock().now().nanoseconds
            self.get_logger().info(
                f'[{self.robot_name}] detected leader: {self.current_leader_id}'
            )

    def control_loop(self):
        if self.released:
            self.publish_ready(True)
            return

        if not self.current_leader_id:
            self.publish_ready(False)
            return

        if not self.post_election_wait_done():
            self.publish_ready(False)
            return

        self.released = True
        self.publish_ready(True)
        self.get_logger().info(f'[{self.robot_name}] formation_ready=True')

    def post_election_wait_done(self):
        if self.leader_detected_time_ns is None:
            return False

        elapsed = (
            self.get_clock().now().nanoseconds - self.leader_detected_time_ns
        ) / 1e9

        return elapsed >= self.post_election_wait_sec

    def publish_ready(self, value: bool):
        msg = Bool()
        msg.data = value
        self.ready_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FormationManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()