import json
import math
from typing import Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from swarm_interfaces.msg import RobotState


class RelayTreeManager(Node):
    """
    First scripted relay-tree manager for 6 robots.

    Behavior:
      1. robot1 stays at the start as root relay.
      2. robot2 leads robot3-robot6 east as group_0.
      3. Once robot2 moves split_distance_m away from robot1:
           - robot2 becomes a relay.
           - robot3 leads robot4 as group_0_A at +branch_angle_deg.
           - robot5 leads robot6 as group_0_B at -branch_angle_deg.

    This is intentionally scripted first. After it works, the same logic can be
    made recursive.
    """

    def __init__(self):
        super().__init__('relay_tree_manager')

        self._declare_parameters()
        self._read_parameters()
        self._init_state()
        self._init_ros_interfaces()

        self.get_logger().info('[relay_tree_manager] started')

    def _declare_parameters(self):
        self.declare_parameter(
            'robot_names',
            ['robot1', 'robot2', 'robot3', 'robot4', 'robot5', 'robot6'],
        )
        self.declare_parameter('root_relay_name', 'robot1')
        self.declare_parameter('initial_leader_name', 'robot2')
        self.declare_parameter('split_distance_m', 4.0)
        self.declare_parameter('branch_angle_deg', 35.0)
        self.declare_parameter('publish_rate_hz', 2.0)

    def _read_parameters(self):
        self.robot_names = list(self.get_parameter('robot_names').value)
        self.root_relay_name = self.get_parameter('root_relay_name').value
        self.initial_leader_name = self.get_parameter('initial_leader_name').value
        self.split_distance_m = float(self.get_parameter('split_distance_m').value)
        self.branch_angle_deg = float(self.get_parameter('branch_angle_deg').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

    def _init_state(self):
        self.states: Dict[str, RobotState] = {}
        self.phase = 'INITIAL_GROUP'
        self.last_phase_log = ''

    def _init_ros_interfaces(self):
        self.assignment_pub = self.create_publisher(
            String,
            '/swarm/role_assignments',
            10,
        )

        self.create_subscription(
            RobotState,
            '/swarm/robot_states',
            self.state_callback,
            10,
        )

        period = 1.0 / self.publish_rate_hz
        self.create_timer(period, self.control_loop)

    def state_callback(self, msg: RobotState):
        self.states[msg.robot_name] = msg

    def control_loop(self):
        self._update_phase()
        assignments = self._build_assignments()
        self._publish_assignments(assignments)

    def _update_phase(self):
        if self.phase != 'INITIAL_GROUP':
            return

        root = self.states.get(self.root_relay_name)
        leader = self.states.get(self.initial_leader_name)

        if root is None or leader is None:
            return

        dist = math.hypot(leader.x - root.x, leader.y - root.y)

        if dist >= self.split_distance_m:
            self.phase = 'FIRST_SPLIT'
            self.get_logger().info(
                f'[relay_tree_manager] first split triggered at distance={dist:.2f}m'
            )

    def _build_assignments(self):
        if self.phase == 'FIRST_SPLIT':
            return self._first_split_assignments()

        return self._initial_assignments()

    def _initial_assignments(self):
        assignments = {}

        for name in self.robot_names:
            assignments[name] = {
                'role': 'group_follower',
                'leader_id': self.initial_leader_name,
                'group_id': 'group_0',
                'parent_group_id': 'root',
                'parent_relay_id': self.root_relay_name,
                'assigned_heading_deg': 0.0,
                'branch_depth': 1,
                'active': True,
            }

        assignments[self.root_relay_name] = {
            'role': 'root_relay',
            'leader_id': '',
            'group_id': 'root',
            'parent_group_id': '',
            'parent_relay_id': '',
            'assigned_heading_deg': 0.0,
            'branch_depth': 0,
            'active': True,
        }

        assignments[self.initial_leader_name] = {
            'role': 'group_leader',
            'leader_id': self.initial_leader_name,
            'group_id': 'group_0',
            'parent_group_id': 'root',
            'parent_relay_id': self.root_relay_name,
            'assigned_heading_deg': 0.0,
            'branch_depth': 1,
            'active': True,
        }

        return assignments

    def _first_split_assignments(self):
        assignments = {}

        assignments['robot1'] = {
            'role': 'root_relay',
            'leader_id': '',
            'group_id': 'root',
            'parent_group_id': '',
            'parent_relay_id': '',
            'assigned_heading_deg': 0.0,
            'branch_depth': 0,
            'active': True,
        }

        assignments['robot2'] = {
            'role': 'relay',
            'leader_id': '',
            'group_id': 'group_0',
            'parent_group_id': 'root',
            'parent_relay_id': 'robot1',
            'assigned_heading_deg': 0.0,
            'branch_depth': 1,
            'active': True,
        }

        assignments['robot3'] = {
            'role': 'group_leader',
            'leader_id': 'robot3',
            'group_id': 'group_0_A',
            'parent_group_id': 'group_0',
            'parent_relay_id': 'robot2',
            'assigned_heading_deg': self.branch_angle_deg,
            'branch_depth': 2,
            'active': True,
        }

        assignments['robot4'] = {
            'role': 'group_follower',
            'leader_id': 'robot3',
            'group_id': 'group_0_A',
            'parent_group_id': 'group_0',
            'parent_relay_id': 'robot2',
            'assigned_heading_deg': self.branch_angle_deg,
            'branch_depth': 2,
            'active': True,
        }

        assignments['robot5'] = {
            'role': 'group_leader',
            'leader_id': 'robot5',
            'group_id': 'group_0_B',
            'parent_group_id': 'group_0',
            'parent_relay_id': 'robot2',
            'assigned_heading_deg': -self.branch_angle_deg,
            'branch_depth': 2,
            'active': True,
        }

        assignments['robot6'] = {
            'role': 'group_follower',
            'leader_id': 'robot5',
            'group_id': 'group_0_B',
            'parent_group_id': 'group_0',
            'parent_relay_id': 'robot2',
            'assigned_heading_deg': -self.branch_angle_deg,
            'branch_depth': 2,
            'active': True,
        }

        return assignments

    def _publish_assignments(self, assignments: dict):
        if self.phase != self.last_phase_log:
            self.last_phase_log = self.phase
            self.get_logger().info(f'[relay_tree_manager] phase={self.phase}')

        msg = String()
        msg.data = json.dumps(assignments)
        self.assignment_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RelayTreeManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
