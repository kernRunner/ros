import json
import math
import re
from dataclasses import dataclass
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from swarm_interfaces.msg import RobotState


@dataclass
class ActiveGroup:
    group_id: str
    parent_group_id: str
    parent_relay_id: str
    robot_names: List[str]
    leader_name: str
    heading_deg: float
    branch_depth: int
    created_ns: int = 0


@dataclass
class RelayRecord:
    robot_name: str
    group_id: str
    parent_group_id: str
    parent_relay_id: str
    heading_deg: float
    branch_depth: int
    role: str = 'relay'


class RelayTreeManager(Node):
    """
    Recursive relay-tree manager.

    Behavior:
      1. The physically rearmost robot becomes the root relay.
      2. All remaining robots form group_0 behind initial_leader_name.
      3. Each active group moves away from its parent relay.
      4. If an active group has at least min_group_size_to_split robots and
         its leader is split_distance_m away from its parent relay:
           - the current rearmost robot in that group becomes a relay.
           - all remaining robots split into left/right child groups.
      5. Groups with fewer than min_group_size_to_split robots continue
         exploring but do not split further.

    With min_group_size_to_split = 3:
      [A, B, C] -> C relay, A left child, B right child
    """

    def __init__(self):
        super().__init__('relay_tree_manager')

        self._declare_parameters()
        self._read_parameters()
        self._init_state()
        self._init_ros_interfaces()

        self.get_logger().info('[relay_tree_manager_recursive] started')

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------

    def _declare_parameters(self):
        self.declare_parameter(
            'robot_names',
            ['robot1', 'robot2', 'robot3', 'robot4', 'robot5', 'robot6'],
        )
        self.declare_parameter('initial_leader_name', 'robot1')
        self.declare_parameter('initial_heading_deg', 0.0)

        self.declare_parameter('split_distance_m', 25.0)
        self.declare_parameter('branch_angle_deg', 30.0)
        self.declare_parameter('publish_rate_hz', 1.0)

        self.declare_parameter('min_group_size_to_split', 3)
        self.declare_parameter('max_branch_depth', 3)

        # Prevent instant re-splitting right after a group is created.
        self.declare_parameter('min_group_age_before_split_sec', 8.0)

        # If true, groups with one robot still get role group_leader.
        self.declare_parameter('single_robot_groups_are_leaders', True)

    def _read_parameters(self):
        self.robot_names = list(self.get_parameter('robot_names').value)
        self.initial_leader_name = str(
            self.get_parameter('initial_leader_name').value
        )
        self.initial_heading_deg = float(
            self.get_parameter('initial_heading_deg').value
        )

        self.split_distance_m = float(self.get_parameter('split_distance_m').value)
        self.branch_angle_deg = float(self.get_parameter('branch_angle_deg').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.min_group_size_to_split = int(
            self.get_parameter('min_group_size_to_split').value
        )
        self.max_branch_depth = int(self.get_parameter('max_branch_depth').value)

        self.min_group_age_before_split_sec = float(
            self.get_parameter('min_group_age_before_split_sec').value
        )
        self.single_robot_groups_are_leaders = bool(
            self.get_parameter('single_robot_groups_are_leaders').value
        )

        if self.min_group_size_to_split < 3:
            self.get_logger().warn(
                'min_group_size_to_split must be at least 3 for relay+left+right. '
                'Forcing it to 3.'
            )
            self.min_group_size_to_split = 3

    # ------------------------------------------------------------------
    # Initialization
    # ------------------------------------------------------------------

    def _init_state(self):
        self.states: Dict[str, RobotState] = {}
        self.phase = 'WAIT_FOR_STATES'
        self.last_phase_log = ''

        self.root_relay_name = ''
        self.relays: Dict[str, RelayRecord] = {}
        self.active_groups: Dict[str, ActiveGroup] = {}

        self.split_counter = 0
        self.last_assignment_json = ''

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
        self.create_timer(1.0 / self.publish_rate_hz, self.control_loop)

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def state_callback(self, msg: RobotState):
        self.states[msg.robot_name] = msg

    def control_loop(self):
        self._update_tree()
        assignments = self._build_assignments()
        self._publish_assignments(assignments)

    # ------------------------------------------------------------------
    # Tree update
    # ------------------------------------------------------------------

    def _update_tree(self):
        if not self._have_required_states():
            self.phase = 'WAIT_FOR_STATES'
            return

        if self.phase == 'WAIT_FOR_STATES':
            self._initialize_tree()
            return

        if self.phase != 'RUNNING':
            return

        # Iterate over a copy because splitting modifies active_groups.
        for group_id in list(self.active_groups.keys()):
            group = self.active_groups.get(group_id)
            if group is None:
                continue

            if self._group_should_split(group):
                self._split_group(group)

    def _initialize_tree(self):
        self.root_relay_name = self._choose_rearmost_robot(
            self.robot_names,
            self.initial_leader_name,
            self.initial_heading_deg,
        )

        if not self.root_relay_name:
            return

        moving_names = [
            name for name in self.robot_names
            if name != self.root_relay_name
        ]
        moving_names = self._sort_front_to_back(
            moving_names,
            self.initial_leader_name,
            self.initial_heading_deg,
        )

        if not moving_names:
            return

        if self.initial_leader_name in moving_names:
            leader_name = self.initial_leader_name
        else:
            leader_name = moving_names[0]

        now_ns = self.get_clock().now().nanoseconds

        self.relays[self.root_relay_name] = RelayRecord(
            robot_name=self.root_relay_name,
            group_id='root',
            parent_group_id='',
            parent_relay_id='',
            heading_deg=self.initial_heading_deg,
            branch_depth=0,
            role='root_relay',
        )

        self.active_groups['group_0'] = ActiveGroup(
            group_id='group_0',
            parent_group_id='root',
            parent_relay_id=self.root_relay_name,
            robot_names=moving_names,
            leader_name=leader_name,
            heading_deg=self.initial_heading_deg,
            branch_depth=1,
            created_ns=now_ns,
        )

        self.phase = 'RUNNING'
        self.get_logger().info(
            f'[relay_tree_manager_recursive] root relay={self.root_relay_name}; '
            f'group_0={moving_names}'
        )

    def _group_should_split(self, group: ActiveGroup) -> bool:
        if len(group.robot_names) < self.min_group_size_to_split:
            return False

        if group.branch_depth >= self.max_branch_depth:
            return False

        now_ns = self.get_clock().now().nanoseconds
        age_sec = (now_ns - group.created_ns) / 1e9
        if age_sec < self.min_group_age_before_split_sec:
            return False

        parent_relay = self.states.get(group.parent_relay_id)
        leader = self.states.get(group.leader_name)

        if parent_relay is None or leader is None:
            return False

        distance = math.hypot(
            leader.x - parent_relay.x,
            leader.y - parent_relay.y,
        )

        return distance >= self.split_distance_m

    def _split_group(self, group: ActiveGroup):
        sorted_names = self._sort_front_to_back(
            group.robot_names,
            group.leader_name,
            group.heading_deg,
        )

        if len(sorted_names) < self.min_group_size_to_split:
            return

        relay_name = sorted_names[-1]
        remaining = [name for name in sorted_names if name != relay_name]

        if len(remaining) < 2:
            return

        relay_group_id = f'{group.group_id}_relay'
        self.relays[relay_name] = RelayRecord(
            robot_name=relay_name,
            group_id=group.group_id,
            parent_group_id=group.parent_group_id,
            parent_relay_id=group.parent_relay_id,
            heading_deg=group.heading_deg,
            branch_depth=group.branch_depth,
            role='relay',
        )

        # Remove the old active group.
        self.active_groups.pop(group.group_id, None)

        # Split all remaining robots into left/right child groups.
        # For len(remaining)==2 this gives one robot left and one robot right.
        mid = max(1, math.ceil(len(remaining) / 2.0))
        left_names = remaining[:mid]
        right_names = remaining[mid:]

        left_heading = group.heading_deg + self.branch_angle_deg
        right_heading = group.heading_deg - self.branch_angle_deg

        left_group_id = f'{group.group_id}_A'
        right_group_id = f'{group.group_id}_B'

        now_ns = self.get_clock().now().nanoseconds

        self._create_child_group(
            group_id=left_group_id,
            parent_group_id=group.group_id,
            parent_relay_id=relay_name,
            names=left_names,
            heading_deg=left_heading,
            branch_depth=group.branch_depth + 1,
            created_ns=now_ns,
        )

        self._create_child_group(
            group_id=right_group_id,
            parent_group_id=group.group_id,
            parent_relay_id=relay_name,
            names=right_names,
            heading_deg=right_heading,
            branch_depth=group.branch_depth + 1,
            created_ns=now_ns,
        )

        self.split_counter += 1

        self.get_logger().info(
            f'[relay_tree_manager_recursive] split {group.group_id}: '
            f'relay={relay_name}; '
            f'{left_group_id}={left_names} heading={left_heading:.1f}; '
            f'{right_group_id}={right_names} heading={right_heading:.1f}'
        )

    def _create_child_group(
        self,
        group_id: str,
        parent_group_id: str,
        parent_relay_id: str,
        names: List[str],
        heading_deg: float,
        branch_depth: int,
        created_ns: int,
    ):
        if not names:
            return

        # Keep front-most robot as leader after sorting by the new heading.
        sorted_names = self._sort_front_to_back(
            names,
            names[0],
            heading_deg,
        )

        if not sorted_names:
            sorted_names = list(names)

        leader_name = sorted_names[0]

        self.active_groups[group_id] = ActiveGroup(
            group_id=group_id,
            parent_group_id=parent_group_id,
            parent_relay_id=parent_relay_id,
            robot_names=sorted_names,
            leader_name=leader_name,
            heading_deg=heading_deg,
            branch_depth=branch_depth,
            created_ns=created_ns,
        )

    # ------------------------------------------------------------------
    # Assignment building
    # ------------------------------------------------------------------

    def _build_assignments(self) -> Dict[str, dict]:
        if self.phase == 'WAIT_FOR_STATES':
            return self._waiting_assignments()

        assignments: Dict[str, dict] = {}

        # Relay assignments.
        for relay in self.relays.values():
            assignments[relay.robot_name] = {
                'role': relay.role,
                'leader_id': '',
                'group_id': relay.group_id,
                'parent_group_id': relay.parent_group_id,
                'parent_relay_id': relay.parent_relay_id,
                'assigned_heading_deg': relay.heading_deg,
                'branch_depth': relay.branch_depth,
                'active': True,
            }

        # Active group assignments.
        for group in self.active_groups.values():
            self._assign_active_group(assignments, group)

        # Safety fallback for any robot missing from assignments.
        for name in self.robot_names:
            if name not in assignments:
                assignments[name] = {
                    'role': 'inactive',
                    'leader_id': '',
                    'group_id': '',
                    'parent_group_id': '',
                    'parent_relay_id': '',
                    'assigned_heading_deg': 0.0,
                    'branch_depth': 0,
                    'active': False,
                }

        return assignments

    def _waiting_assignments(self) -> Dict[str, dict]:
        # Important:
        # While waiting for all robot states, do NOT assign follower/leader roles.
        # With require_formation_ready=False, followers can start moving immediately.
        # So during WAIT_FOR_STATES we explicitly hold every robot idle.
        assignments: Dict[str, dict] = {}

        for name in self.robot_names:
            assignments[name] = {
                'role': 'idle',
                'leader_id': '',
                'group_id': '',
                'parent_group_id': '',
                'parent_relay_id': '',
                'assigned_heading_deg': self.initial_heading_deg,
                'branch_depth': 0,
                'active': True,
            }

        return assignments

    def _assign_active_group(
        self,
        assignments: Dict[str, dict],
        group: ActiveGroup,
    ):
        names = [
            name for name in group.robot_names
            if name not in self.relays
        ]

        if not names:
            return

        if group.leader_name not in names:
            group.leader_name = names[0]

        assignments[group.leader_name] = self._leader_assignment(
            leader_id=group.leader_name,
            group_id=group.group_id,
            parent_group_id=group.parent_group_id,
            parent_relay_id=group.parent_relay_id,
            heading_deg=group.heading_deg,
            branch_depth=group.branch_depth,
        )

        for name in names:
            if name == group.leader_name:
                continue

            assignments[name] = self._follower_assignment(
                leader_id=group.leader_name,
                group_id=group.group_id,
                parent_group_id=group.parent_group_id,
                parent_relay_id=group.parent_relay_id,
                heading_deg=group.heading_deg,
                branch_depth=group.branch_depth,
            )

    def _leader_assignment(
        self,
        leader_id: str,
        group_id: str,
        parent_group_id: str,
        parent_relay_id: str,
        heading_deg: float,
        branch_depth: int,
    ) -> dict:
        return {
            'role': 'group_leader',
            'leader_id': leader_id,
            'group_id': group_id,
            'parent_group_id': parent_group_id,
            'parent_relay_id': parent_relay_id,
            'assigned_heading_deg': heading_deg,
            'branch_depth': branch_depth,
            'active': True,
        }

    def _follower_assignment(
        self,
        leader_id: str,
        group_id: str,
        parent_group_id: str,
        parent_relay_id: str,
        heading_deg: float,
        branch_depth: int,
    ) -> dict:
        return {
            'role': 'group_follower',
            'leader_id': leader_id,
            'group_id': group_id,
            'parent_group_id': parent_group_id,
            'parent_relay_id': parent_relay_id,
            'assigned_heading_deg': heading_deg,
            'branch_depth': branch_depth,
            'active': True,
        }

    # ------------------------------------------------------------------
    # Geometry helpers
    # ------------------------------------------------------------------

    def _have_required_states(self) -> bool:
        return all(name in self.states for name in self.robot_names)

    def _choose_rearmost_robot(
        self,
        names: List[str],
        leader_name: str,
        heading_deg: float,
    ) -> Optional[str]:
        ordered = self._sort_front_to_back(names, leader_name, heading_deg)
        if not ordered:
            return None
        return ordered[-1]

    def _sort_front_to_back(
        self,
        names: List[str],
        leader_name: str,
        heading_deg: float,
    ) -> List[str]:
        leader = self.states.get(leader_name)
        if leader is None:
            return [name for name in names if name in self.states]

        heading = math.radians(heading_deg)
        fx = math.cos(heading)
        fy = math.sin(heading)
        valid = [name for name in names if name in self.states]

        def key(name: str):
            state = self.states[name]
            dx = state.x - leader.x
            dy = state.y - leader.y
            proj = dx * fx + dy * fy
            return (proj, -self.robot_number(name))

        return sorted(valid, key=key, reverse=True)

    def robot_number(self, name: str) -> int:
        match = re.search(r'\d+', name)
        return int(match.group()) if match else 999

    # ------------------------------------------------------------------
    # Publish
    # ------------------------------------------------------------------

    def _publish_assignments(self, assignments: Dict[str, dict]):
        if self.phase != self.last_phase_log:
            self.last_phase_log = self.phase
            self.get_logger().info(f'[relay_tree_manager_recursive] phase={self.phase}')

        msg = String()
        msg.data = json.dumps(assignments, sort_keys=True)

        # Publish continuously because late subscribers need current assignments.
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
