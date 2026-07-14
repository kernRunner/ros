# Publishes this robot's swarm state and applies relay-tree role assignments.
# The node reads the robot pose, mission mode, and role assignment messages, then updates the robot's current role, group, leader, parent relay, branch heading, and active state.
# Note: Parts of this file were developed and refined with the help of an AI/LLM assistant; the final code was reviewed, adapted, and integrated into the ROS 2 swarm project by the project team.

from typing import Dict
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger

from swarm_interfaces.msg import RobotState

from swarm_control.core.math_utils import (
    quaternion_to_yaw,
    robot_name_tiebreak,
    time_msg_to_ns,
)
from swarm_control.core.scan_utils import sector_min


class SwarmMember(Node):
    def __init__(self):
        super().__init__('swarm_member')

        self._declare_parameters()
        self._read_parameters()
        self._init_state()
        self._init_ros_interfaces()

        self.get_logger().info(f'[{self.robot_name}] swarm_member started')

    def _declare_parameters(self):
        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('state_timeout_sec', 1.5)
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('lock_leader_after_startup', True)
        self.declare_parameter('startup_election_delay_sec', 2.0)
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)

        self.declare_parameter('use_ground_truth_pose', True)
        self.declare_parameter('ground_truth_pose_topic', 'ground_truth_pose')

        self.declare_parameter('use_role_assignments', True)
        self.declare_parameter('role_assignment_topic', '/swarm/role_assignments')

    def _read_parameters(self):
        self.robot_name = self.get_parameter('robot_name').value
        self.state_timeout_sec = float(self.get_parameter('state_timeout_sec').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.lock_leader_after_startup = bool(
            self.get_parameter('lock_leader_after_startup').value
        )
        self.startup_election_delay_sec = float(
            self.get_parameter('startup_election_delay_sec').value
        )
        self.spawn_x = float(self.get_parameter('spawn_x').value)
        self.spawn_y = float(self.get_parameter('spawn_y').value)

        self.use_ground_truth_pose = bool(
            self.get_parameter('use_ground_truth_pose').value
        )
        self.ground_truth_pose_topic = self.get_parameter(
            'ground_truth_pose_topic'
        ).value

        self.use_role_assignments = bool(
            self.get_parameter('use_role_assignments').value
        )
        self.role_assignment_topic = self.get_parameter(
            'role_assignment_topic'
        ).value

    def _init_state(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.linear_speed = 0.0
        self.front_clearance = 0.0

        self.have_ground_truth_pose = False

        self.role = 'follower'
        self.current_leader = ''
        self.last_logged_leader = ''

        self.group_id = ''
        self.parent_group_id = ''
        self.parent_relay_id = ''
        self.assigned_heading_deg = 0.0
        self.branch_depth = 0
        self.active = True

        self.last_states: Dict[str, RobotState] = {}

        self.initial_election_done = False
        self.reselect_requested = False
        self.start_time_ns = self.get_clock().now().nanoseconds

        self.role_assignments = {}
        self.have_assignment = False

    def _init_ros_interfaces(self):
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data,
        )

        if self.use_ground_truth_pose:
            self.create_subscription(
                PoseStamped,
                self.ground_truth_pose_topic,
                self.ground_truth_pose_callback,
                10,
            )

        if self.use_role_assignments:
            self.create_subscription(
                String,
                self.role_assignment_topic,
                self.assignment_callback,
                10,
            )

        self.state_pub = self.create_publisher(
            RobotState,
            '/swarm/robot_states',
            10,
        )

        self.create_subscription(
            RobotState,
            '/swarm/robot_states',
            self.state_callback,
            10,
        )

        self.create_service(
            Trigger,
            '/swarm/reselect_leader',
            self.handle_reselect_leader,
        )

        period = 1.0 / self.publish_rate_hz
        self.create_timer(period, self.publish_state)

    def odom_callback(self, msg: Odometry):
        self.linear_speed = msg.twist.twist.linear.x

        if self.use_ground_truth_pose and self.have_ground_truth_pose:
            return

        self.x = self.spawn_x + msg.pose.pose.position.x
        self.y = self.spawn_y + msg.pose.pose.position.y
        self.yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def ground_truth_pose_callback(self, msg: PoseStamped):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.yaw = quaternion_to_yaw(msg.pose.orientation)

        if not self.have_ground_truth_pose:
            self.have_ground_truth_pose = True

    def scan_callback(self, msg: LaserScan):
        if not msg.ranges:
            return

        self.front_clearance = sector_min(
            scan=msg,
            center_angle=0.0,
            half_width=0.35,
            default=0.0,
        )

    def assignment_callback(self, msg: String):
        # Reads relay-tree assignments from the manager.
        try:
            assignments = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warn(
                f'[{self.robot_name}] failed to parse role assignment: {exc}'
            )
            return

        if not isinstance(assignments, dict):
            return

        self.role_assignments = assignments

        assignment = assignments.get(self.robot_name)
        if assignment is None:
            self.have_assignment = False
            return

        self.have_assignment = True
        self.apply_assignment(assignment)

    def apply_assignment(self, assignment: dict):
        # Applies this robot's role, leader, and relay-tree group data.
        old_role = self.role
        old_leader = self.current_leader
        old_group = self.group_id

        self.role = str(assignment.get('role', self.role))
        self.current_leader = str(assignment.get('leader_id', self.current_leader))
        self.group_id = str(assignment.get('group_id', self.group_id))
        self.parent_group_id = str(
            assignment.get('parent_group_id', self.parent_group_id)
        )
        self.parent_relay_id = str(
            assignment.get('parent_relay_id', self.parent_relay_id)
        )
        self.assigned_heading_deg = float(
            assignment.get('assigned_heading_deg', self.assigned_heading_deg)
        )
        self.branch_depth = int(assignment.get('branch_depth', self.branch_depth))
        self.active = bool(assignment.get('active', self.active))

        if (
            self.role != old_role
            or self.current_leader != old_leader
            or self.group_id != old_group
        ):
            self.get_logger().info(
                f'[{self.robot_name}] assignment: '
                f'role={self.role} leader={self.current_leader} '
                f'group={self.group_id} heading={self.assigned_heading_deg:.1f}'
            )

    def state_callback(self, msg: RobotState):
        self.last_states[msg.robot_name] = msg

    def handle_reselect_leader(self, request, response):
        self.reselect_requested = True
        response.success = True
        response.message = 'Leader reselection requested.'
        self.get_logger().info(f'[{self.robot_name}] received reselection request')
        return response

    def publish_state(self):
        # Publishes this robot's current RobotState.
        now = self.get_clock().now()

        if self.use_role_assignments and self.have_assignment:
            pass
        else:
            self.update_leader_if_needed(now)
            self.update_role_from_election()

        msg = self.build_robot_state_msg(now)
        self.last_states[self.robot_name] = msg
        self.state_pub.publish(msg)

    def update_leader_if_needed(self, now):
        # Runs leader election only when assignments are not available.
        if not self.initial_election_done:
            if self.should_run_election(now):
                candidates = self.get_active_candidates(now)

                if len(candidates) < 4:
                    self.current_leader = ''
                    return

                self.current_leader = self.elect_leader(now)
                self.initial_election_done = True
                self.reselect_requested = False
                self.log_leader_change('leader selected')
                return

            self.current_leader = ''
            return

        if self.reselect_requested:
            self.current_leader = self.elect_leader(now)
            self.reselect_requested = False
            self.log_leader_change('leader reselected')
            return

        if not self.lock_leader_after_startup:
            self.current_leader = self.elect_leader(now)
            self.log_leader_change('leader updated')

    def should_run_election(self, now) -> bool:
        elapsed_sec = (now.nanoseconds - self.start_time_ns) / 1e9
        startup_ready = (
            not self.initial_election_done
            and elapsed_sec >= self.startup_election_delay_sec
        )
        return startup_ready or self.reselect_requested

    def update_role_from_election(self):
        if self.current_leader and self.current_leader == self.robot_name:
            self.role = 'leader'
            self.group_id = 'group_0'
            self.assigned_heading_deg = 0.0
        else:
            self.role = 'follower'

    def build_robot_state_msg(self, now) -> RobotState:
        msg = RobotState()

        msg.robot_name = self.robot_name
        msg.stamp = now.to_msg()

        msg.x = self.x
        msg.y = self.y
        msg.yaw = self.yaw
        msg.linear_speed = self.linear_speed

        msg.leader_score = self.compute_leader_score()
        msg.role = self.role
        msg.leader_id = self.current_leader
        msg.active = self.active

        msg.group_id = self.group_id
        msg.parent_group_id = self.parent_group_id
        msg.parent_relay_id = self.parent_relay_id
        msg.assigned_heading_deg = self.assigned_heading_deg
        msg.branch_depth = self.branch_depth

        msg.is_relay = self.role in ('root_relay', 'relay')
        msg.is_group_leader = self.role in ('leader', 'group_leader')

        return msg

    def compute_leader_score(self) -> float:
        # Scores robots using front clearance, speed, and a stable tie-break.
        open_space_term = min(self.front_clearance, 3.0)
        speed_term = max(self.linear_speed, 0.0)
        tie_break = robot_name_tiebreak(self.robot_name)
        return 2.0 * open_space_term + 0.5 * speed_term + tie_break

    def elect_leader(self, now) -> str:
        # Selects the best leader from recent active robot states.
        candidates = self.get_active_candidates(now)

        if not candidates:
            return self.robot_name

        max_x = max(state.x for state in candidates)

        front_row = [
            state for state in candidates
            if abs(state.x - max_x) < 0.3
        ]

        if not front_row:
            front_row = candidates

        leader = max(
            front_row,
            key=lambda s: (
                s.leader_score,
                s.x,
                s.y,
                robot_name_tiebreak(s.robot_name),
            ),
        )

        return leader.robot_name

    def get_active_candidates(self, now):
        candidates = []

        for state in self.last_states.values():
            age = (now.nanoseconds - time_msg_to_ns(state.stamp)) / 1e9

            if age <= self.state_timeout_sec and state.active:
                candidates.append(state)

        return candidates

    def log_leader_change(self, label: str):
        if self.current_leader == self.last_logged_leader:
            return

        self.get_logger().info(
            f'[{self.robot_name}] {label}: {self.current_leader}'
        )
        self.last_logged_leader = self.current_leader


def main(args=None):
    rclpy.init(args=args)
    node = SwarmMember()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()