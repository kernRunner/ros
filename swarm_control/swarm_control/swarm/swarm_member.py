from typing import Dict

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
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

    def _init_state(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.linear_speed = 0.0
        self.front_clearance = 0.0

        self.role = 'follower'
        self.current_leader = ''
        self.last_logged_leader = ''

        self.last_states: Dict[str, RobotState] = {}

        self.initial_election_done = False
        self.reselect_requested = False
        self.start_time_ns = self.get_clock().now().nanoseconds

    def _init_ros_interfaces(self):
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

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

    # -------------------------
    # Callbacks
    # -------------------------

    def odom_callback(self, msg: Odometry):
        self.x = self.spawn_x + msg.pose.pose.position.x
        self.y = self.spawn_y + msg.pose.pose.position.y
        self.linear_speed = msg.twist.twist.linear.x
        self.yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def scan_callback(self, msg: LaserScan):
        if not msg.ranges:
            return

        self.front_clearance = sector_min(
            scan=msg,
            center_angle=0.0,
            half_width=0.35,
            default=0.0,
        )

    def state_callback(self, msg: RobotState):
        self.last_states[msg.robot_name] = msg

    def handle_reselect_leader(self, request, response):
        self.reselect_requested = True

        response.success = True
        response.message = 'Leader reselection requested.'

        self.get_logger().info(f'[{self.robot_name}] received reselection request')
        return response

    # -------------------------
    # Main behavior
    # -------------------------

    def publish_state(self):
        now = self.get_clock().now()

        self._update_leader_if_needed(now)
        self._update_role()

        msg = self._build_robot_state_msg(now)
        self.last_states[self.robot_name] = msg
        self.state_pub.publish(msg)

    def _update_leader_if_needed(self, now):
        if self._should_run_election(now):
            self.current_leader = self._elect_leader(now)
            self.initial_election_done = True
            self.reselect_requested = False
            self._log_leader_change('leader selected')
            return

        if not self.current_leader:
            self.current_leader = self.robot_name

        if not self.lock_leader_after_startup and self.initial_election_done:
            self.current_leader = self._elect_leader(now)
            self._log_leader_change('leader updated')

    def _should_run_election(self, now) -> bool:
        elapsed_sec = (now.nanoseconds - self.start_time_ns) / 1e9

        startup_ready = (
            not self.initial_election_done
            and elapsed_sec >= self.startup_election_delay_sec
        )

        return startup_ready or self.reselect_requested

    def _update_role(self):
        if self.current_leader == self.robot_name:
            self.role = 'leader'
        else:
            self.role = 'follower'

    def _build_robot_state_msg(self, now) -> RobotState:
        msg = RobotState()

        msg.robot_name = self.robot_name
        msg.stamp = now.to_msg()

        msg.x = self.x
        msg.y = self.y
        msg.yaw = self.yaw
        msg.linear_speed = self.linear_speed

        msg.leader_score = self._compute_leader_score()
        msg.role = self.role
        msg.leader_id = self.current_leader
        msg.active = True

        return msg

    # -------------------------
    # Leader election
    # -------------------------

    def _compute_leader_score(self) -> float:
        open_space_term = min(self.front_clearance, 3.0)
        speed_term = max(self.linear_speed, 0.0)
        tie_break = robot_name_tiebreak(self.robot_name)

        return 2.0 * open_space_term + 0.5 * speed_term + tie_break

    def _elect_leader(self, now) -> str:
        candidates = self._get_active_candidates(now)

        if not candidates:
            return self.robot_name

        # -------------------------
        # 1. Find "front row"
        # (assuming robots face +X direction)
        # -------------------------
        max_x = max(state.x for state in candidates)

        # robots close to max_x are "front row"
        front_row = [
            state for state in candidates
            if abs(state.x - max_x) < 0.3   # tolerance
        ]

        if not front_row:
            front_row = candidates

        # -------------------------
        # 2. Decide LEFT vs RIGHT
        # -------------------------
        # Option A (simple): always pick leftmost
        # leader = min(front_row, key=lambda s: s.y)

        # Option B (better): pick side with more open space
        leader = max(
            front_row,
            key=lambda s: (
                s.leader_score,
                s.x,
                s.y,
                robot_name_tiebreak(s.robot_name),
            )
        )

        return leader.robot_name

    def _get_active_candidates(self, now):
        candidates = []

        for state in self.last_states.values():
            age = (now.nanoseconds - time_msg_to_ns(state.stamp)) / 1e9

            if age <= self.state_timeout_sec and state.active:
                candidates.append(state)

        return candidates

    def _log_leader_change(self, label: str):
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