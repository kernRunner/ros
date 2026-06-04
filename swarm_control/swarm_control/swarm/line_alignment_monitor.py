import json
import math
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from std_msgs.msg import String
from swarm_interfaces.msg import RobotState

from swarm_control.core.math_utils import normalize_angle


class LineAlignmentMonitor(Node):
    """
    Monitors three alignment errors.

    1. path_lateral_error_m:
       Side distance from each robot to the nearest point on /swarm/leader_path.
       This checks whether robots are on the recorded leader path.

    2. neighbor_line_error_m:
       Side distance of each robot from the physical line made by its chain
       neighbors. This checks whether the visible chain shape is straight.

    3. leader_axis_error_m:
       Side distance from each follower to the line through the CURRENT leader
       pose/yaw. This checks your visual question:
         "Is robot2 directly behind robot1, or too far right/left?"

       This is different from path_lateral_error. During/after turns, a robot can
       be exactly on the old leader path but not directly behind the leader's
       current heading.

    It also reports yaw_error_rad relative to the path tangent.
    """

    def __init__(self):
        super().__init__('line_alignment_monitor')
        self._declare_parameters()
        self._read_parameters()
        self._init_state()
        self._init_ros_interfaces()
        self.get_logger().info('[line_alignment_monitor] started')

    def _declare_parameters(self):
        self.declare_parameter('path_topic', '/swarm/leader_path')
        self.declare_parameter('state_topic', '/swarm/robot_states')
        self.declare_parameter('chain_order_topic', '/swarm/chain_order')
        self.declare_parameter('output_topic', '/swarm/line_alignment')

        self.declare_parameter('check_period_sec', 1.0)

        self.declare_parameter('warn_path_lateral_error_m', 0.12)
        self.declare_parameter('bad_path_lateral_error_m', 0.22)

        self.declare_parameter('warn_neighbor_line_error_m', 0.12)
        self.declare_parameter('bad_neighbor_line_error_m', 0.22)

        self.declare_parameter('warn_leader_axis_error_m', 0.15)
        self.declare_parameter('bad_leader_axis_error_m', 0.30)

        self.declare_parameter('warn_lateral_spread_m', 0.15)
        self.declare_parameter('bad_lateral_spread_m', 0.28)

        self.declare_parameter('warn_yaw_error_rad', 0.18)
        self.declare_parameter('bad_yaw_error_rad', 0.35)

        self.declare_parameter('ignore_leader_for_path_error', True)

    def _read_parameters(self):
        self.path_topic = self.get_parameter('path_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.chain_order_topic = self.get_parameter('chain_order_topic').value
        self.output_topic = self.get_parameter('output_topic').value

        self.check_period_sec = float(self.get_parameter('check_period_sec').value)

        self.warn_path_lateral_error_m = float(
            self.get_parameter('warn_path_lateral_error_m').value
        )
        self.bad_path_lateral_error_m = float(
            self.get_parameter('bad_path_lateral_error_m').value
        )

        self.warn_neighbor_line_error_m = float(
            self.get_parameter('warn_neighbor_line_error_m').value
        )
        self.bad_neighbor_line_error_m = float(
            self.get_parameter('bad_neighbor_line_error_m').value
        )

        self.warn_leader_axis_error_m = float(
            self.get_parameter('warn_leader_axis_error_m').value
        )
        self.bad_leader_axis_error_m = float(
            self.get_parameter('bad_leader_axis_error_m').value
        )

        self.warn_lateral_spread_m = float(
            self.get_parameter('warn_lateral_spread_m').value
        )
        self.bad_lateral_spread_m = float(
            self.get_parameter('bad_lateral_spread_m').value
        )

        self.warn_yaw_error_rad = float(
            self.get_parameter('warn_yaw_error_rad').value
        )
        self.bad_yaw_error_rad = float(
            self.get_parameter('bad_yaw_error_rad').value
        )

        self.ignore_leader_for_path_error = bool(
            self.get_parameter('ignore_leader_for_path_error').value
        )

    def _init_state(self):
        self.path: Optional[Path] = None
        self._path_cache = None
        self.robot_states: Dict[str, RobotState] = {}
        self.chain_order: List[str] = []

    def _init_ros_interfaces(self):
        self.create_subscription(Path, self.path_topic, self.path_callback, 10)
        self.create_subscription(RobotState, self.state_topic, self.state_callback, 10)
        self.create_subscription(String, self.chain_order_topic, self.chain_order_callback, 10)

        self.pub = self.create_publisher(String, self.output_topic, 10)
        self.create_timer(self.check_period_sec, self.check_alignment)

    def path_callback(self, msg: Path):
        self.path = msg
        self._path_cache = None

    def state_callback(self, msg: RobotState):
        if msg.active:
            self.robot_states[msg.robot_name] = msg

    def chain_order_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            if isinstance(data, list):
                self.chain_order = [str(x) for x in data]
        except Exception:
            return

    def check_alignment(self):
        order = self._get_chain_order()
        if len(order) < 2:
            return

        path_errors, yaw_errors = self.compute_path_lateral_and_yaw_errors(order)
        neighbor_errors = self.compute_neighbor_line_errors(order)
        leader_axis_errors = self.compute_leader_axis_errors(order)

        if not path_errors and not neighbor_errors and not leader_axis_errors:
            return

        max_path_error = max([abs(v) for v in path_errors.values()], default=0.0)
        max_neighbor_error = max([abs(v) for v in neighbor_errors.values()], default=0.0)
        max_leader_axis_error = max([abs(v) for v in leader_axis_errors.values()], default=0.0)
        max_yaw_error = max([abs(v) for v in yaw_errors.values()], default=0.0)

        lateral_spread = 0.0
        if path_errors:
            lateral_spread = max(path_errors.values()) - min(path_errors.values())

        status = self.compute_status(
            max_path_error,
            max_neighbor_error,
            max_leader_axis_error,
            lateral_spread,
            max_yaw_error,
        )

        report = {
            'status': status,
            'chain_order': order,
            'max_path_lateral_error_m': round(max_path_error, 3),
            'path_lateral_errors_m': {k: round(v, 3) for k, v in path_errors.items()},
            'max_neighbor_line_error_m': round(max_neighbor_error, 3),
            'neighbor_line_errors_m': {k: round(v, 3) for k, v in neighbor_errors.items()},
            'max_leader_axis_error_m': round(max_leader_axis_error, 3),
            'leader_axis_errors_m': {k: round(v, 3) for k, v in leader_axis_errors.items()},
            'lateral_spread_m': round(lateral_spread, 3),
            'max_yaw_error_rad': round(max_yaw_error, 3),
            'yaw_errors_rad': {k: round(v, 3) for k, v in yaw_errors.items()},
        }

        out = String()
        out.data = json.dumps(report)
        self.pub.publish(out)

        path_text = self._format_errors(path_errors)
        neighbor_text = self._format_errors(neighbor_errors)
        leader_axis_text = self._format_errors(leader_axis_errors)
        yaw_text = self._format_errors(yaw_errors)

        msg = (
            f'{status} '
            f'path_max={max_path_error:.2f}m '
            f'neighbor_max={max_neighbor_error:.2f}m '
            f'leader_axis_max={max_leader_axis_error:.2f}m '
            f'spread={lateral_spread:.2f}m '
            f'yaw_max={max_yaw_error:.2f}rad '
            f'path[{path_text}] '
            f'neighbor[{neighbor_text}] '
            f'leader_axis[{leader_axis_text}] '
            f'yaw[{yaw_text}]'
        )

        if status == 'OK':
            self.get_logger().info(msg)
        elif status == 'WARN':
            self.get_logger().warn(msg)
        else:
            self.get_logger().error(msg)

    def _format_errors(self, values: Dict[str, float]) -> str:
        return ' '.join(f'{name}={value:+.2f}' for name, value in values.items())

    def compute_status(
        self,
        max_path_error,
        max_neighbor_error,
        max_leader_axis_error,
        lateral_spread,
        max_yaw_error,
    ):
        if (
            max_path_error >= self.bad_path_lateral_error_m
            or max_neighbor_error >= self.bad_neighbor_line_error_m
            or max_leader_axis_error >= self.bad_leader_axis_error_m
            or lateral_spread >= self.bad_lateral_spread_m
            or max_yaw_error >= self.bad_yaw_error_rad
        ):
            return 'BAD'

        if (
            max_path_error >= self.warn_path_lateral_error_m
            or max_neighbor_error >= self.warn_neighbor_line_error_m
            or max_leader_axis_error >= self.warn_leader_axis_error_m
            or lateral_spread >= self.warn_lateral_spread_m
            or max_yaw_error >= self.warn_yaw_error_rad
        ):
            return 'WARN'

        return 'OK'

    def _get_chain_order(self):
        if self.chain_order:
            return [name for name in self.chain_order if name in self.robot_states]
        return sorted(self.robot_states.keys())

    # ------------------------------------------------------------------
    # Path error + yaw error
    # ------------------------------------------------------------------

    def compute_path_lateral_and_yaw_errors(self, order: List[str]):
        total, cumulative = self._get_path_lengths()
        if total is None or total < 0.25:
            return {}, {}

        lateral_errors = {}
        yaw_errors = {}

        for name in order:
            robot = self.robot_states.get(name)
            if robot is None:
                continue

            if self.ignore_leader_for_path_error and robot.role == 'leader':
                continue

            s = self.closest_path_s(cumulative, robot.x, robot.y)
            lateral_error, path_yaw = self.compute_lateral_error_to_path(
                cumulative,
                total,
                s,
                robot.x,
                robot.y,
            )

            lateral_errors[name] = lateral_error
            yaw_errors[name] = normalize_angle(robot.yaw - path_yaw)

        return lateral_errors, yaw_errors

    # ------------------------------------------------------------------
    # Current leader-axis error
    # ------------------------------------------------------------------

    def compute_leader_axis_errors(self, order: List[str]):
        errors = {}

        if not order:
            return errors

        leader = self.robot_states.get(order[0])
        if leader is None:
            return errors

        # Side distance to the line through the current leader pose/yaw.
        # Positive = follower is left of leader heading.
        nx = -math.sin(leader.yaw)
        ny = math.cos(leader.yaw)

        for name in order[1:]:
            robot = self.robot_states.get(name)
            if robot is None:
                continue

            dx = robot.x - leader.x
            dy = robot.y - leader.y
            errors[name] = dx * nx + dy * ny

        return errors

    # ------------------------------------------------------------------
    # Neighbor-line error
    # ------------------------------------------------------------------

    def compute_neighbor_line_errors(self, order: List[str]) -> Dict[str, float]:
        errors = {}

        if len(order) < 3:
            return errors

        for i in range(1, len(order) - 1):
            prev_robot = self.robot_states.get(order[i - 1])
            this_robot = self.robot_states.get(order[i])
            next_robot = self.robot_states.get(order[i + 1])

            if prev_robot is None or this_robot is None or next_robot is None:
                continue

            errors[this_robot.robot_name] = self.signed_point_to_line_distance(
                ax=prev_robot.x,
                ay=prev_robot.y,
                bx=next_robot.x,
                by=next_robot.y,
                px=this_robot.x,
                py=this_robot.y,
            )

        if len(order) >= 3:
            a = self.robot_states.get(order[-3])
            b = self.robot_states.get(order[-2])
            tail = self.robot_states.get(order[-1])

            if a is not None and b is not None and tail is not None:
                errors[tail.robot_name] = self.signed_point_to_line_distance(
                    ax=a.x,
                    ay=a.y,
                    bx=b.x,
                    by=b.y,
                    px=tail.x,
                    py=tail.y,
                )

        return errors

    def signed_point_to_line_distance(self, ax, ay, bx, by, px, py) -> float:
        vx = bx - ax
        vy = by - ay
        length = math.hypot(vx, vy)

        if length < 1e-6:
            return 0.0

        return ((px - ax) * vy - (py - ay) * vx) / length

    # ------------------------------------------------------------------
    # Path helpers
    # ------------------------------------------------------------------

    def _get_path_lengths(self):
        if self.path is None or len(self.path.poses) < 2:
            return None, None

        path_id = id(self.path)
        if self._path_cache is not None and self._path_cache[0] == path_id:
            _, total, cumulative = self._path_cache
            return total, cumulative

        cumulative = [0.0]
        total = 0.0

        for i in range(1, len(self.path.poses)):
            p0 = self.path.poses[i - 1].pose.position
            p1 = self.path.poses[i].pose.position
            segment = math.hypot(p1.x - p0.x, p1.y - p0.y)
            total += segment
            cumulative.append(total)

        self._path_cache = (path_id, total, cumulative)
        return total, cumulative

    def closest_path_s(self, cumulative: List[float], x: float, y: float) -> float:
        best_dist_sq = float('inf')
        best_s = 0.0

        for i in range(1, len(self.path.poses)):
            p0 = self.path.poses[i - 1].pose.position
            p1 = self.path.poses[i].pose.position

            vx = p1.x - p0.x
            vy = p1.y - p0.y
            wx = x - p0.x
            wy = y - p0.y

            seg_len_sq = vx * vx + vy * vy

            if seg_len_sq <= 1e-9:
                t = 0.0
            else:
                t = (wx * vx + wy * vy) / seg_len_sq
                t = max(0.0, min(1.0, t))

            px = p0.x + t * vx
            py = p0.y + t * vy
            dist_sq = (x - px) ** 2 + (y - py) ** 2

            if dist_sq < best_dist_sq:
                best_dist_sq = dist_sq
                best_s = cumulative[i - 1] + t * math.sqrt(seg_len_sq)

        return best_s

    def sample_path(self, cumulative: List[float], total: float, s: float):
        s = max(0.0, min(total, s))

        if s <= 0.0:
            p = self.path.poses[0].pose.position
            return p.x, p.y

        if s >= total:
            p = self.path.poses[-1].pose.position
            return p.x, p.y

        for i in range(1, len(cumulative)):
            if cumulative[i] >= s:
                p0 = self.path.poses[i - 1].pose.position
                p1 = self.path.poses[i].pose.position
                s0 = cumulative[i - 1]
                s1 = cumulative[i]
                ratio = 0.0 if s1 <= s0 else (s - s0) / (s1 - s0)
                return (
                    p0.x + ratio * (p1.x - p0.x),
                    p0.y + ratio * (p1.y - p0.y),
                )

        p = self.path.poses[-1].pose.position
        return p.x, p.y

    def sample_path_yaw(self, cumulative: List[float], total: float, s: float) -> float:
        s1 = max(0.0, s - 0.15)
        s2 = min(total, s + 0.15)
        p1 = self.sample_path(cumulative, total, s1)
        p2 = self.sample_path(cumulative, total, s2)
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]

        if math.hypot(dx, dy) < 0.01:
            return 0.0

        return math.atan2(dy, dx)

    def compute_lateral_error_to_path(self, cumulative, total, s, x, y):
        cx, cy = self.sample_path(cumulative, total, s)
        path_yaw = self.sample_path_yaw(cumulative, total, s)

        nx = -math.sin(path_yaw)
        ny = math.cos(path_yaw)

        lateral_error = (x - cx) * nx + (y - cy) * ny
        return lateral_error, path_yaw


def main(args=None):
    rclpy.init(args=args)
    node = LineAlignmentMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
