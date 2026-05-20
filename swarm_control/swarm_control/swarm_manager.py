# import math
# from enum import Enum

# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import PoseStamped
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


# class Role(str, Enum):
#     LEADER = "leader"
#     FOLLOWER = "follower"
#     BREADCRUMB = "breadcrumb"
#     RESERVE = "reserve"


# class RobotInfo:
#     def __init__(self, name: str):
#         self.name = name
#         self.x = None
#         self.y = None
#         self.role = Role.RESERVE
#         self.branch_id = 0
#         self.parent_anchor = None
#         self.has_pose = False

#     def set_pose(self, x: float, y: float):
#         self.x = x
#         self.y = y
#         self.has_pose = True

#     def distance_to(self, other_xy):
#         if not self.has_pose:
#             return float("inf")
#         ox, oy = other_xy
#         return math.hypot(self.x - ox, self.y - oy)


# class SwarmManager(Node):
#     def __init__(self):
#         super().__init__("swarm_manager")

#         self.declare_parameter("robot_names", ["robot1", "robot2", "robot3"])
#         self.declare_parameter("distance_threshold", 2.0)

#         robot_names = self.get_parameter("robot_names").value
#         self.distance_threshold = float(
#             self.get_parameter("distance_threshold").value
#         )

#         self.robots = {name: RobotInfo(name) for name in robot_names}

#         self.anchor_chain = []
#         self.last_anchor_xy = (0.0, 0.0)
#         self.phase = 0

#         self.goal_publishers = {}
#         self.odom_subs = []

#         odom_qos = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=10,
#             durability=DurabilityPolicy.VOLATILE,
#         )

#         for name in robot_names:
#             odom_topic = f"/model/{name}/odometry"
#             goal_topic = f"/{name}/goal_pose"

#             self.odom_subs.append(
#                 self.create_subscription(
#                     Odometry,
#                     odom_topic,
#                     self.make_odom_cb(name),
#                     odom_qos
#                 )
#             )

#             self.goal_publishers[name] = self.create_publisher(
#                 PoseStamped,
#                 goal_topic,
#                 10
#             )

#         self.timer = self.create_timer(1.0, self.control_loop)

#         self.initialize_roles(robot_names)

#         self.get_logger().info(
#             f"Swarm manager started with robots: {robot_names}"
#         )
#         self.get_logger().info(
#             f"Distance threshold: {self.distance_threshold:.2f} m"
#         )

#     def initialize_roles(self, robot_names):
#         if len(robot_names) >= 1:
#             self.robots[robot_names[0]].role = Role.LEADER
#         if len(robot_names) >= 2:
#             self.robots[robot_names[1]].role = Role.FOLLOWER
#         if len(robot_names) >= 3:
#             self.robots[robot_names[2]].role = Role.RESERVE

#     def make_odom_cb(self, robot_name):
#         def cb(msg: Odometry):
#             self.robots[robot_name].set_pose(
#                 msg.pose.pose.position.x,
#                 msg.pose.pose.position.y
#             )
#         return cb

#     def all_have_pose(self):
#         return all(robot.has_pose for robot in self.robots.values())

#     def get_leader(self):
#         for robot in self.robots.values():
#             if robot.role == Role.LEADER:
#                 return robot
#         return None

#     def get_active_non_breadcrumbs(self):
#         return [
#             r for r in self.robots.values()
#             if r.role in (Role.LEADER, Role.FOLLOWER, Role.RESERVE)
#         ]

#     def get_breadcrumb_candidates(self):
#         return [
#             r for r in self.robots.values()
#             if r.role in (Role.FOLLOWER, Role.RESERVE)
#         ]

#     def choose_breadcrumb_robot(self):
#         candidates = self.get_breadcrumb_candidates()
#         available = [r for r in candidates if r.has_pose]
#         if not available:
#             return None

#         reserves = [r for r in available if r.role == Role.RESERVE]
#         if reserves:
#             return reserves[0]

#         followers = [r for r in available if r.role == Role.FOLLOWER]
#         if followers:
#             return followers[0]

#         return None

#     def drop_breadcrumb(self):
#         robot = self.choose_breadcrumb_robot()
#         if robot is None:
#             self.get_logger().warn("No robot available to become breadcrumb")
#             return

#         robot.role = Role.BREADCRUMB
#         self.anchor_chain.append((robot.name, robot.x, robot.y))
#         self.last_anchor_xy = (robot.x, robot.y)

#         self.get_logger().info(
#             f"[BREADCRUMB] {robot.name} anchored at "
#             f"({robot.x:.2f}, {robot.y:.2f})"
#         )

#         self.promote_roles_after_drop()

#     def promote_roles_after_drop(self):
#         leader = self.get_leader()
#         if leader is None:
#             non_breadcrumbs = self.get_active_non_breadcrumbs()
#             if non_breadcrumbs:
#                 non_breadcrumbs[0].role = Role.LEADER
#             return

#         followers = [r for r in self.robots.values() if r.role == Role.FOLLOWER]
#         reserves = [r for r in self.robots.values() if r.role == Role.RESERVE]

#         if not followers and reserves:
#             reserves[0].role = Role.FOLLOWER
#             self.get_logger().info(
#                 f"[ROLE] {reserves[0].name} promoted to FOLLOWER"
#             )

#     def control_loop(self):
#         if not self.all_have_pose():
#             missing = [name for name, robot in self.robots.items() if not robot.has_pose]
#             self.get_logger().info(f"Waiting for robot poses... missing={missing}")
#             return

#         leader = self.get_leader()
#         if leader is None:
#             self.get_logger().warn("No leader available")
#             return

#         dist = leader.distance_to(self.last_anchor_xy)

#         self.get_logger().info(
#             f"[STATUS] leader={leader.name} role={leader.role.value} "
#             f"pos=({leader.x:.2f},{leader.y:.2f}) "
#             f"dist_from_last_anchor={dist:.2f} "
#             f"anchors={len(self.anchor_chain)}"
#         )

#         if dist > self.distance_threshold:
#             self.drop_breadcrumb()

#     def publish_goal(self, robot_name: str, x: float, y: float):
#         msg = PoseStamped()
#         msg.header.frame_id = "map"
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.pose.position.x = x
#         msg.pose.position.y = y
#         msg.pose.position.z = 0.0
#         msg.pose.orientation.w = 1.0

#         self.goal_publishers[robot_name].publish(msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = SwarmManager()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()