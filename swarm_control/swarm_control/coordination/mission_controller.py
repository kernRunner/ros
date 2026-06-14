import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MissionController(Node):
    """
    Persistent swarm mission mode controller.

    Input:
      /swarm/mission_command  std_msgs/String

    Output:
      /swarm/mission_mode     std_msgs/String, published continuously

    Accepted command payloads:
      plain text:
        explore
        stop
        return_home

      JSON-like:
        {"mode":"explore"}
        {"mode":"stop"}
        {"mode":"return_home"}

    Why this node exists:
      /swarm/mission_command is an event topic. A robot can miss a one-shot
      event while launching, spinning slowly, or under CPU load. This controller
      turns that event into persistent state so all robots eventually converge
      to the same mode.
    """

    VALID_MODES = {'explore', 'stop', 'return_home'}

    def __init__(self):
        super().__init__('mission_controller')

        self.declare_parameter('command_topic', '/swarm/mission_command')
        self.declare_parameter('mode_topic', '/swarm/mission_mode')
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('default_mode', 'explore')

        self.command_topic = self.get_parameter('command_topic').value
        self.mode_topic = self.get_parameter('mode_topic').value
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.current_mode = str(self.get_parameter('default_mode').value).strip().lower()

        if self.current_mode not in self.VALID_MODES:
            self.get_logger().warn(
                f"Invalid default_mode={self.current_mode!r}; using 'explore'"
            )
            self.current_mode = 'explore'

        self.mode_pub = self.create_publisher(String, self.mode_topic, 10)
        self.create_subscription(String, self.command_topic, self.command_callback, 10)

        period = 1.0 / max(0.1, self.publish_rate_hz)
        self.create_timer(period, self.publish_mode)

        self.get_logger().info(
            f'mission_controller started: command_topic={self.command_topic}, '
            f'mode_topic={self.mode_topic}, current_mode={self.current_mode}'
        )

    def command_callback(self, msg: String):
        mode = self.parse_mode(msg.data)

        if mode is None:
            self.get_logger().warn(f'Ignoring invalid mission command: {msg.data!r}')
            return

        if mode == self.current_mode:
            self.get_logger().info(f'mission mode already {self.current_mode}')
            self.publish_mode()
            return

        old = self.current_mode
        self.current_mode = mode
        self.get_logger().warn(f'mission mode changed: {old} -> {self.current_mode}')
        self.publish_mode()

    def parse_mode(self, raw: str):
        raw = (raw or '').strip()

        if not raw:
            return None

        mode = raw.strip().lower()

        if raw.startswith('{'):
            try:
                data = json.loads(raw)
                mode = str(data.get('mode', '')).strip().lower()
            except Exception:
                lowered = raw.lower()
                if '"stop"' in lowered or "'stop'" in lowered:
                    mode = 'stop'
                elif '"return_home"' in lowered or "'return_home'" in lowered:
                    mode = 'return_home'
                elif '"explore"' in lowered or "'explore'" in lowered:
                    mode = 'explore'

        if mode not in self.VALID_MODES:
            return None

        return mode

    def publish_mode(self):
        msg = String()
        msg.data = self.current_mode
        self.mode_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()