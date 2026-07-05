#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

from tf2_ros import Buffer, TransformListener, TransformException


class SwarmLidarMapper(Node):
    def __init__(self):
        super().__init__('swarm_lidar_mapper')

        self.robot_count = 9
        self.map_topic = '/swarm/map'

        self.resolution = 0.25
        self.width = 800
        self.height = 800
        self.origin_x = -20.0
        self.origin_y = -100.0

        # Log-odds-like confidence map:
        # unknown = 0
        # free evidence decreases value
        # occupied evidence increases value
        self.evidence = [0] * (self.width * self.height)
        self.data = [-1] * (self.width * self.height)

        self.free_decrement = 2
        self.occupied_increment = 6
        self.min_evidence = -20
        self.max_evidence = 40

        # Thresholds for publishing OccupancyGrid
        self.free_threshold = -2
        self.occupied_threshold = 8

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_pub = self.create_publisher(OccupancyGrid, self.map_topic, 1)

        for i in range(1, self.robot_count + 1):
            topic = f'/robot{i}/scan'
            self.create_subscription(
                LaserScan,
                topic,
                self.scan_callback,
                qos_profile_sensor_data,
            )
            self.get_logger().info(f'Subscribed to {topic}')

        self.create_timer(0.5, self.publish_map)
        self.get_logger().info('Swarm lidar mapper started, publishing /swarm/map')

    def world_to_cell(self, x, y):
        mx = int((x - self.origin_x) / self.resolution)
        my = int((y - self.origin_y) / self.resolution)

        if mx < 0 or mx >= self.width or my < 0 or my >= self.height:
            return None

        return mx, my

    def cell_index(self, mx, my):
        return my * self.width + mx

    def add_evidence(self, mx, my, delta):
        if mx < 0 or mx >= self.width or my < 0 or my >= self.height:
            return

        idx = self.cell_index(mx, my)
        value = self.evidence[idx] + delta

        if value < self.min_evidence:
            value = self.min_evidence
        elif value > self.max_evidence:
            value = self.max_evidence

        self.evidence[idx] = value

    def mark_free(self, mx, my):
        self.add_evidence(mx, my, -self.free_decrement)

    def mark_occupied(self, mx, my):
        self.add_evidence(mx, my, self.occupied_increment)

    def mark_ray(self, x0, y0, x1, y1, occupied):
        start = self.world_to_cell(x0, y0)
        end = self.world_to_cell(x1, y1)

        if start is None or end is None:
            return

        x0c, y0c = start
        x1c, y1c = end

        dx = abs(x1c - x0c)
        dy = abs(y1c - y0c)

        sx = 1 if x0c < x1c else -1
        sy = 1 if y0c < y1c else -1

        err = dx - dy

        x = x0c
        y = y0c

        while True:
            if x == x1c and y == y1c:
                break

            # Ray passed through this cell, so it is free.
            self.mark_free(x, y)

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

        if occupied:
            # End point is a hit, so likely occupied.
            self.mark_occupied(x1c, y1c)
        else:
            # Max-range ray ended with no hit, so endpoint is also free.
            self.mark_free(x1c, y1c)

    def yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg: LaserScan):
        try:
            tf = self.tf_buffer.lookup_transform(
                'world',
                msg.header.frame_id,
                rclpy.time.Time(),
            )
        except TransformException:
            return

        x0 = tf.transform.translation.x
        y0 = tf.transform.translation.y
        yaw = self.yaw_from_quaternion(tf.transform.rotation)

        angle = msg.angle_min

        for r in msg.ranges:
            if math.isnan(r) or r < msg.range_min:
                angle += msg.angle_increment
                continue

            if math.isinf(r) or r >= msg.range_max * 0.98:
                r_use = msg.range_max
                occupied = False
            else:
                r_use = r
                occupied = True

            world_angle = yaw + angle
            x1 = x0 + r_use * math.cos(world_angle)
            y1 = y0 + r_use * math.sin(world_angle)

            self.mark_ray(x0, y0, x1, y1, occupied)

            angle += msg.angle_increment

    def rebuild_occupancy_grid(self):
        for idx, value in enumerate(self.evidence):
            if value >= self.occupied_threshold:
                self.data[idx] = 100
            elif value <= self.free_threshold:
                self.data[idx] = 0
            else:
                self.data[idx] = -1

    def publish_map(self):
        self.rebuild_occupancy_grid()

        msg = OccupancyGrid()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height

        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = self.data

        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SwarmLidarMapper()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()