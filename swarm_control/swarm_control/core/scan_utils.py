# Shared helpers for reading distance values from LaserScan sectors.

import math

from sensor_msgs.msg import LaserScan
from swarm_control.core.math_utils import normalize_angle


def sector_values(
    scan: LaserScan,
    center_angle: float,
    half_width: float,
):
    # Returns valid scan ranges inside an angular sector.
    values = []
    angle = scan.angle_min

    for r in scan.ranges:
        diff = normalize_angle(angle - center_angle)

        if (
            abs(diff) <= half_width
            and math.isfinite(r)
            and scan.range_min < r < scan.range_max
        ):
            values.append(r)

        angle += scan.angle_increment

    return values


def sector_min(
    scan: LaserScan,
    center_angle: float,
    half_width: float,
    default: float = float('inf'),
):
    # Returns a stable low-distance value from a scan sector.
    values = sector_values(scan, center_angle, half_width)

    if not values:
        return default

    values.sort()
    return values[min(len(values) - 1, len(values) // 5)]


def sector_avg(
    scan: LaserScan,
    center_angle: float,
    half_width: float,
    default: float = float('inf'),
):
    # Returns the average distance inside a scan sector.
    values = sector_values(scan, center_angle, half_width)

    if not values:
        return default

    return sum(values) / len(values)