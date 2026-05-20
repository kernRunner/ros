import math


def quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def time_msg_to_ns(t) -> int:
    return int(t.sec) * 1_000_000_000 + int(t.nanosec)


def robot_name_tiebreak(name: str) -> float:
    digits = ''.join(ch for ch in name if ch.isdigit())
    if digits:
        return -float(int(digits)) * 1e-3
    return 0.0