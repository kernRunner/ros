from geometry_msgs.msg import Twist


def make_twist(linear_x: float = 0.0, angular_z: float = 0.0) -> Twist:
    msg = Twist()
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    return msg


def smooth_value(old: float, new: float, alpha: float) -> float:
    return alpha * old + (1.0 - alpha) * new