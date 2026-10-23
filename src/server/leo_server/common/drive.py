"""Drive command message.

``ackermann_msgs`` is the preferred type. When it is not installed the same
numbers travel as ``geometry_msgs/TwistStamped``: linear.x is m/s and
angular.z is steering radians. Both publishers and subscribers call this
module, so the choice stays in one place.
"""

from __future__ import annotations


def drive_type():
    try:
        from ackermann_msgs.msg import AckermannDriveStamped

        return AckermannDriveStamped
    except ImportError:
        from geometry_msgs.msg import TwistStamped

        return TwistStamped


def fill_drive(message, speed_mps: float, steering_rad: float) -> None:
    if hasattr(message, "drive"):
        message.drive.speed = float(speed_mps)
        message.drive.steering_angle = float(steering_rad)
        return
    message.twist.linear.x = float(speed_mps)
    message.twist.angular.z = float(steering_rad)


def read_drive(message) -> tuple[float, float]:
    if hasattr(message, "drive"):
        return float(message.drive.speed), float(message.drive.steering_angle)
    return float(message.twist.linear.x), float(message.twist.angular.z)
