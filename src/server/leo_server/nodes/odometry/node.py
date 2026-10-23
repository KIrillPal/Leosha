"""Publish /odom and odom -> base_link from the fusion tracker."""

from __future__ import annotations

import argparse
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from leo_server.common.config import config_from_env
from leo_server.common.drive import drive_type, read_drive
from leo_server.common.heartbeat import Heartbeat
from leo_server.nodes.odometry.tracker import Tracker


def _yaw_quat(yaw: float) -> tuple[float, float]:
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class OdometryNode(Node):
    def __init__(self, config) -> None:
        super().__init__("odometry")
        self._tracker = Tracker(config.odometry, config.geometry["wheelbase_m"])
        self._heartbeat = Heartbeat(self)
        self._tf = TransformBroadcaster(self)
        self._pub = self.create_publisher(Odometry, "/odom", 10)
        self._last = self.get_clock().now()
        self._last_scan = None
        self.create_subscription(drive_type(), "/robot/applied", self._on_applied, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.create_subscription(Imu, "/imu", self._on_imu, 10)
        self.create_service(Trigger, "/odometry/reset", self._on_reset)
        period = 1.0 / float(config.odometry["publish_hz"])
        self.create_timer(period, self._tick)

    def _on_applied(self, message) -> None:
        speed, steer = read_drive(message)
        self._tracker.set_applied(speed, steer)

    def _on_imu(self, message: Imu) -> None:
        self._tracker.gyro_z = float(message.angular_velocity.z)

    def _on_scan(self, message: LaserScan) -> None:
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        dt = 0.1 if self._last_scan is None else max(0.0, stamp - self._last_scan)
        self._last_scan = stamp
        self._tracker.observe_scan(message.ranges, float(message.angle_min), float(message.angle_increment), dt)

    def _on_reset(self, _request, response):
        self._tracker.reset_memory()
        self._last_scan = None
        response.success = True
        response.message = "cleared scan history; pose kept"
        return response

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._last).nanoseconds / 1e9
        self._last = now
        self._tracker.step(max(0.0, dt))
        self._publish(now)
        self._heartbeat.tick()

    def _publish(self, now) -> None:
        pose = self._tracker.fusion
        message = Odometry()
        message.header.stamp = now.to_msg()
        message.header.frame_id = "odom"
        message.child_frame_id = "base_link"
        message.pose.pose.position.x = pose.x
        message.pose.pose.position.y = pose.y
        message.pose.pose.orientation.z, message.pose.pose.orientation.w = _yaw_quat(pose.yaw)
        flat = [0.0] * 36
        flat[0] = float(pose.covariance[0, 0])
        flat[7] = float(pose.covariance[1, 1])
        flat[35] = float(pose.covariance[2, 2])
        message.pose.covariance = flat
        message.twist.twist.linear.x = self._tracker.speed_mps
        message.twist.twist.angular.z = self._tracker.steering_rad
        self._pub.publish(message)
        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = pose.x
        transform.transform.translation.y = pose.y
        transform.transform.rotation.z = message.pose.pose.orientation.z
        transform.transform.rotation.w = message.pose.pose.orientation.w
        self._tf.sendTransform(transform)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=None)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = OdometryNode(config_from_env(args.config))
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
