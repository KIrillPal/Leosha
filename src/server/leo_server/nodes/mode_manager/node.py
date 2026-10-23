"""Owns the mode parameter, the behaviour tick, and the slam_toolbox process."""

from __future__ import annotations

import argparse
import json
import math

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger

from leo_server.common.config import config_from_env
from leo_server.common.drive import drive_type, fill_drive
from leo_server.common.heartbeat import Heartbeat
from leo_server.nodes.mode_manager.modes import MODES, Actuation, Keys, ModeController, ScanView, Sense
from leo_server.nodes.mode_manager.slam_process import SlamProcess


class ModeManager(Node):
    def __init__(self, config) -> None:
        super().__init__("mode_manager")
        self._config = config
        self._controller = ModeController(
            {"client": config.client, "modes": config.modes},
            config.friends_path(),
        )
        self._slam = SlamProcess(config.slam_toolbox, config.root / "logs")
        self._heartbeat = Heartbeat(self)
        self._keys = Keys()
        self._persons: list = []
        self._frame_id = 0
        self._frame_w = int(config.modes["tracking"]["frame_width_fallback"])
        self._frame_h = int(config.modes["tracking"]["frame_height_fallback"])
        self._vision_stamp = 0.0
        self._scan: ScanView | None = None
        self._yaw_rate = 0.0
        self._action = None
        self._last_tick = self.get_clock().now()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self._mode_pub = self.create_publisher(String, "/leo/mode", latched)
        self._ui_pub = self.create_publisher(String, "/leo/ui", 10)
        self._drive_pub = self.create_publisher(drive_type(), "/cmd/drive", 10)
        self._head_pub = self.create_publisher(JointState, "/cmd/head", 10)
        self._reset = self.create_client(Trigger, "/odometry/reset")
        self.create_subscription(String, "/teleop/input", self._on_input, 10)
        self.create_subscription(String, "/vision/persons", self._on_persons, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.declare_parameter("mode", "pause")
        self.add_on_set_parameters_callback(self._on_param)
        period = 1.0 / float(config.modes["tick_hz"])
        self.create_timer(period, self._tick)
        self._publish_mode()

    def _on_param(self, params) -> SetParametersResult:
        for param in params:
            if param.name != "mode":
                continue
            try:
                self._switch(str(param.value))
            except ValueError as exc:
                return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def _switch(self, mode: str) -> None:
        self._keys = Keys()
        self._action = None
        was_slam = self._controller.uses_slam()
        self._controller.set_mode(mode)
        if self._controller.consume_slam_restart():
            self._slam.restart()
            if self._reset.service_is_ready():
                self._reset.call_async(Trigger.Request())
        elif was_slam and not self._controller.uses_slam():
            self._slam.stop()
        self._publish_mode()

    def _publish_mode(self) -> None:
        message = String()
        message.data = self._controller.mode
        self._mode_pub.publish(message)

    def _on_input(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        self._keys = Keys(
            w=bool(payload.get("w")),
            a=bool(payload.get("a")),
            s=bool(payload.get("s")),
            d=bool(payload.get("d")),
            shift=bool(payload.get("shift")),
            tracking=bool(payload.get("tracking")),
            head_dx=float(payload.get("head_dx") or 0.0),
            head_dy=float(payload.get("head_dy") or 0.0),
        )
        if payload.get("action"):
            self._action = payload["action"]

    def _on_persons(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        self._persons = list(payload.get("persons") or [])
        self._frame_id = int(payload.get("frame_id") or 0)
        self._frame_w = int(payload.get("width") or self._frame_w)
        self._frame_h = int(payload.get("height") or self._frame_h)
        self._vision_stamp = float(payload.get("timestamp_ns") or 0.0)

    def _on_scan(self, message: LaserScan) -> None:
        self._scan = ScanView(float(message.angle_min), float(message.angle_increment), list(message.ranges))

    def _on_joints(self, message: JointState) -> None:
        names = list(message.name)
        if "head_pan" in names and "head_tilt" in names:
            pan = math.degrees(float(message.position[names.index("head_pan")]))
            tilt = math.degrees(float(message.position[names.index("head_tilt")]))
            self._controller.set_measured_joints(pan, tilt)

    def _tick(self) -> None:
        now_msg = self.get_clock().now()
        dt = (now_msg - self._last_tick).nanoseconds / 1e9
        self._last_tick = now_msg
        latency = 0.0
        if self._vision_stamp:
            latency = max(0.0, (now_msg.nanoseconds - self._vision_stamp) / 1e6)
        command = self._controller.tick(
            Sense(
                dt=max(0.0, dt),
                now=now_msg.nanoseconds / 1e9,
                keys=self._keys,
                persons=self._persons,
                frame_id=self._frame_id,
                frame_w=self._frame_w,
                frame_h=self._frame_h,
                vision_latency_ms=latency,
                scan=self._scan,
                yaw_rate=self._yaw_rate,
                action=self._action,
            )
        )
        self._keys.head_dx = 0.0
        self._keys.head_dy = 0.0
        self._action = None
        self._publish(command)
        self._heartbeat.tick()

    def _publish(self, command: Actuation) -> None:
        drive = drive_type()()
        drive.header.stamp = self.get_clock().now().to_msg()
        drive.header.frame_id = "base_link"
        fill_drive(drive, command.speed_mps, command.steering_rad)
        self._drive_pub.publish(drive)
        head = JointState()
        head.header.stamp = drive.header.stamp
        head.name = ["head_pan", "head_tilt"]
        head.position = [math.radians(command.head_pan_deg), math.radians(command.head_tilt_deg)]
        self._head_pub.publish(head)
        ui = String()
        ui.data = json.dumps(self._controller.ui_state)
        self._ui_pub.publish(ui)

    def destroy_node(self):
        self._slam.stop()
        return super().destroy_node()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=None)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = ModeManager(config_from_env(args.config))
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


# Imported by tests that only need the mode list.
__all__ = ["MODES", "ModeManager", "main"]
