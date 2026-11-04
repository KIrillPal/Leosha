"""ZMQ session on one side, ROS topics on the other. This node does not pick a mode."""

from __future__ import annotations

import argparse
import json
import math
import threading
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Imu, JointState, LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from leo_protocol import console
from leo_protocol.duckdns import is_ipv4
from leo_protocol.protocol import CAMERA, LIDAR, Command, module_failed
from leo_server.common.config import config_from_env
from leo_server.common.drive import drive_type, fill_drive, read_drive
from leo_server.common.heartbeat import Heartbeat, LoopRate
from leo_server.nodes.robot_bridge.clock_sync import ClockSync
from leo_server.nodes.robot_bridge.duckdns import DuckDns
from leo_server.nodes.robot_bridge.link import ServerLink, reply_bytes

_LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


def _stamp(node: Node, nanoseconds: int):
    stamp = node.get_clock().now().to_msg()
    if nanoseconds < 0:
        return stamp
    stamp.sec = int(nanoseconds // 1_000_000_000)
    stamp.nanosec = int(nanoseconds % 1_000_000_000)
    return stamp


def _yaw_quat(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _pitch_quat(pitch: float) -> tuple[float, float, float, float]:
    return (0.0, math.sin(pitch / 2.0), 0.0, math.cos(pitch / 2.0))


class RobotBridge(Node):
    def __init__(self, config, server_ip: str = "") -> None:
        super().__init__("robot_bridge")
        self._config = config
        ports = config.network
        self._link = ServerLink(
            ports["bind_address"],
            ports["telemetry_port"],
            ports["handshake_port"],
            ports["command_port"],
            ports["robot_ip"],
        )
        self._duckdns = None
        self._time_sync = ClockSync()
        self._heartbeat = Heartbeat(self)
        self._cycle = LoopRate()
        self._sense = LoopRate()
        self._seq = 0
        self._speed = 0.0
        self._steer = 0.0
        self._pan_deg = 0.0
        self._tilt_deg = 0.0
        self._last_scan_id = None
        self._last_frame_id = None
        self._latency_ms = 0.0
        self._client_fps = {"loop": 0.0, "camera": 0.0, "lidar": 0.0, "telemetry": 0.0}
        self._modules: dict[str, bool] = {}
        self._last_reply_at: float | None = None
        self._scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self._image_pub = self.create_publisher(CompressedImage, "/camera/image/compressed", 10)
        self._joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self._imu_pub = self.create_publisher(Imu, "/imu", 10)
        self._applied_pub = self.create_publisher(drive_type(), "/robot/applied", 10)
        self._status_pub = self.create_publisher(String, "/robot/status", 10)
        self._truth_pub = self.create_publisher(String, "/sim/ground_truth", 10)
        self._stats_pub = self.create_publisher(String, "/leo/system_stats", 10)
        self._ip_lock = threading.Lock()
        self._ip_pub = self.create_publisher(String, "/leo/robot_ip", _LATCHED)
        self._published_ip = ""
        self._connected_ip = ""
        self._tf = TransformBroadcaster(self)
        self.create_subscription(drive_type(), "/cmd/drive", self._on_drive, 10)
        self.create_subscription(JointState, "/cmd/head", self._on_head, 10)
        self.create_subscription(String, "/leo/robot_ip_set", self._on_ip, 10)
        self._publish_robot_ip()
        self.create_timer(0.01, self._poll)
        self.create_timer(0.02, self._repeat_command)
        self.create_timer(1.0, self._publish_stats)
        self._link.start(self._on_hello)
        self._duckdns = DuckDns.start_from(config.duckdns, self.get_logger(), server_ip)
        self.get_logger().info("robot bridge listening")

    def _on_hello(self, hello, server_time_ns: int) -> bytes:
        self._adopt(getattr(hello, "local_ip", ""))
        self._publish_robot_ip()
        ip = self._link.robot_ip or self._link.last_peer
        if ip:
            self._connected_ip = ip
            console.write(console.robot_connected_line(ip))
        sim = self._config.sim_payload(hello.world or "room") if hello.sim else None
        self._time_sync.observe(hello.client_time_ns, self.get_clock().now().nanoseconds)
        return reply_bytes(server_time_ns, hello, self._config.client_payload(), sim, self._config.network)

    def _adopt(self, declared: str) -> None:
        ip = (declared or "").strip()
        if not is_ipv4(ip):
            ip = self._link.last_peer
        self._link.set_robot_ip(ip)

    def _on_drive(self, message) -> None:
        self._speed, self._steer = read_drive(message)
        self._send_command()

    def _on_head(self, message) -> None:
        if len(message.position) >= 2:
            self._pan_deg = math.degrees(float(message.position[0]))
            self._tilt_deg = math.degrees(float(message.position[1]))
        self._send_command()

    def _on_ip(self, message: String) -> None:
        self._link.set_robot_ip(message.data.strip())
        self._publish_robot_ip()

    def _send_command(self) -> None:
        self._seq += 1
        command = Command(
            seq=self._seq,
            server_time_ns=self.get_clock().now().nanoseconds,
            speed_mps=self._speed,
            steering_rad=self._steer,
            head_pan_deg=self._pan_deg,
            head_tilt_deg=self._tilt_deg,
        )
        try:
            self._link.send_command(command)
            self._cycle.tick()
        except Exception:
            return

    def _repeat_command(self) -> None:
        self._send_command()
        self._publish_robot_ip()
        self._heartbeat.tick()

    def _poll(self) -> None:
        packet = self._link.poll()
        self._publish_robot_ip()
        if packet is None:
            return
        self._last_reply_at = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds
        self._time_sync.observe(packet.client_time_ns, ros_now)
        self._sense.tick()
        self._client_fps = {
            "loop": packet.fps_loop,
            "camera": packet.fps_camera,
            "lidar": packet.fps_lidar,
            "telemetry": packet.fps_telemetry,
        }
        self._modules = dict(packet.modules)
        self._publish_applied(packet)
        self._publish_joints(packet)
        self._publish_scan(packet, ros_now)
        self._publish_image(packet)
        if packet.gyro_z is not None:
            self._publish_imu(packet)
        if packet.ground_truth is not None:
            message = String()
            message.data = json.dumps({"x": packet.ground_truth[0], "y": packet.ground_truth[1], "yaw": packet.ground_truth[2]})
            self._truth_pub.publish(message)

    def _publish_applied(self, packet) -> None:
        message = drive_type()()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        fill_drive(message, packet.speed_mps, packet.steering_rad)
        self._applied_pub.publish(message)

    def _publish_joints(self, packet) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = ["head_pan", "head_tilt"]
        message.position = [math.radians(packet.head_pan_deg), math.radians(packet.head_tilt_deg)]
        self._joint_pub.publish(message)
        self._publish_head_tf(packet)

    def _publish_head_tf(self, packet) -> None:
        now = self.get_clock().now().to_msg()
        yaw = TransformStamped()
        yaw.header.stamp = now
        yaw.header.frame_id = "head_base"
        yaw.child_frame_id = "head_yaw"
        yaw.transform.rotation.z, yaw.transform.rotation.w = _yaw_quat(math.radians(packet.head_pan_deg))[2:]
        pitch = TransformStamped()
        pitch.header.stamp = now
        pitch.header.frame_id = "head_yaw"
        pitch.child_frame_id = "camera"
        quat = _pitch_quat(math.radians(packet.head_tilt_deg))
        pitch.transform.rotation.y = quat[1]
        pitch.transform.rotation.w = quat[3]
        self._tf.sendTransform([yaw, pitch])

    def _publish_scan(self, packet, ros_now: int) -> None:
        scan = packet.scan
        repeat = scan is None or scan.capture_time_ns == self._last_scan_id
        if module_failed(self._modules, LIDAR) or repeat:
            return
        self._last_scan_id = scan.capture_time_ns
        stamp_ns = self._time_sync.to_ros_ns(scan.capture_time_ns)
        self._latency_ms = max(0.0, (ros_now - stamp_ns) / 1_000_000.0)
        message = LaserScan()
        message.header.stamp = _stamp(self, stamp_ns)
        message.header.frame_id = "laser"
        message.angle_min = float(scan.angle_min)
        message.angle_increment = float(scan.angle_increment)
        message.angle_max = float(scan.angle_min + scan.angle_increment * (len(scan.ranges) - 1))
        message.range_min = float(scan.range_min)
        message.range_max = float(scan.range_max)
        message.scan_time = float(scan.scan_time_sec)
        message.time_increment = float(scan.scan_time_sec) / max(1, len(scan.ranges))
        message.ranges = [float(value) for value in scan.ranges]
        self._scan_pub.publish(message)

    def _publish_image(self, packet) -> None:
        if module_failed(self._modules, CAMERA) or not packet.jpeg or packet.frame_time_ns == self._last_frame_id:
            return
        self._last_frame_id = packet.frame_time_ns
        message = CompressedImage()
        message.header.stamp = _stamp(self, self._time_sync.to_ros_ns(packet.frame_time_ns))
        message.header.frame_id = "camera"
        message.format = "jpeg"
        message.data = packet.jpeg
        self._image_pub.publish(message)

    def _publish_imu(self, packet) -> None:
        message = Imu()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.angular_velocity.z = float(packet.gyro_z)
        self._imu_pub.publish(message)

    def _publish_stats(self) -> None:
        message = String()
        message.data = json.dumps(
            {
                "system_fps": round(self._sense.value, 2),
                "command_fps": round(self._cycle.value, 2),
                "latency_ms": round(self._latency_ms, 1),
                "client": self._client_fps,
            }
        )
        self._stats_pub.publish(message)
        status = String()
        age = None if self._last_reply_at is None else time.monotonic() - self._last_reply_at
        status.data = json.dumps(
            {
                "robot_ip": self._link.robot_ip,
                "connected": self._last_scan_id is not None or self._last_frame_id is not None,
                "reply_age_sec": None if age is None else round(age, 3),
                "modules": self._modules,
            }
        )
        self._status_pub.publish(status)
        self._publish_robot_ip()

    def _publish_robot_ip(self) -> None:
        ip = self._link.robot_ip
        with self._ip_lock:
            if not ip or ip == self._published_ip:
                return
            self._published_ip = ip
        message = String()
        message.data = ip
        self._ip_pub.publish(message)
        self.get_logger().info(f"robot ip {ip}")

    def destroy_node(self):
        if self._connected_ip:
            console.write(console.robot_disconnected_line(self._connected_ip))
            self._connected_ip = ""
        if self._duckdns is not None:
            self._duckdns.close()
        self._link.close()
        return super().destroy_node()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=None)
    parser.add_argument("--server", default="")
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = RobotBridge(config_from_env(args.config), args.server.strip())
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
