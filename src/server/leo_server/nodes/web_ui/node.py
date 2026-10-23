"""Operator pages. Input becomes ROS messages; the mode manager is the only mode owner."""

from __future__ import annotations

import argparse
import json
import threading

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import String

from leo_server.common.config import config_from_env
from leo_server.common.heartbeat import Heartbeat
from leo_server.nodes.mode_manager.modes import MODES
from leo_server.nodes.web_ui.app import UiState, create_app
from leo_server.nodes.web_ui.map_image import grid_png


class WebUi(Node):
    def __init__(self, config) -> None:
        super().__init__("web_ui")
        self._state = UiState(list(MODES), config.network["robot_ip"])
        self._state.monitor_port = int(config.monitor["port"])
        self._heartbeat = Heartbeat(self)
        self._input_pub = self.create_publisher(String, "/teleop/input", 10)
        self._ip_pub = self.create_publisher(String, "/leo/robot_ip", 10)
        from rcl_interfaces.srv import SetParameters

        self._set_param_type = SetParameters
        self._mode_client = self.create_client(SetParameters, "/mode_manager/set_parameters")
        self.create_subscription(String, "/leo/mode", self._on_mode, 10)
        self.create_subscription(String, "/leo/ui", self._on_ui, 10)
        self.create_subscription(String, "/leo/system_stats", self._on_stats, 10)
        self.create_subscription(CompressedImage, "/camera/image/compressed", self._on_image, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, 10)
        self.create_timer(1.0, self._heartbeat.tick)
        app = create_app(self._state, self._publish)
        thread = threading.Thread(
            target=lambda: app.run(host=config.web_ui["host"], port=int(config.web_ui["port"]), threaded=True, use_reloader=False),
            name="web",
            daemon=True,
        )
        thread.start()
        self.get_logger().info(f"web ui on {config.web_ui['host']}:{config.web_ui['port']}")

    def _publish(self, kind: str, payload) -> None:
        if kind == "mode":
            if not self._mode_client.service_is_ready():
                return
            request = self._set_param_type.Request()
            request.parameters = [Parameter("mode", value=str(payload)).to_parameter_msg()]
            self._mode_client.call_async(request)
            return
        message = String()
        if kind == "input":
            message.data = json.dumps(payload)
            self._input_pub.publish(message)
            return
        if kind == "ip":
            message.data = str(payload)
            self._ip_pub.publish(message)
            with self._state.lock:
                self._state.robot_ip = str(payload)

    def _on_mode(self, message: String) -> None:
        with self._state.lock:
            self._state.mode = message.data

    def _on_ui(self, message: String) -> None:
        with self._state.lock:
            self._state.ui = json.loads(message.data)

    def _on_stats(self, message: String) -> None:
        with self._state.lock:
            self._state.stats = json.loads(message.data)

    def _on_image(self, message: CompressedImage) -> None:
        with self._state.lock:
            self._state.jpeg = bytes(message.data)

    def _on_scan(self, message: LaserScan) -> None:
        step = max(1, len(message.ranges) // 360)
        with self._state.lock:
            self._state.scan = {
                "angle_min": float(message.angle_min),
                "angle_increment": float(message.angle_increment) * step,
                "ranges": [float(v) for v in message.ranges[::step]],
            }

    def _on_map(self, message: OccupancyGrid) -> None:
        png = grid_png(int(message.info.width), int(message.info.height), list(message.data))
        with self._state.lock:
            self._state.map_png = png


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=None)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = WebUi(config_from_env(args.config))
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
