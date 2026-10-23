"""Subscribe to compressed camera frames and publish /vision/persons."""

from __future__ import annotations

import argparse
import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from leo_server.common.config import config_from_env
from leo_server.common.heartbeat import Heartbeat
from leo_server.nodes.vision.pipeline import FrameGate, PosePipeline


class VisionNode(Node):
    def __init__(self, config) -> None:
        super().__init__("vision")
        self._pipeline = PosePipeline(config.vision)
        self._gate = FrameGate()
        self._heartbeat = Heartbeat(self)
        self._frame_id = 0
        self._pub = self.create_publisher(String, "/vision/persons", 10)
        self.create_subscription(CompressedImage, "/camera/image/compressed", self._on_image, 10)
        try:
            self._pipeline.load()
            self.get_logger().info("vision model loaded")
        except Exception as exc:
            self.get_logger().error(f"vision model unavailable: {exc}")

    def _on_image(self, message: CompressedImage) -> None:
        stamp = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
        if not self._gate.accept(stamp):
            return
        self._frame_id += 1
        self._heartbeat.tick()
        result = self._pipeline.infer(bytes(message.data), self._frame_id, 0, 0)
        result["timestamp_ns"] = int(self.get_clock().now().nanoseconds)
        out = String()
        out.data = json.dumps(result)
        self._pub.publish(out)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=None)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = VisionNode(config_from_env(args.config))
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
