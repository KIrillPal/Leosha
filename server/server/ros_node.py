from __future__ import annotations

import logging

LOGGER = logging.getLogger(__name__)


class Ros2ServerBridge:
    """ROS2 bridge: node, SLAM subscriptions when rclpy available."""

    def __init__(self, slam_service=None) -> None:
        self._enabled = False
        self._node = None
        self._slam_service = slam_service

    def start(self) -> None:
        try:
            import rclpy
            from rclpy.node import Node
        except Exception:
            self._enabled = False
            LOGGER.info("ROS2 bridge disabled: rclpy not available")
            return
        rclpy.init(args=None)
        self._node = Node("server_bridge")
        self._enabled = True
        if self._slam_service:
            self._slam_service.set_ros_enabled(True)
            self._slam_service.try_subscribe_ros(self._node)
        LOGGER.info("ROS2 bridge started")

    def stop(self) -> None:
        if not self._enabled:
            return
        try:
            import rclpy
        except Exception:
            return
        if self._node is not None:
            self._node.destroy_node()
        rclpy.shutdown()
        LOGGER.info("ROS2 bridge stopped")
