from __future__ import annotations


class Ros2ServerBridge:
    """Минимальный ROS2 bridge-слой (без обязательного rclpy в тестах)."""

    def __init__(self) -> None:
        self._enabled = False
        self._node = None

    def start(self) -> None:
        try:
            import rclpy
            from rclpy.node import Node
        except Exception:
            self._enabled = False
            return
        rclpy.init(args=None)
        self._node = Node("server_bridge")
        self._enabled = True

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
