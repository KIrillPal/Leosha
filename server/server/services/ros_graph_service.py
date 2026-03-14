"""ROS2 node graph for UI: nodes with status, FPS, description."""

from __future__ import annotations

import logging
import subprocess
import threading
from dataclasses import dataclass, field
from time import monotonic

LOGGER = logging.getLogger(__name__)

# Registry: node name -> description and optional topic for FPS
NODE_REGISTRY: dict[str, dict] = {
    "server_bridge": {
        "description": "ROS2 bridge сервера: инициализация rclpy, управление нодами",
        "topic": None,
    },
    "slam_toolbox": {
        "description": "2D SLAM: построение карты по лидару, pose graph, loop closure",
        "topic": "/map",
    },
    "ekf_localization_node": {
        "description": "EKF: слияние одометрии колёс и IMU для odom->base_link",
        "topic": "/odometry/filtered",
    },
    "robot_state_publisher": {
        "description": "Публикация TF из URDF (robot_state_publisher)",
        "topic": "/robot_description",
    },
}


@dataclass
class NodeInfo:
    name: str
    status: str  # running, stopped, unknown
    fps: float
    description: str
    last_seen: float = 0.0


class RosGraphService:
    """Provides ROS node graph for UI with status, FPS, descriptions."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._node_fps: dict[str, float] = {}  # our tracked nodes
        self._last_graph: list[NodeInfo] = []
        self._last_fetch: float = 0.0
        self._cache_ttl = 2.0  # seconds

    def set_node_fps(self, node_name: str, fps: float) -> None:
        """Update FPS for a node we track (e.g. server_bridge)."""
        with self._lock:
            self._node_fps[node_name] = fps

    def get_graph(self) -> list[dict]:
        """Returns list of {name, status, fps, description} for UI."""
        now = monotonic()
        with self._lock:
            if now - self._last_fetch < self._cache_ttl and self._last_graph:
                return [self._node_to_dict(n) for n in self._last_graph]

        # Fetch node list via ros2 CLI
        try:
            result = subprocess.run(
                ["ros2", "node", "list"],
                capture_output=True,
                text=True,
                timeout=3,
                env=None,  # inherit env (assume ROS is sourced)
            )
            if result.returncode != 0:
                # No ROS or no nodes - return registry with status=stopped
                return self._graph_from_registry_only()
            lines = [l.strip() for l in result.stdout.strip().splitlines() if l.strip()]
        except (FileNotFoundError, subprocess.TimeoutExpired) as e:
            LOGGER.debug("ros2 node list failed: %s", e)
            return self._graph_from_registry_only()

        with self._lock:
            self._last_fetch = now
            seen = set()
            graph = []
            for name in lines:
                # ros2 node list returns /node_name
                clean = name.lstrip("/") if name.startswith("/") else name
                seen.add(clean)
                info = NODE_REGISTRY[clean] if clean in NODE_REGISTRY else {"description": "ROS2 нода", "topic": None}
                fps = self._node_fps.get(clean, 0.0)
                graph.append(NodeInfo(
                    name=clean,
                    status="running",
                    fps=fps,
                    description=str(info["description"]),
                    last_seen=now,
                ))
            # Add registry nodes not in list (stopped/not launched)
            for reg_name, reg_info in NODE_REGISTRY.items():
                if reg_name not in seen:
                    graph.append(NodeInfo(
                        name=reg_name,
                        status="stopped",
                        fps=0.0,
                        description=str(reg_info["description"]),
                    ))
            self._last_graph = graph
            return [self._node_to_dict(n) for n in graph]

    def _node_to_dict(self, n: NodeInfo) -> dict:
        return {
            "name": n.name,
            "status": n.status,
            "fps": round(n.fps, 2),
            "description": n.description,
        }

    def _graph_from_registry_only(self) -> list[dict]:
        """When ros2 not available, return registry with all stopped."""
        with self._lock:
            self._last_fetch = monotonic()
            self._last_graph = [
                NodeInfo(
                    name=name,
                    status="stopped",
                    fps=self._node_fps.get(name, 0.0),
                    description=str(info["description"]),
                )
                for name, info in NODE_REGISTRY.items()
            ]
            return [self._node_to_dict(n) for n in self._last_graph]
