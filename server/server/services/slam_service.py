"""SLAM pipeline statistics and map/pose access for teleop-slam UI."""

from __future__ import annotations

import io
import logging
import threading
from dataclasses import dataclass, field
from time import monotonic

from ..models import SlamStats

LOGGER = logging.getLogger(__name__)


@dataclass
class _SlamState:
    scan_count: int = 0
    map_updates: int = 0
    pose_updates: int = 0
    last_scan_t: float = 0.0
    last_map_t: float = 0.0
    last_pose_t: float = 0.0
    map_png: bytes | None = None
    pose_x: float = 0.0
    pose_y: float = 0.0
    pose_theta: float = 0.0
    status: str = "initializing"


class SlamService:
    """Collects SLAM stats from telemetry and optional ROS topics."""

    def __init__(self, robot_client) -> None:
        self._robot = robot_client
        self._lock = threading.Lock()
        self._state = _SlamState()
        self._ros_enabled = False
        self._ros_subscribers = []

    def set_ros_enabled(self, enabled: bool) -> None:
        self._ros_enabled = enabled

    def update_from_telemetry(self) -> None:
        """Call from controller tick or telemetry handler to update scan-derived stats."""
        scan = self._robot.get_latest_lidar_scan()
        with self._lock:
            if scan:
                self._state.scan_count += 1
                self._state.last_scan_t = monotonic()
                self._state.status = "mapping" if self._state.status == "initializing" else self._state.status

    def update_map(self, map_png: bytes) -> None:
        """Called when new map data arrives (e.g. from ROS subscriber)."""
        with self._lock:
            self._state.map_png = map_png
            self._state.map_updates += 1
            self._state.last_map_t = monotonic()

    def update_pose(self, x: float, y: float, theta: float) -> None:
        """Called when new pose arrives (e.g. from ROS subscriber)."""
        with self._lock:
            self._state.pose_x = x
            self._state.pose_y = y
            self._state.pose_theta = theta
            self._state.pose_updates += 1
            self._state.last_pose_t = monotonic()

    def get_stats(self) -> SlamStats:
        with self._lock:
            s = self._state
            now = monotonic()
            # FPS: scans per second over last second
            scan_dt = now - s.last_scan_t if s.last_scan_t > 0 else 1.0
            fps = 1.0 / scan_dt if scan_dt < 2.0 and s.scan_count > 0 else 0.0
            # Use map update rate if we have it
            if s.map_updates > 0 and s.last_map_t > 0:
                map_dt = now - s.last_map_t
                fps = 1.0 / map_dt if map_dt < 2.0 else fps
            # Latency: time since last update
            latency_ms = (now - max(s.last_scan_t, s.last_map_t, s.last_pose_t)) * 1000.0
            return SlamStats(
                fps=round(fps, 2),
                latency_ms=round(latency_ms, 2),
                scan_count=s.scan_count,
                map_updates=s.map_updates,
                pose_updates=s.pose_updates,
                status=s.status,
            )

    def get_map_png(self) -> bytes | None:
        with self._lock:
            return self._state.map_png

    def get_pose(self) -> tuple[float, float, float]:
        with self._lock:
            return (self._state.pose_x, self._state.pose_y, self._state.pose_theta)

    def try_subscribe_ros(self, node) -> bool:
        """Subscribe to /map and pose when rclpy node is available. Returns True if subscribed."""
        try:
            from geometry_msgs.msg import PoseWithCovarianceStamped
            from nav_msgs.msg import OccupancyGrid
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )

            def on_map(msg: OccupancyGrid) -> None:
                try:
                    import numpy as np
                    from PIL import Image
                    w, h = msg.info.width, msg.info.height
                    if w <= 0 or h <= 0:
                        return
                    arr = np.array(msg.data, dtype=np.int8).reshape((h, w))
                    # -1=unknown, 0=free, 100=occupied
                    img_arr = np.zeros((h, w, 3), dtype=np.uint8)
                    img_arr[arr == -1] = [128, 128, 128]
                    img_arr[arr == 0] = [255, 255, 255]
                    img_arr[arr == 100] = [0, 0, 0]
                    img = Image.fromarray(img_arr, mode="RGB")
                    buf = io.BytesIO()
                    img.save(buf, format="PNG")
                    self.update_map(buf.getvalue())
                except Exception as e:
                    LOGGER.debug("Map conversion error: %s", e)

            def on_pose(msg: PoseWithCovarianceStamped) -> None:
                p = msg.pose.pose.position
                o = msg.pose.pose.orientation
                import math
                siny_cosp = 2 * (o.w * o.z + o.x * o.y)
                cosy_cosp = 1 - 2 * (o.y * o.y + o.z * o.z)
                theta = math.atan2(siny_cosp, cosy_cosp)
                self.update_pose(float(p.x), float(p.y), theta)

            node.create_subscription(OccupancyGrid, "/map", on_map, qos)
            node.create_subscription(
                PoseWithCovarianceStamped,
                "/slam_toolbox/pose",
                on_pose,
                qos,
            )
            self._ros_subscribers.append("map")
            self._ros_subscribers.append("pose")
            LOGGER.info("SLAM service subscribed to ROS /map and /slam_toolbox/pose")
            return True
        except Exception as e:
            LOGGER.debug("SLAM ROS subscribe failed (slam_toolbox may not be running): %s", e)
            return False
