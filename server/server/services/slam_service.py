"""SLAM pipeline statistics and map/pose access for teleop-slam UI."""

from __future__ import annotations

import io
import logging
import math
import threading
from dataclasses import dataclass, field
from time import monotonic

from ..models import SlamStats

LOGGER = logging.getLogger(__name__)


@dataclass
class MapMeta:
    """OccupancyGrid metadata for world↔pixel conversion."""
    resolution: float = 0.05
    origin_x: float = 0.0
    origin_y: float = 0.0
    width: int = 0
    height: int = 0


@dataclass
class _SlamState:
    scan_count: int = 0
    map_updates: int = 0
    pose_updates: int = 0
    last_scan_t: float = 0.0
    last_map_t: float = 0.0
    last_pose_t: float = 0.0
    map_png: bytes | None = None
    map_meta: MapMeta = field(default_factory=MapMeta)
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
        self._tf_buffer = None

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

    def update_map(self, map_png: bytes, meta: MapMeta | None = None) -> None:
        """Called when new map data arrives (e.g. from ROS subscriber)."""
        with self._lock:
            self._state.map_png = map_png
            if meta is not None:
                self._state.map_meta = meta
            self._state.map_updates += 1
            self._state.last_map_t = monotonic()
            if self._state.status == "initializing":
                self._state.status = "mapping"

    def update_pose(self, x: float, y: float, theta: float) -> None:
        """Called when new pose arrives (e.g. from TF lookup)."""
        with self._lock:
            self._state.pose_x = x
            self._state.pose_y = y
            self._state.pose_theta = theta
            self._state.pose_updates += 1
            self._state.last_pose_t = monotonic()
            if self._state.status == "initializing":
                self._state.status = "mapping"

    def get_stats(self) -> SlamStats:
        with self._lock:
            s = self._state
            now = monotonic()
            scan_dt = now - s.last_scan_t if s.last_scan_t > 0 else 1.0
            fps = 1.0 / scan_dt if scan_dt < 2.0 and s.scan_count > 0 else 0.0
            if s.map_updates > 0 and s.last_map_t > 0:
                map_dt = now - s.last_map_t
                fps = 1.0 / map_dt if map_dt < 2.0 else fps
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

    def get_map_meta(self) -> MapMeta:
        with self._lock:
            return self._state.map_meta

    def get_pose(self) -> tuple[float, float, float]:
        with self._lock:
            return (self._state.pose_x, self._state.pose_y, self._state.pose_theta)

    def _tf_lookup_pose(self) -> None:
        """Periodic TF lookup: map → base_footprint for SLAM pose.
        Skips update when map frame is not in TF yet (slam_toolbox not ready).
        """
        if self._tf_buffer is None:
            return
        from tf2_ros import TransformException

        try:
            from rclpy.time import Time
            tf = self._tf_buffer.lookup_transform("map", "base_footprint", Time())
            t = tf.transform.translation
            r = tf.transform.rotation
            siny_cosp = 2.0 * (r.w * r.z + r.x * r.y)
            cosy_cosp = 1.0 - 2.0 * (r.y * r.y + r.z * r.z)
            theta = math.atan2(siny_cosp, cosy_cosp)
            self.update_pose(float(t.x), float(t.y), theta)
        except TransformException:
            # map frame not in TF yet, extrapolation window mismatch, etc.
            # This is a normal transient condition while SLAM/TF warms up.
            return

    def try_subscribe_ros(self, node) -> bool:
        """Subscribe to /map and set up TF listener for pose. Returns True if subscribed."""
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        from tf2_ros import Buffer, TransformListener

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)

        def on_map(msg: OccupancyGrid) -> None:
            import numpy as np
            from PIL import Image
            w, h = msg.info.width, msg.info.height
            if w <= 0 or h <= 0:
                return
            meta = MapMeta(
                resolution=float(msg.info.resolution),
                origin_x=float(msg.info.origin.position.x),
                origin_y=float(msg.info.origin.position.y),
                width=w,
                height=h,
            )
            arr = np.array(msg.data, dtype=np.int8).reshape((h, w))
            img_arr = np.zeros((h, w, 3), dtype=np.uint8)
            img_arr[arr == -1] = [128, 128, 128]
            img_arr[arr == 0] = [255, 255, 255]
            img_arr[arr == 100] = [0, 0, 0]
            img = Image.fromarray(np.flipud(img_arr), mode="RGB")
            buf = io.BytesIO()
            img.save(buf, format="PNG")
            self.update_map(buf.getvalue(), meta)

        node.create_subscription(OccupancyGrid, "/map", on_map, map_qos)
        node.create_timer(0.2, self._tf_lookup_pose)  # 5 Hz TF lookup

        self._ros_subscribers.append("map")
        self._ros_subscribers.append("tf_pose")
        LOGGER.info("SLAM service: subscribed /map (RELIABLE+TRANSIENT_LOCAL), TF listener for pose")
        return True
