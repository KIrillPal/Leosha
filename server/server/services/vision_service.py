from __future__ import annotations

import json
import logging
import threading
import time

LOGGER = logging.getLogger(__name__)


class VisionService:
    """Thread-safe storage for /vision/persons ROS topic payloads."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._persons: list[dict] = []
        self._latest_frame_id = -1
        self._latest_timestamp_ns = 0
        self._last_receive_time = 0.0  # time.time() when last /vision/persons received
        self._ros_subscribers: list[str] = []

    def try_subscribe_ros(self, node) -> bool:
        from std_msgs.msg import String

        def on_persons(msg: String) -> None:
            payload = json.loads(msg.data)
            persons = payload["persons"]
            if not isinstance(persons, list):
                raise ValueError("vision payload field 'persons' must be list")
            frame_id = int(payload["frame_id"])
            timestamp_ns = int(payload["timestamp_ns"])
            receive_time = time.time()
            with self._lock:
                self._persons = [dict(person) for person in persons]
                self._latest_frame_id = frame_id
                self._latest_timestamp_ns = timestamp_ns
                self._last_receive_time = receive_time

        node.create_subscription(String, "/vision/persons", on_persons, 10)
        self._ros_subscribers.append("/vision/persons")
        LOGGER.info("Vision service: subscribed to /vision/persons")
        return True

    def get_persons(self) -> list[dict]:
        with self._lock:
            return [dict(person) for person in self._persons]

    def get_latest_frame_id(self) -> int:
        with self._lock:
            return int(self._latest_frame_id)

    def get_last_receive_time(self) -> float:
        """Monotonic receive time (time.time()) when last /vision/persons was received."""
        with self._lock:
            return float(self._last_receive_time)

    def get_stats(self) -> dict:
        with self._lock:
            return {
                "frame_id": int(self._latest_frame_id),
                "timestamp_ns": int(self._latest_timestamp_ns),
                "persons_count": len(self._persons),
            }
