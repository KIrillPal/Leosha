"""One-hertz JSON heartbeat so the monitor can show each node's real loop rate."""

from __future__ import annotations

import json
import time


class LoopRate:
    def __init__(self) -> None:
        self.value = 0.0
        self._count = 0
        self._start = time.monotonic()

    def tick(self) -> float:
        self._count += 1
        now = time.monotonic()
        elapsed = now - self._start
        if elapsed >= 0.5:
            self.value = self._count / elapsed
            self._count = 0
            self._start = now
        return self.value


class Heartbeat:
    def __init__(self, node) -> None:
        from std_msgs.msg import String

        self._node = node
        self._rate = LoopRate()
        self._pub = node.create_publisher(String, "/leo/heartbeat", 10)
        self._string = String
        node.create_timer(1.0, self._publish)

    def tick(self) -> None:
        self._rate.tick()

    def _publish(self) -> None:
        message = self._string()
        message.data = json.dumps({"node": self._node.get_name(), "fps": round(self._rate.value, 2), "stamp": time.time()})
        self._pub.publish(message)
