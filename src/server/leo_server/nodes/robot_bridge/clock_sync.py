"""Map the robot's monotonic clock onto ROS time.

``ros_time - client_time`` includes the queue delay, so the smallest recent
sample is the offset. The window has to forget old samples: the wall clock
here steps forward by a few seconds every half minute, and a permanent
minimum would leave every later scan stamped in the past. slam_toolbox
would then draw the live scan at an old pose and redraw the room.
"""

from __future__ import annotations

from collections import deque

_WINDOW = 15
_AGREE = 5
_EARLY_LIMIT_NS = 1_000_000_000
_FOLLOW_NS = 500_000_000


class ClockSync:
    def __init__(self) -> None:
        self.offset_ns: int | None = None
        self._samples: deque[int] = deque(maxlen=_WINDOW)

    def observe(self, client_ns: int, ros_ns: int) -> None:
        sample = int(ros_ns) - int(client_ns)
        if self._is_early_glitch(sample):
            return
        self._samples.append(sample)
        self._drop_samples_from_before_a_clock_step()
        self.offset_ns = min(self._samples)

    def to_ros_ns(self, client_ns: int) -> int:
        if self.offset_ns is None:
            return int(client_ns)
        return int(client_ns) + self.offset_ns

    def _is_early_glitch(self, sample: int) -> bool:
        return self.offset_ns is not None and sample < self.offset_ns - _EARLY_LIMIT_NS

    def _drop_samples_from_before_a_clock_step(self) -> None:
        if self.offset_ns is None or len(self._samples) < _AGREE:
            return
        recent = list(self._samples)[-_AGREE:]
        if min(recent) <= self.offset_ns + _FOLLOW_NS:
            return
        self._samples.clear()
        self._samples.extend(recent)
