"""Names and freshness for modules that can be checked from their own signal."""

from __future__ import annotations

import time
from datetime import datetime

from leo_protocol.console import module_line
from leo_protocol.protocol import CAMERA, LIDAR

MOTOR = "motor"
WHEEL_SERVO = "wheel servo"
NECK_SERVO = "neck servo"
FACE_SERVO = "face servo"
ACTUATORS = (MOTOR, WHEEL_SERVO, NECK_SERVO, FACE_SERVO)

# A sample older than this many device periods is not a live signal.
STALE_PERIODS = 3


def fresh(stamp_ns: int, period_sec: float, now_ns: int | None = None) -> bool:
    """True when ``stamp_ns`` is within ``STALE_PERIODS`` of ``now_ns``."""
    if period_sec <= 0 or stamp_ns <= 0:
        return False
    now = time.monotonic_ns() if now_ns is None else now_ns
    limit_ns = int(STALE_PERIODS * period_sec * 1_000_000_000)
    return now - int(stamp_ns) <= limit_ns


def status_line(name: str, ok: bool, *, now: datetime | None = None) -> str:
    """Return one console line for a module that is up or down."""
    return module_line(name, ok, now=now)


def format_modules(states: dict[str, bool], *, now: datetime | None = None) -> str:
    """Return every module line, sharing one timestamp."""
    moment = datetime.now() if now is None else now
    return "\n".join(status_line(name, ok, now=moment) for name, ok in states.items())
