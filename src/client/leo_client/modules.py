"""Names and freshness for modules that can be checked from their own signal."""

from __future__ import annotations

import time

from leo_protocol.protocol import CAMERA, LIDAR

MOTOR = "motor"
WHEEL_SERVO = "wheel servo"
NECK_SERVO = "neck servo"
FACE_SERVO = "face servo"
ACTUATORS = (MOTOR, WHEEL_SERVO, NECK_SERVO, FACE_SERVO)

# A sample older than this many device periods is not a live signal.
STALE_PERIODS = 3

_GREEN = "\033[32m"
_RED = "\033[31m"
_RESET = "\033[0m"


def fresh(stamp_ns: int, period_sec: float, now_ns: int | None = None) -> bool:
    """True when ``stamp_ns`` is within ``STALE_PERIODS`` of ``now_ns``."""
    if period_sec <= 0 or stamp_ns <= 0:
        return False
    now = time.monotonic_ns() if now_ns is None else now_ns
    limit_ns = int(STALE_PERIODS * period_sec * 1_000_000_000)
    return now - int(stamp_ns) <= limit_ns


def status_line(name: str, ok: bool) -> str:
    label = f"{_GREEN}OK{_RESET}" if ok else f"{_RED}FAILED{_RESET}"
    return f"{label} {name}"


def format_modules(states: dict[str, bool]) -> str:
    return "\n".join(status_line(name, ok) for name, ok in states.items())
