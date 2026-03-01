from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class HealthcheckResult:
    sensor_name: str
    ok: bool
    latency_ms: float
    reason: str = ""
    details: dict[str, Any] = field(default_factory=dict)

