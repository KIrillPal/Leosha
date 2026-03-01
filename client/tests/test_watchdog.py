from __future__ import annotations

import time

from client.models import RobotStatus
from client.watchdog import Watchdog


def test_watchdog_transitions():
    wd = Watchdog(warn_ms=1.0, timeout_ms=2.0, critical_ms=4.0)
    wd.feed()
    assert wd.status() == RobotStatus.RUNNING
    time.sleep(0.0015)
    assert wd.status() in (RobotStatus.DEGRADED, RobotStatus.WAITING_FOR_SERVER)
    time.sleep(0.002)
    assert wd.status() in (RobotStatus.WAITING_FOR_SERVER, RobotStatus.EMERGENCY_STOP)

