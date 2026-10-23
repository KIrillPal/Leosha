"""Rolling rate meter used by the client loops and the server heartbeat."""

from __future__ import annotations

import time


class Rate:
    def __init__(self) -> None:
        self.value = 0.0
        self._count = 0
        self._window_start = time.monotonic()

    def tick(self, now: float | None = None) -> float:
        self._count += 1
        moment = time.monotonic() if now is None else now
        elapsed = moment - self._window_start
        if elapsed >= 0.5:
            self.value = self._count / elapsed
            self._count = 0
            self._window_start = moment
        return self.value
