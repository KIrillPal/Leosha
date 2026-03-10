from __future__ import annotations

from ..interfaces import ModeExecutor
from ..models import ActuatorCommand, OperatingMode


class PauseExecutor(ModeExecutor):
    @property
    def mode(self) -> OperatingMode:
        return OperatingMode.PAUSE

    def compute(self, state_snapshot: dict) -> ActuatorCommand:
        return ActuatorCommand()

