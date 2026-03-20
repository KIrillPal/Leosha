from __future__ import annotations

from ..interfaces import AlgorithmContext, InputState, OperationProfile
from ..models import ControlCommand, ManualInputState, TelemetryFrame


class BaseControlProfile(OperationProfile):
    """Reusable base profile for all behavior modes."""

    @property
    def algorithm(self):
        # Backwards compatibility with legacy tests.
        return self

    def compute_command(
        self,
        context: AlgorithmContext,
        manual: ManualInputState,
        telemetry: TelemetryFrame,
        robot_config: dict | None = None,
    ) -> ControlCommand:
        state = InputState(
            manual=manual,
            telemetry=telemetry,
            robot_config=robot_config,
        )
        command = self.tick(context, state)
        self.post_tick(state)
        return command
