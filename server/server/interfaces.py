from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from time import monotonic

from .models import ControlCommand, ControlMode, ManualInputState, TelemetryFrame


@dataclass
class AlgorithmContext:
    head_sensitivity: float


@dataclass
class InputState:
    """Unified snapshot for profile tick: operator input + sensors + optional SLAM."""

    manual: ManualInputState = field(default_factory=ManualInputState)
    telemetry: TelemetryFrame = field(default_factory=TelemetryFrame)
    robot_config: dict | None = None
    lidar_scan: dict | None = None
    camera_frame: bytes = b""
    slam_pose: tuple[float, float, float] | None = None
    slam_status: str = "unknown"
    pending_actions: list[dict] = field(default_factory=list)
    dt: float = 0.0
    timestamp: float = field(default_factory=monotonic)
    vision_latency_ms: float = 0.0  # Vision pipeline latency (ms), used for target extrapolation


class ControlAlgorithm(ABC):
    @property
    @abstractmethod
    def mode(self) -> ControlMode:
        raise NotImplementedError

    @property
    @abstractmethod
    def name(self) -> str:
        raise NotImplementedError

    def on_enter(self, context: AlgorithmContext) -> None:
        """Вызывается при активации алгоритма."""

    def on_exit(self, context: AlgorithmContext) -> None:
        """Вызывается при деактивации алгоритма."""

    @abstractmethod
    def compute_command(
        self,
        context: AlgorithmContext,
        manual: ManualInputState,
        telemetry: TelemetryFrame,
        robot_config: dict | None = None,
    ) -> ControlCommand:
        raise NotImplementedError


class AutonomyAlgorithm(ControlAlgorithm, ABC):
    @abstractmethod
    def set_target(self, target: dict) -> None:
        raise NotImplementedError


class OperationProfile(ABC):
    """Base profile contract.

    New runtime should call `tick(context, input_state)`.
    For backwards compatibility with existing tests, profile is also exposed as
    `profile.algorithm.compute_command(...)`.
    """

    @property
    @abstractmethod
    def mode(self) -> ControlMode:
        raise NotImplementedError

    @property
    @abstractmethod
    def title(self) -> str:
        raise NotImplementedError

    @property
    def algorithm(self) -> ControlAlgorithm:
        # Legacy compatibility: profile can act as its own algorithm adapter.
        return self  # type: ignore[return-value]

    @property
    def requires_slam(self) -> bool:
        return False

    def on_activate(self, context: AlgorithmContext) -> None:
        """Lifecycle hook for mode/profile switch."""

    def on_deactivate(self, context: AlgorithmContext) -> None:
        """Lifecycle hook for mode/profile switch."""

    def on_action(self, action: dict) -> None:
        """Optional one-shot action from UI / API."""

    def reset_runtime_state(self, manual: ManualInputState) -> None:
        """Optional profile-specific reset (e.g., camera head state)."""

    def post_tick(self, input_state: InputState) -> None:
        """Optional post tick cleanup (e.g., consume one-shot deltas)."""

    @abstractmethod
    def tick(self, context: AlgorithmContext, input_state: InputState) -> ControlCommand:
        raise NotImplementedError

    def get_ui_state(self) -> dict:
        return {}

    def save_state(self) -> dict:
        return {}

    def restore_state(self, state: dict) -> None:
        del state

    # Legacy compatibility path for old tests calling algorithm.compute_command(...)
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
            dt=0.0,
        )
        command = self.tick(context, state)
        self.post_tick(state)
        return command
