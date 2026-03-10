from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

from .models import ControlCommand, ControlMode, ManualInputState, TelemetryFrame


@dataclass
class AlgorithmContext:
    head_sensitivity: float


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
    @property
    @abstractmethod
    def mode(self) -> ControlMode:
        raise NotImplementedError

    @property
    @abstractmethod
    def title(self) -> str:
        raise NotImplementedError

    @property
    @abstractmethod
    def algorithm(self) -> ControlAlgorithm:
        raise NotImplementedError
