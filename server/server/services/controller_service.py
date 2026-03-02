from __future__ import annotations

import logging
import threading
from dataclasses import dataclass
from time import monotonic, sleep

from ..algorithms import AutonomyProfile1, PauseProfile, TeleoperationProfile
from ..interfaces import AlgorithmContext, OperationProfile
from ..models import ControlCommand, ControlMode, ManualInputState, TelemetryFrame
from .robot_client import RobotClient

LOGGER = logging.getLogger(__name__)


@dataclass
class TeleopUiState:
    x: float = 0.0
    y: float = 0.0
    tracking_enabled: bool = False


class ControllerService:
    def __init__(self, robot_client: RobotClient, context: AlgorithmContext) -> None:
        self._robot = robot_client
        self._context = context
        self._profiles: dict[ControlMode, OperationProfile] = {
            ControlMode.PAUSE: PauseProfile(),
            ControlMode.TELEOPERATION: TeleoperationProfile(),
            ControlMode.AUTONOMY_PROFILE_1: AutonomyProfile1(),
        }
        self._active_mode = ControlMode.PAUSE
        self._manual = ManualInputState()
        self._ui = TeleopUiState()
        self._last_telemetry = TelemetryFrame()
        self._last_command = ControlCommand()
        self._lock = threading.Lock()
        self._loop_stop = threading.Event()
        self._loop_thread: threading.Thread | None = None

    @property
    def modes(self) -> list[str]:
        return [mode.value for mode in self._profiles]

    @property
    def active_mode(self) -> ControlMode:
        with self._lock:
            return self._active_mode

    def set_mode(self, mode_raw: str) -> ControlMode:
        mode = ControlMode(mode_raw)
        with self._lock:
            if mode != self._active_mode:
                prev_mode = self._active_mode
                self._profiles[self._active_mode].algorithm.on_exit(self._context)
                self._profiles[mode].algorithm.on_enter(self._context)
                self._active_mode = mode
                LOGGER.info("Control mode changed: %s -> %s", prev_mode.value, mode.value)
        return mode

    def set_tracking(self, enabled: bool) -> None:
        with self._lock:
            self._manual.tracking_enabled = bool(enabled)
            self._ui.tracking_enabled = bool(enabled)

    def reset_head_state(self) -> None:
        with self._lock:
            self._manual.head_dx = 0.0
            self._manual.head_dy = 0.0
            self._ui.x = 0.0
            self._ui.y = 0.0

    def apply_mouse_delta(self, dx: float, dy: float) -> None:
        with self._lock:
            self._manual.head_dx = float(dx)
            self._manual.head_dy = float(dy)
            self._ui.x += dx
            self._ui.y += dy

    def apply_keyboard(self, key: str, state: bool) -> None:
        with self._lock:
            key_l = key.lower()
            if hasattr(self._manual, key_l):
                setattr(self._manual, key_l, bool(state))

    def get_ui_status(self) -> dict:
        with self._lock:
            return {
                "x": float(self._ui.x),
                "y": float(self._ui.y),
                "tracking_enabled": self._ui.tracking_enabled,
                "key_states": {
                    "w": self._manual.w,
                    "a": self._manual.a,
                    "s": self._manual.s,
                    "d": self._manual.d,
                    "shift": self._manual.shift,
                    "ctrl": self._manual.ctrl,
                },
                "mode": self._active_mode.value,
            }

    def tick_once(self) -> ControlCommand:
        telemetry = self._robot.get_latest_telemetry()
        with self._lock:
            command = self._profiles[self._active_mode].algorithm.compute_command(self._context, self._manual, telemetry)
            self._manual.head_dx = 0.0
            self._manual.head_dy = 0.0
            self._last_telemetry = telemetry
            self._last_command = command
        self._robot.send_command(command)
        return command

    def start_command_loop(self, hz: float) -> None:
        if self._loop_thread and self._loop_thread.is_alive():
            LOGGER.warning("Controller command loop already running")
            return
        self._loop_stop.clear()
        period = 1.0 / hz

        def _loop() -> None:
            next_t = monotonic()
            while not self._loop_stop.is_set():
                self.tick_once()
                next_t += period
                sleep(max(0.0, next_t - monotonic()))

        self._loop_thread = threading.Thread(target=_loop, daemon=True)
        self._loop_thread.start()
        LOGGER.info("Controller command loop started at %.2f Hz", hz)

    def stop_command_loop(self) -> None:
        self._loop_stop.set()
        if self._loop_thread and self._loop_thread.is_alive():
            self._loop_thread.join(timeout=1.0)
        LOGGER.info("Controller command loop stopped")
