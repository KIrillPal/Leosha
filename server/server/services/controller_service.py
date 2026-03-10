from __future__ import annotations

import logging
import threading
from dataclasses import dataclass
from time import monotonic, sleep

from ..algorithms import AutonomyProfile1, PauseProfile, TeleoperationProfile, TeleopSlamProfile
from ..interfaces import AlgorithmContext, OperationProfile
from ..models import ControlCommand, ControlMode, ManualInputState, TelemetryFrame
from .robot_client import RobotClient

LOGGER = logging.getLogger(__name__)


def _head_axis_to_deg(value: float, min_deg: float, max_deg: float) -> float:
    """Как на клиенте: значение -1..1 в углы (min_deg..max_deg). Не throttle 0/1."""
    value = max(-1.0, min(1.0, float(value)))
    if value >= 0.0:
        return value * float(max_deg)
    return value * abs(float(min_deg))


@dataclass
class TeleopUiState:
    x: float = 0.0
    y: float = 0.0
    tracking_enabled: bool = False


class ControllerService:
    def __init__(
        self,
        robot_client: RobotClient,
        context: AlgorithmContext,
        slam_service=None,
    ) -> None:
        self._robot = robot_client
        self._context = context
        self._slam_service = slam_service
        self._profiles: dict[ControlMode, OperationProfile] = {
            ControlMode.PAUSE: PauseProfile(),
            ControlMode.TELEOPERATION: TeleoperationProfile(),
            ControlMode.TELEOP_SLAM: TeleopSlamProfile(),
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

    def _command_status(self, robot_config: dict | None) -> dict:
        c = self._last_command
        head = None
        if robot_config:
            head = (robot_config.get("robot_geometry") or {}).get("head") or robot_config.get("head")
        if head is not None and isinstance(head, dict):
            pan_deg = _head_axis_to_deg(
                c.head_pan,
                float(head.get("neck_min_deg", -70.0)),
                float(head.get("neck_max_deg", 70.0)),
            )
            tilt_deg = _head_axis_to_deg(
                c.head_tilt,
                float(head.get("face_min_deg", -45.0)),
                float(head.get("face_max_deg", 45.0)),
            )
        else:
            pan_deg = float(c.head_pan) * 60.0
            tilt_deg = float(c.head_tilt) * 45.0
        return {
            "speed": float(c.speed),
            "steering": float(c.steering),
            "head_pan": float(c.head_pan),
            "head_tilt": float(c.head_tilt),
            "head_pan_deg": pan_deg,
            "head_tilt_deg": tilt_deg,
        }

    def get_ui_status(self) -> dict:
        robot_config = self._robot.get_robot_config()
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
                "command": self._command_status(robot_config),
                "telemetry": {
                    "speed_mps": float(self._last_telemetry.speed_mps),
                    "steering_rad": float(self._last_telemetry.steering_rad),
                    "steering_deg": float(self._last_telemetry.steering_rad) * 57.29577951308232,
                    "imu_yaw_rate": float(self._last_telemetry.imu_yaw_rate),
                },
            }

    def tick_once(self) -> ControlCommand:
        telemetry = self._robot.get_latest_telemetry()
        robot_config = self._robot.get_robot_config()
        if self._slam_service and self.active_mode == ControlMode.TELEOP_SLAM:
            self._slam_service.update_from_telemetry()
        with self._lock:
            command = self._profiles[self._active_mode].algorithm.compute_command(
                self._context, self._manual, telemetry, robot_config=robot_config
            )
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
