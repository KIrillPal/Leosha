from __future__ import annotations

import logging
import threading
from pathlib import Path
from time import monotonic, sleep

import yaml

from ..algorithms import AutonomyProfile1, FollowingProfile, PauseProfile, TeleoperationProfile, TeleopSlamProfile
from ..interfaces import AlgorithmContext, InputState, OperationProfile
from ..models import ControlCommand, ControlMode, ManualInputState, TelemetryFrame
from .robot_client import RobotClient

LOGGER = logging.getLogger(__name__)


def _head_axis_to_deg(value: float, min_deg: float, max_deg: float) -> float:
    """Как на клиенте: значение -1..1 в углы (min_deg..max_deg). Не throttle 0/1."""
    value = max(-1.0, min(1.0, float(value)))
    if value >= 0.0:
        return value * float(max_deg)
    return value * abs(float(min_deg))


class ControllerService:
    def __init__(
        self,
        robot_client: RobotClient,
        context: AlgorithmContext,
        slam_service=None,
        state_file: Path | str | None = None,
        profiles_config: dict[str, dict] | None = None,
    ) -> None:
        self._robot = robot_client
        self._context = context
        self._slam_service = slam_service
        self._state_file = Path(state_file) if state_file else None
        self._profiles = self._build_profiles(profiles_config or {})
        self._active_mode = ControlMode.PAUSE
        self._manual = ManualInputState()
        self._last_telemetry = TelemetryFrame()
        self._last_command = ControlCommand()
        self._pending_actions: list[dict] = []
        self._last_tick_at = monotonic()
        self._lock = threading.Lock()
        self._loop_stop = threading.Event()
        self._loop_thread: threading.Thread | None = None
        self._restore_mode_from_state()
        self._restore_profiles_state()

    def _build_profiles(self, profiles_config: dict[str, dict]) -> dict[ControlMode, OperationProfile]:
        """Hardcoded profile classes + per-profile config from YAML."""
        teleop_cfg = dict(profiles_config.get("teleoperation") or {})
        teleop_slam_cfg = dict(profiles_config.get("teleop_slam") or {})
        autonomy_cfg = dict(profiles_config.get("autonomy_profile_1") or {})
        # Following skeleton can be configured but intentionally not mounted as active
        # control mode yet (shares autonomy slot in current protocol).
        self._following_profile = FollowingProfile(**dict(profiles_config.get("following") or {}))
        return {
            ControlMode.PAUSE: PauseProfile(),
            ControlMode.TELEOPERATION: TeleoperationProfile(**teleop_cfg),
            ControlMode.TELEOP_SLAM: TeleopSlamProfile(**teleop_slam_cfg),
            ControlMode.AUTONOMY_PROFILE_1: AutonomyProfile1(**autonomy_cfg),
        }

    @property
    def modes(self) -> list[str]:
        return [mode.value for mode in self._profiles]

    @property
    def active_mode(self) -> ControlMode:
        with self._lock:
            return self._active_mode

    @property
    def active_profile(self) -> OperationProfile:
        with self._lock:
            return self._profiles[self._active_mode]

    def is_slam_active(self) -> bool:
        with self._lock:
            return bool(getattr(self._profiles[self._active_mode], "requires_slam", False))

    def _restore_mode_from_state(self) -> None:
        if not self._state_file or not self._state_file.exists():
            return
        try:
            with self._state_file.open("r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            mode_raw = data.get("active_mode", data.get("mode"))
            if mode_raw and mode_raw in [m.value for m in ControlMode]:
                self._active_mode = ControlMode(mode_raw)
                LOGGER.info("Restored control mode from state: %s", mode_raw)
        except Exception as e:
            LOGGER.debug("Could not restore mode from %s: %s", self._state_file, e)

    def _restore_profiles_state(self) -> None:
        if not self._state_file or not self._state_file.exists():
            return
        try:
            with self._state_file.open("r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            profile_states = data.get("profile_states") or {}
            if not isinstance(profile_states, dict):
                return
            for mode, profile in self._profiles.items():
                state = profile_states.get(mode.value)
                if isinstance(state, dict):
                    profile.restore_state(state)
        except Exception as e:
            LOGGER.debug("Could not restore profile states from %s: %s", self._state_file, e)

    def _save_state(self) -> None:
        if not self._state_file:
            return
        try:
            self._state_file.parent.mkdir(parents=True, exist_ok=True)
            with self._state_file.open("w", encoding="utf-8") as f:
                yaml.safe_dump(
                    {
                        "active_mode": self._active_mode.value,
                        "profile_states": {
                            mode.value: profile.save_state()
                            for mode, profile in self._profiles.items()
                        },
                    },
                    f,
                    allow_unicode=True,
                )
        except Exception as e:
            LOGGER.warning("Could not save mode to %s: %s", self._state_file, e)

    def set_mode(self, mode_raw: str) -> ControlMode:
        mode = ControlMode(mode_raw)
        with self._lock:
            if mode != self._active_mode:
                prev_mode = self._active_mode
                self._profiles[self._active_mode].on_deactivate(self._context)
                self._profiles[mode].on_activate(self._context)
                self._active_mode = mode
                self._save_state()
                LOGGER.info("Control mode changed: %s -> %s", prev_mode.value, mode.value)
        return mode

    def set_tracking(self, enabled: bool) -> None:
        with self._lock:
            self._manual.tracking_enabled = bool(enabled)

    def reset_head_state(self) -> None:
        with self._lock:
            self._profiles[self._active_mode].reset_runtime_state(self._manual)

    def apply_mouse_delta(self, dx: float, dy: float) -> None:
        with self._lock:
            self._manual.head_dx = float(dx)
            self._manual.head_dy = float(dy)

    def apply_keyboard(self, key: str, state: bool) -> None:
        with self._lock:
            key_l = key.lower()
            if hasattr(self._manual, key_l):
                setattr(self._manual, key_l, bool(state))

    def add_action(self, action: dict) -> None:
        with self._lock:
            self._pending_actions.append(dict(action))

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
        ui_state = self.active_profile.get_ui_state()
        with self._lock:
            return {
                "x": float(ui_state.get("x", 0.0)),
                "y": float(ui_state.get("y", 0.0)),
                "tracking_enabled": self._manual.tracking_enabled,
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
                "profile_state": ui_state,
            }

    def tick_once(self) -> ControlCommand:
        telemetry = self._robot.get_latest_telemetry()
        robot_config = self._robot.get_robot_config()
        lidar_scan = self._robot.get_latest_lidar_scan()
        camera_frame = self._robot.get_latest_frame()
        now = monotonic()

        slam_pose = None
        slam_status = "unknown"
        profile = self.active_profile
        if self._slam_service and profile.requires_slam:
            self._slam_service.update_from_telemetry()
            slam_pose = self._slam_service.get_pose()
            slam_status = self._slam_service.get_stats().status

        with self._lock:
            input_state = InputState(
                manual=self._manual,
                telemetry=telemetry,
                robot_config=robot_config,
                lidar_scan=lidar_scan,
                camera_frame=camera_frame,
                slam_pose=slam_pose,
                slam_status=slam_status,
                pending_actions=list(self._pending_actions),
                dt=max(0.0, now - self._last_tick_at),
                timestamp=now,
            )
            self._pending_actions.clear()
            for action in input_state.pending_actions:
                profile.on_action(action)
            command = profile.tick(self._context, input_state)
            profile.post_tick(input_state)
            self._last_telemetry = telemetry
            self._last_command = command
            self._last_tick_at = now
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
