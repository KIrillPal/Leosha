from __future__ import annotations

import logging
import threading
from pathlib import Path
from time import monotonic, sleep, time

import yaml

from ..algorithms import (
    AutonomyProfile1,
    FollowingProfile,
    PauseProfile,
    SillyFollowingProfile,
    StaringProfile,
    TeleoperationProfile,
    TeleopSlamProfile,
)
from ..interfaces import AlgorithmContext, InputState, OperationProfile
from ..models import ControlCommand, ControlMode, ManualInputState, TelemetryFrame
from .robot_client import RobotClient
from .vision_service import VisionService

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
        vision_service: VisionService | None = None,
        state_file: Path | str | None = None,
        profiles_config: dict[str, dict] | None = None,
    ) -> None:
        self._robot = robot_client
        self._context = context
        self._slam_service = slam_service
        self._vision_service = vision_service if vision_service is not None else VisionService()
        self._state_file = Path(state_file) if state_file else None
        if profiles_config is None:
            raise ValueError("profiles_config is required")
        self._profiles = self._build_profiles(profiles_config)
        self._active_mode = ControlMode.PAUSE
        self._manual = ManualInputState()
        self._last_telemetry = TelemetryFrame()
        self._last_command = ControlCommand()
        self._last_head_command_latency_ms: float | None = None
        self._pending_actions: list[dict] = []
        self._last_tick_at = monotonic()
        self._lock = threading.Lock()
        self._loop_stop = threading.Event()
        self._loop_thread: threading.Thread | None = None
        self._restore_mode_from_state()
        self._restore_profiles_state()

    def _build_profiles(self, profiles_config: dict[str, dict]) -> dict[ControlMode, OperationProfile]:
        """Hardcoded profile classes + per-profile config from YAML."""
        teleop_cfg = dict(profiles_config["teleoperation"])
        teleop_slam_cfg = dict(profiles_config["teleop_slam"])
        autonomy_cfg = dict(profiles_config["autonomy_profile_1"])
        staring_cfg = dict(profiles_config["staring"])
        silly_following_cfg = dict(profiles_config["silly_following"])
        # Following skeleton can be configured but intentionally not mounted as active
        # control mode yet (shares autonomy slot in current protocol).
        self._following_profile = FollowingProfile(**dict(profiles_config["following"]))
        return {
            ControlMode.PAUSE: PauseProfile(),
            ControlMode.TELEOPERATION: TeleoperationProfile(
                title="Телеуправление",
                control_mode=ControlMode.TELEOPERATION,
                **teleop_cfg,
            ),
            ControlMode.TELEOP_SLAM: TeleopSlamProfile(**teleop_slam_cfg),
            ControlMode.AUTONOMY_PROFILE_1: AutonomyProfile1(**autonomy_cfg),
            ControlMode.STARING: StaringProfile(vision_service=self._vision_service, **staring_cfg),
            ControlMode.SILLY_FOLLOWING: SillyFollowingProfile(
                vision_service=self._vision_service,
                **silly_following_cfg,
            ),
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
        with self._state_file.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        if not isinstance(data, dict):
            raise ValueError(f"Invalid state file {self._state_file}: expected mapping")
        if "active_mode" not in data:
            raise ValueError(
                f"State file {self._state_file} is in old format (missing 'active_mode'). "
                "Remove it to start with default mode."
            )
        mode_raw = data["active_mode"]
        if mode_raw not in [m.value for m in ControlMode]:
            raise ValueError(f"Invalid active mode in state file: {mode_raw}")
        self._active_mode = ControlMode(mode_raw)
        LOGGER.info("Restored control mode from state: %s", mode_raw)

    def _restore_profiles_state(self) -> None:
        if not self._state_file or not self._state_file.exists():
            return
        with self._state_file.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        if not isinstance(data, dict):
            raise ValueError(f"Invalid state file {self._state_file}: expected mapping")
        if "profile_states" not in data:
            raise ValueError(
                f"State file {self._state_file} is in old format (missing 'profile_states'). "
                "Remove it to start with default state."
            )
        profile_states = data["profile_states"]
        if not isinstance(profile_states, dict):
            raise ValueError(f"Invalid profile_states in state file {self._state_file}")
        for mode, profile in self._profiles.items():
            state = profile_states.get(mode.value)
            if state is not None:
                if not isinstance(state, dict):
                    raise ValueError(f"Profile state for {mode.value} must be mapping")
                profile.restore_state(state)

    def _save_state(self) -> None:
        if not self._state_file:
            return
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
        if robot_config is not None and "robot_geometry" in robot_config and "head" in robot_config["robot_geometry"]:
            head = robot_config["robot_geometry"]["head"]
            pan_deg = _head_axis_to_deg(
                c.head_pan,
                float(head["neck_min_deg"]),
                float(head["neck_max_deg"]),
            )
            tilt_deg = _head_axis_to_deg(
                c.head_tilt,
                float(head["face_min_deg"]),
                float(head["face_max_deg"]),
            )
        else:
            # Display fallback when robot has not sent config yet (e.g. SLAM tab)
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
                "x": float(ui_state["x"]),
                "y": float(ui_state["y"]),
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
                    "head_command_latency_ms": self._last_head_command_latency_ms,
                },
                "profile_state": ui_state,
            }

    def get_silly_following_tuning(self) -> dict[str, float]:
        with self._lock:
            profile = self._profiles[ControlMode.SILLY_FOLLOWING]
            return {
                "forward_throttle": float(profile._forward_throttle),
                "backward_throttle": float(profile._backward_throttle),
                "eye_confidence_threshold": float(profile._eye_confidence_threshold),
            }

    def set_silly_following_tuning(self, *, forward_throttle: float, backward_throttle: float, eye_confidence_threshold: float) -> dict[str, float]:
        with self._lock:
            profile = self._profiles[ControlMode.SILLY_FOLLOWING]
            profile._forward_throttle = float(forward_throttle)
            profile._backward_throttle = float(backward_throttle)
            profile._eye_confidence_threshold = float(eye_confidence_threshold)
            return {
                "forward_throttle": float(profile._forward_throttle),
                "backward_throttle": float(profile._backward_throttle),
                "eye_confidence_threshold": float(profile._eye_confidence_threshold),
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
                vision_latency_ms=float(self._last_head_command_latency_ms or 0.0),
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
        # Latency from vision result received to command sent (ms)
        try:
            recv = self._vision_service.get_last_receive_time()
            if recv > 0:
                self._last_head_command_latency_ms = (time() - recv) * 1000.0
        except Exception:
            pass
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
