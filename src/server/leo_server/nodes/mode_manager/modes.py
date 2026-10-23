"""The one place that owns the robot mode.

Switching mode drops the previous behaviour objects, so map-building,
tracking and key state cannot leak. Head commands start from the measured
joint angles, so the servos do not jump to zero.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from leo_server.nodes.mode_manager.drive import FollowMemory, follow_drive, keyboard_drive
from leo_server.nodes.mode_manager.friends import FriendDB
from leo_server.nodes.mode_manager.head import (
    HoldHead,
    MouseHead,
    SearchMemory,
    TrackingHead,
    advance_search,
    compensate_yaw,
    limits_from,
)
from leo_server.nodes.mode_manager.person_tracker import PersonTracker

MODES = (
    "pause",
    "teleoperation",
    "teleop_slam",
    "staring",
    "silly_following",
    "autonomy_profile_1",
)
SLAM_MODES = frozenset({"teleop_slam"})


@dataclass
class Keys:
    w: bool = False
    a: bool = False
    s: bool = False
    d: bool = False
    shift: bool = False
    tracking: bool = False
    head_dx: float = 0.0
    head_dy: float = 0.0


@dataclass
class ScanView:
    angle_min: float
    angle_increment: float
    ranges: list


@dataclass
class Sense:
    dt: float
    now: float
    keys: Keys
    persons: list = field(default_factory=list)
    frame_id: int = 0
    frame_w: int = 640
    frame_h: int = 480
    vision_latency_ms: float = 0.0
    scan: ScanView | None = None
    yaw_rate: float = 0.0
    action: dict | None = None


@dataclass
class Actuation:
    speed_mps: float
    steering_rad: float
    head_pan_deg: float
    head_tilt_deg: float


class _Pause:
    def __init__(self, pan: float, tilt: float) -> None:
        self._head = HoldHead(pan, tilt)

    def tick(self, sense: Sense) -> Actuation:
        pan, tilt = self._head.tick(sense)
        return Actuation(0.0, 0.0, pan, tilt)

    def ui(self) -> dict:
        return {"x": self._head.pan, "y": self._head.tilt}


class _Teleop:
    def __init__(self, cfg: dict, pan: float, tilt: float) -> None:
        limits = limits_from(cfg["client"]["calibration"])
        self._keys = cfg["modes"]["keyboard"]
        self._head = MouseHead(pan, tilt, cfg["modes"]["mouse"], limits)

    def tick(self, sense: Sense) -> Actuation:
        speed, steer = keyboard_drive(sense.keys, self._keys, sense.yaw_rate, sense.keys.tracking)
        pan, tilt = self._head.tick(sense, enabled=sense.keys.tracking)
        return Actuation(speed, steer, pan, tilt)

    def ui(self) -> dict:
        return {"x": self._head.pan, "y": self._head.tilt, "tracking_enabled": False}


class _Staring:
    def __init__(self, cfg: dict, friends: FriendDB, pan: float, tilt: float) -> None:
        limits = limits_from(cfg["client"]["calibration"])
        self._keys = cfg["modes"]["keyboard"]
        self._gain = float(cfg["modes"]["staring"]["gyro_compensation_gain"])
        self._limits = limits
        mouse = MouseHead(pan, tilt, cfg["modes"]["mouse"], limits)
        self._head = TrackingHead(pan, tilt, PersonTracker(cfg["modes"]["tracking"], friends), mouse, cfg["modes"]["tracking"], limits)

    def tick(self, sense: Sense) -> Actuation:
        speed, steer = keyboard_drive(sense.keys, self._keys, sense.yaw_rate, True)
        self._head.pan = compensate_yaw(self._head.pan, sense.yaw_rate, sense.dt, self._gain, self._limits)
        pan, tilt = self._head.tick(sense, allow_fallback=True, use_mouse=True)
        return Actuation(speed, steer, pan, tilt)

    def ui(self) -> dict:
        state = self._head.tracker.ui()
        state["x"] = self._head.pan
        state["y"] = self._head.tilt
        return state


class _Silly:
    def __init__(self, cfg: dict, friends: FriendDB, pan: float, tilt: float) -> None:
        limits = limits_from(cfg["client"]["calibration"])
        self._cfg = cfg["modes"]["silly_following"]
        self._limits = limits
        self._gain = float(self._cfg["gyro_compensation_gain"])
        mouse = MouseHead(pan, tilt, cfg["modes"]["mouse"], limits)
        self._head = TrackingHead(pan, tilt, PersonTracker(cfg["modes"]["tracking"], friends), mouse, cfg["modes"]["tracking"], limits)
        self._follow = FollowMemory()
        self._search = SearchMemory()

    def tick(self, sense: Sense) -> Actuation:
        self._head.pan = compensate_yaw(self._head.pan, sense.yaw_rate, sense.dt, self._gain, self._limits)
        pan, tilt = self._head.tick(sense, allow_fallback=True, use_mouse=False)
        if not sense.keys.tracking or not self._head.locked:
            self._follow.phase = "idle"
            if sense.keys.tracking:
                pan, tilt = advance_search(pan, tilt, self._search, sense.now, sense.dt, self._cfg, self._limits)
                self._head.pan, self._head.tilt = pan, tilt
            else:
                self._search.lost_since = None
            return Actuation(0.0, 0.0, pan, tilt)
        self._search = SearchMemory()
        speed, steer = follow_drive(
            pan,
            tilt,
            self._limits.pan_max,
            sense.scan,
            sense.yaw_rate,
            sense.now,
            self._follow,
            self._cfg,
        )
        return Actuation(speed, steer, pan, tilt)

    def ui(self) -> dict:
        state = self._head.tracker.ui()
        state.update({"x": self._head.pan, "y": self._head.tilt, "follow_state": self._follow.phase, "searching": self._search.searching})
        return state


class _Autonomy:
    def __init__(self, pan: float, tilt: float) -> None:
        self._head = HoldHead(pan, tilt)
        self.target = None

    def tick(self, sense: Sense) -> Actuation:
        if sense.action and sense.action.get("type") == "set_target":
            self.target = sense.action.get("target")
        pan, tilt = self._head.tick(sense)
        return Actuation(0.0, 0.0, pan, tilt)

    def ui(self) -> dict:
        return {"x": self._head.pan, "y": self._head.tilt, "target": self.target}


def _build(mode: str, cfg: dict, friends: FriendDB, pan: float, tilt: float):
    if mode == "pause":
        return _Pause(pan, tilt)
    if mode in ("teleoperation", "teleop_slam"):
        return _Teleop(cfg, pan, tilt)
    if mode == "staring":
        return _Staring(cfg, friends, pan, tilt)
    if mode == "silly_following":
        return _Silly(cfg, friends, pan, tilt)
    if mode == "autonomy_profile_1":
        return _Autonomy(pan, tilt)
    raise ValueError(f"unknown mode {mode}")


class ModeController:
    def __init__(self, cfg: dict, friends_path) -> None:
        self._cfg = cfg
        self.friends = FriendDB(friends_path)
        self.mode = "pause"
        self.measured_pan = 0.0
        self.measured_tilt = 0.0
        self._behaviour = _build(self.mode, cfg, self.friends, 0.0, 0.0)
        self._restart_slam = False
        self.ui_state: dict = {}

    def set_measured_joints(self, pan_deg: float, tilt_deg: float) -> None:
        self.measured_pan = float(pan_deg)
        self.measured_tilt = float(tilt_deg)

    def set_mode(self, mode: str) -> None:
        if mode not in MODES:
            raise ValueError(f"unknown mode {mode}")
        if mode == self.mode:
            return
        self.mode = mode
        self._behaviour = _build(mode, self._cfg, self.friends, self.measured_pan, self.measured_tilt)
        self._restart_slam = mode in SLAM_MODES

    def uses_slam(self) -> bool:
        return self.mode in SLAM_MODES

    def consume_slam_restart(self) -> bool:
        restart = self._restart_slam
        self._restart_slam = False
        return restart

    def tick(self, sense: Sense) -> Actuation:
        command = self._behaviour.tick(sense)
        self.ui_state = self._behaviour.ui()
        self.ui_state["mode"] = self.mode
        self.ui_state["uses_slam"] = self.uses_slam()
        return command
