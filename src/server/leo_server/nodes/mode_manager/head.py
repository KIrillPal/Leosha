"""Head behaviours. Angles are degrees and start from the last measured pose."""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

from leo_server.nodes.mode_manager.person_tracker import PersonTracker, Target


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass
class HeadLimits:
    pan_min: float
    pan_max: float
    tilt_min: float
    tilt_max: float


def limits_from(calibration: dict) -> HeadLimits:
    return HeadLimits(
        float(calibration["head_pan_min_deg"]),
        float(calibration["head_pan_max_deg"]),
        float(calibration["head_tilt_min_deg"]),
        float(calibration["head_tilt_max_deg"]),
    )


class HoldHead:
    def __init__(self, pan_deg: float, tilt_deg: float) -> None:
        self.pan = pan_deg
        self.tilt = tilt_deg

    def tick(self, sense, enabled: bool = True) -> tuple[float, float]:
        del sense, enabled
        return self.pan, self.tilt


class MouseHead:
    def __init__(self, pan_deg: float, tilt_deg: float, mouse: dict, limits: HeadLimits) -> None:
        self.pan = pan_deg
        self.tilt = tilt_deg
        self._mouse = mouse
        self._limits = limits

    def tick(self, sense, enabled: bool = True) -> tuple[float, float]:
        if sense.action and sense.action.get("type") == "reset_head":
            self.pan = 0.0
            self.tilt = 0.0
        elif enabled:
            self.pan += sense.keys.head_dx * float(self._mouse["pan_deg_per_px"])
            self.tilt += sense.keys.head_dy * float(self._mouse["tilt_deg_per_px"])
            self.pan = _clamp(self.pan, self._limits.pan_min, self._limits.pan_max)
            self.tilt = _clamp(self.tilt, self._limits.tilt_min, self._limits.tilt_max)
        return self.pan, self.tilt


def compensate_yaw(pan_deg: float, yaw_rate: float, dt: float, gain: float, limits: HeadLimits) -> float:
    moved = pan_deg - math.degrees(yaw_rate) * dt * gain
    return _clamp(moved, limits.pan_min, limits.pan_max)


def track_error(target: Target, frame_w: int, frame_h: int, deadzone: float) -> tuple[float, float]:
    err_x = (target.cx / max(1.0, float(frame_w)) - 0.5) * 2.0
    err_y = (target.cy / max(1.0, float(frame_h)) - 0.5) * 2.0
    if abs(err_x) < deadzone:
        err_x = 0.0
    if abs(err_y) < deadzone:
        err_y = 0.0
    return err_x, err_y


class TrackingHead:
    """Look at a person. With no target, either hold the mouse or report that nobody is locked."""

    def __init__(self, pan_deg: float, tilt_deg: float, tracker: PersonTracker, mouse: MouseHead, tracking_cfg: dict, limits: HeadLimits) -> None:
        self.pan = pan_deg
        self.tilt = tilt_deg
        self.tracker = tracker
        self._mouse = mouse
        self._cfg = tracking_cfg
        self._limits = limits
        self.locked = False

    def tick(self, sense, allow_fallback: bool, use_mouse: bool) -> tuple[float, float]:
        self.tracker.handle_action(sense.action, sense.persons)
        target = None
        if sense.keys.tracking:
            target = self.tracker.update(
                sense.persons,
                sense.frame_id,
                sense.frame_w,
                sense.frame_h,
                sense.dt,
                sense.vision_latency_ms,
                sense.now,
                allow_fallback,
            )
        else:
            self.tracker._clear()
        self.locked = target is not None
        if target is None:
            if use_mouse and sense.keys.tracking:
                self.pan, self.tilt = self._mouse.tick(sense, enabled=True)
            return self.pan, self.tilt
        err_x, err_y = track_error(target, sense.frame_w, sense.frame_h, float(self._cfg["head_tracking_deadzone"]))
        step = float(self._cfg["head_tracking_gain"]) * max(0.0, sense.dt)
        self.pan = _clamp(self.pan - err_x * step * self._limits.pan_max, self._limits.pan_min, self._limits.pan_max)
        self.tilt = _clamp(self.tilt - err_y * step * self._limits.tilt_max, self._limits.tilt_min, self._limits.tilt_max)
        self._mouse.pan, self._mouse.tilt = self.pan, self.tilt
        return self.pan, self.tilt


@dataclass
class SearchMemory:
    lost_since: float | None = None
    target_pan: float | None = None
    target_tilt: float | None = None
    dwell_until: float = 0.0
    previous: tuple[float, float] | None = None
    searching: bool = False


def advance_search(pan: float, tilt: float, memory: SearchMemory, now: float, dt: float, cfg: dict, limits: HeadLimits) -> tuple[float, float]:
    if memory.lost_since is None:
        memory.lost_since = now
    if now - memory.lost_since < float(cfg["search_timeout_sec"]):
        memory.searching = False
        return pan, tilt
    memory.searching = True
    if memory.dwell_until > now:
        return pan, tilt
    if memory.target_pan is None or memory.target_tilt is None:
        memory.target_pan, memory.target_tilt = _pick_target(memory, cfg, limits)
    pan, reached_pan = _step(pan, memory.target_pan, float(cfg["search_pan_deg_per_sec"]) * dt)
    tilt, reached_tilt = _step(tilt, memory.target_tilt, float(cfg["search_tilt_deg_per_sec"]) * dt)
    pan = _clamp(pan, limits.pan_min, limits.pan_max)
    tilt = _clamp(tilt, limits.tilt_min, limits.tilt_max)
    if reached_pan and reached_tilt:
        memory.previous = (memory.target_pan, memory.target_tilt)
        memory.target_pan = None
        memory.target_tilt = None
        memory.dwell_until = now + float(cfg["search_dwell_sec"])
    return pan, tilt


def _step(current: float, target: float, max_step: float) -> tuple[float, bool]:
    delta = target - current
    if abs(delta) <= max_step:
        return target, True
    return current + math.copysign(max_step, delta), False


def _pick_target(memory: SearchMemory, cfg: dict, limits: HeadLimits) -> tuple[float, float]:
    pan_min = max(limits.pan_min, float(cfg["search_pan_min_deg"]))
    pan_max = min(limits.pan_max, float(cfg["search_pan_max_deg"]))
    tilt_min = max(limits.tilt_min, float(cfg["search_tilt_min_deg"]))
    tilt_max = min(limits.tilt_max, float(cfg["search_tilt_max_deg"]))
    need = float(cfg["search_min_target_delta_deg"])
    choice = (pan_min, tilt_min)
    for _ in range(16):
        choice = (random.uniform(pan_min, pan_max), random.uniform(tilt_min, tilt_max))
        if memory.previous is None:
            return choice
        if math.hypot(choice[0] - memory.previous[0], choice[1] - memory.previous[1]) >= need:
            return choice
    return choice
