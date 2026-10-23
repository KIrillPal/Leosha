"""Convert physical commands into the throttle and servo angles the hardware expects."""

from __future__ import annotations


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _pairs(calibration: dict) -> list[tuple[float, float]]:
    paired = zip(calibration["speed_mps"], calibration["speed_throttle"])
    return sorted((float(speed), float(throttle)) for speed, throttle in paired)


def throttle_from_speed(speed_mps: float, calibration: dict) -> float:
    points = _pairs(calibration)
    if speed_mps <= points[0][0]:
        return points[0][1]
    if speed_mps >= points[-1][0]:
        return points[-1][1]
    for (left_s, left_t), (right_s, right_t) in zip(points, points[1:]):
        if left_s <= speed_mps <= right_s:
            span = right_s - left_s
            if span == 0.0:
                return right_t
            mix = (speed_mps - left_s) / span
            return left_t + mix * (right_t - left_t)
    return points[-1][1]


def speed_from_throttle(throttle: float, calibration: dict) -> float:
    points = sorted((thr, speed) for speed, thr in _pairs(calibration))
    if throttle <= points[0][0]:
        return points[0][1]
    if throttle >= points[-1][0]:
        return points[-1][1]
    for (left_t, left_s), (right_t, right_s) in zip(points, points[1:]):
        if left_t <= throttle <= right_t:
            span = right_t - left_t
            if span == 0.0:
                return right_s
            mix = (throttle - left_t) / span
            return left_s + mix * (right_s - left_s)
    return points[-1][1]


def steering_throttle(steering_rad: float, calibration: dict) -> float:
    limit = float(calibration["steering_max_rad"])
    max_throttle = float(calibration["steering_max_throttle"])
    fraction = 0.0 if limit == 0.0 else _clamp(steering_rad / limit, -1.0, 1.0)
    throttle = fraction * max_throttle
    if calibration["steering_invert"]:
        throttle = -throttle
    return _clamp(throttle, -max_throttle, max_throttle)


def steering_from_throttle(throttle: float, calibration: dict) -> float:
    max_throttle = float(calibration["steering_max_throttle"])
    limit = float(calibration["steering_max_rad"])
    if calibration["steering_invert"]:
        throttle = -throttle
    if max_throttle == 0.0:
        return 0.0
    fraction = _clamp(throttle / max_throttle, -1.0, 1.0)
    return fraction * limit


def _servo(command_deg: float, zero_deg: float, low: float, high: float) -> float:
    return zero_deg + _clamp(command_deg, low, high)


def pan_servo_deg(command_deg: float, calibration: dict) -> float:
    return _servo(
        command_deg,
        calibration["head_pan_zero_deg"],
        calibration["head_pan_min_deg"],
        calibration["head_pan_max_deg"],
    )


def tilt_servo_deg(command_deg: float, calibration: dict) -> float:
    return _servo(
        command_deg,
        calibration["head_tilt_zero_deg"],
        calibration["head_tilt_min_deg"],
        calibration["head_tilt_max_deg"],
    )


def pan_command_deg(servo_deg: float, calibration: dict) -> float:
    return _clamp(
        servo_deg - calibration["head_pan_zero_deg"],
        calibration["head_pan_min_deg"],
        calibration["head_pan_max_deg"],
    )


def tilt_command_deg(servo_deg: float, calibration: dict) -> float:
    return _clamp(
        servo_deg - calibration["head_tilt_zero_deg"],
        calibration["head_tilt_min_deg"],
        calibration["head_tilt_max_deg"],
    )
