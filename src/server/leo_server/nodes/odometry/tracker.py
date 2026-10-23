"""Odometry pipeline: noise grow, then each enabled motion source, then a scan match.

IMU, encoders and visual SLAM are invoked from here when enabled. Their
functions return nothing until those sensors are added.
"""

from __future__ import annotations

from leo_server.nodes.odometry.fusion import Fusion2D
from leo_server.nodes.odometry.sources import (
    body_delta,
    measure_encoder,
    measure_imu,
    measure_visual,
    prepare_scan,
    scan_measure,
)


class Tracker:
    def __init__(self, cfg: dict, wheelbase_m: float) -> None:
        self.cfg = cfg
        self.wheelbase_m = float(wheelbase_m)
        self.fusion = Fusion2D()
        self.speed_mps = 0.0
        self.steering_rad = 0.0
        self.gyro_z: float | None = None
        self.encoder: tuple[float | None, float | None] = (None, None)
        self.visual_delta = None
        self._previous = None
        self._anchor = (0.0, 0.0, 0.0)

    def set_applied(self, speed_mps: float, steering_rad: float) -> None:
        self.speed_mps = float(speed_mps)
        self.steering_rad = float(steering_rad)

    def step(self, dt: float) -> None:
        if dt <= 0.0:
            return
        self.fusion.predict(dt, float(self.cfg["process_noise_xy"]), float(self.cfg["process_noise_yaw"]))
        if self.cfg["kinematic_enabled"]:
            dx, dy, dyaw = body_delta(self.speed_mps, self.steering_rad, dt, self.wheelbase_m)
            self.fusion.predict_body(
                dx,
                dy,
                dyaw,
                float(self.cfg["kinematic_var_xy"]) * dt,
                float(self.cfg["kinematic_var_yaw"]) * dt,
            )
        self._apply(self._imu(dt))
        self._apply(self._encoder(dt))
        self._apply(self._visual())

    def observe_scan(self, ranges, angle_min: float, increment: float, dt: float) -> None:
        points = prepare_scan(ranges, angle_min, increment, self.cfg)
        if self.cfg["scan_match_enabled"] and self._previous is not None and len(points):
            seed = body_delta(self.speed_mps, self.steering_rad, max(dt, 1e-3), self.wheelbase_m)
            # The match is motion since the previous scan, so it composes onto that
            # pose. Kinematics already moved the filter in between; this corrects it.
            self._apply(scan_measure(self._anchor[0], self._anchor[1], self._anchor[2], self._previous, points, seed, self.cfg))
        if len(points):
            self._previous = points
            self._anchor = (self.fusion.x, self.fusion.y, self.fusion.yaw)

    def reset_memory(self) -> None:
        self.fusion.forget_motion()
        self._previous = None
        self._anchor = (self.fusion.x, self.fusion.y, self.fusion.yaw)

    def _imu(self, dt: float):
        if not self.cfg["imu_enabled"]:
            return None
        return measure_imu(self.gyro_z, self.fusion.x, self.fusion.y, self.fusion.yaw, dt, float(self.cfg["imu_var_yaw"]))

    def _encoder(self, dt: float):
        if not self.cfg["encoder_enabled"]:
            return None
        left, right = self.encoder
        return measure_encoder(left, right, self.fusion.x, self.fusion.y, self.fusion.yaw, dt, float(self.cfg["encoder_var_xy"]))

    def _visual(self):
        if not self.cfg["visual_enabled"]:
            return None
        return measure_visual(
            self.visual_delta,
            self.fusion.x,
            self.fusion.y,
            self.fusion.yaw,
            float(self.cfg["visual_var_xy"]),
            float(self.cfg["visual_var_yaw"]),
        )

    def _apply(self, measure) -> None:
        if measure is None:
            return
        self.fusion.update_pose(measure.x, measure.y, measure.yaw, measure.var_xy, measure.var_yaw)
