"""Load ``config/robot.yaml``. Missing keys, unknown keys, and a missing file all fail."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

import yaml

from leo_protocol.protocol import HANDSHAKE_PORT


class ConfigError(ValueError):
    """The robot config file is missing or does not match the schema."""


def _as_float(value, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError(f"{path} must be a number")
    return float(value)


def _as_int(value, path: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConfigError(f"{path} must be an integer")
    return int(value)


def _as_bool(value, path: str) -> bool:
    if not isinstance(value, bool):
        raise ConfigError(f"{path} must be a boolean")
    return value


def _as_str(value, path: str) -> str:
    if not isinstance(value, str):
        raise ConfigError(f"{path} must be a string")
    return value


def _mapping(value, path: str) -> dict:
    if not isinstance(value, dict):
        raise ConfigError(f"{path} must be a mapping")
    return value


def _check_keys(data: dict, spec: dict, path: str) -> None:
    extra = set(data) - set(spec)
    missing = set(spec) - set(data)
    if extra or missing:
        raise ConfigError(f"{path} extra={sorted(extra)} missing={sorted(missing)}")


def _floats(value, path: str) -> list[float]:
    if not isinstance(value, list) or not value:
        raise ConfigError(f"{path} must be a non-empty list of numbers")
    return [_as_float(item, f"{path}[]") for item in value]


def _segments(value, path: str) -> tuple[tuple[float, float, float, float], ...]:
    if not isinstance(value, list):
        raise ConfigError(f"{path} must be a list of segments")
    segments = []
    for index, item in enumerate(value):
        if not isinstance(item, list) or len(item) != 4:
            raise ConfigError(f"{path}[{index}] must be [x1, y1, x2, y2]")
        segments.append(tuple(_as_float(v, f"{path}[{index}]") for v in item))
    return tuple(segments)


def _section(data: dict, spec: dict, path: str) -> dict:
    _check_keys(data, spec, path)
    out = {}
    for key, kind in spec.items():
        out[key] = kind(data[key], f"{path}.{key}")
    return out


_FLOAT = _as_float
_INT = _as_int
_BOOL = _as_bool
_STR = _as_str


def _watchdog(value, path: str) -> dict:
    return _section(_mapping(value, path), {"timeout_ms": _FLOAT, "critical_ms": _FLOAT}, path)


def _calibration(value, path: str) -> dict:
    spec = {
        "speed_mps": _floats,
        "speed_throttle": _floats,
        "steering_max_rad": _FLOAT,
        "steering_max_throttle": _FLOAT,
        "steering_invert": _BOOL,
        "head_pan_zero_deg": _FLOAT,
        "head_tilt_zero_deg": _FLOAT,
        "head_pan_min_deg": _FLOAT,
        "head_pan_max_deg": _FLOAT,
        "head_tilt_min_deg": _FLOAT,
        "head_tilt_max_deg": _FLOAT,
    }
    parsed = _section(_mapping(value, path), spec, path)
    if len(parsed["speed_mps"]) != len(parsed["speed_throttle"]):
        raise ConfigError(f"{path} speed_mps and speed_throttle must have the same length")
    return parsed


def _servos(value, path: str) -> dict:
    spec = {
        "pca_channels": _INT,
        "pca_frequency": _INT,
        "motor_channel": _INT,
        "motor_pwm_min": _INT,
        "motor_pwm_max": _INT,
        "wheel_channel": _INT,
        "wheel_pwm_min": _INT,
        "wheel_pwm_max": _INT,
        "neck_channel": _INT,
        "neck_actuation_range": _INT,
        "neck_pwm_min": _INT,
        "neck_pwm_max": _INT,
        "face_channel": _INT,
        "face_actuation_range": _INT,
        "face_pwm_min": _INT,
        "face_pwm_max": _INT,
    }
    return _section(_mapping(value, path), spec, path)


def _camera(value, path: str) -> dict:
    spec = {
        "enabled": _BOOL,
        "fps": _FLOAT,
        "width": _INT,
        "height": _INT,
        "jpeg_quality": _INT,
        "tuning_file": _STR,
        "exposure_time": _INT,
        "analogue_gain": _FLOAT,
        "awb_enable": _BOOL,
        "ae_enable": _BOOL,
    }
    return _section(_mapping(value, path), spec, path)


def _lidar(value, path: str) -> dict:
    spec = {
        "enabled": _BOOL,
        "scan_hz": _FLOAT,
        "baudrate": _INT,
        "sample_rate": _INT,
        "min_angle_deg": _FLOAT,
        "max_angle_deg": _FLOAT,
        "range_min_m": _FLOAT,
        "range_max_m": _FLOAT,
        "num_rays": _INT,
        "zero_angle_deg": _FLOAT,
        "invert_to_ccw": _BOOL,
        "blind_start_deg": _FLOAT,
        "blind_end_deg": _FLOAT,
        "blind_max_m": _FLOAT,
    }
    parsed = _section(_mapping(value, path), spec, path)
    if parsed["num_rays"] < 8:
        raise ConfigError(f"{path}.num_rays must be at least 8")
    return parsed


def _imu(value, path: str) -> dict:
    return _section(_mapping(value, path), {"enabled": _BOOL}, path)


def _client(value, path: str) -> dict:
    spec = {
        "command_hz": _FLOAT,
        "telemetry_hz": _FLOAT,
        "watchdog": _watchdog,
        "calibration": _calibration,
        "servos": _servos,
        "camera": _camera,
        "lidar": _lidar,
        "imu": _imu,
    }
    return _section(_mapping(value, path), spec, path)


def _network(value, path: str) -> dict:
    spec = {
        "bind_address": _STR,
        "robot_ip": _STR,
        "telemetry_port": _INT,
        "handshake_port": _INT,
        "command_port": _INT,
    }
    parsed = _section(_mapping(value, path), spec, path)
    if parsed["handshake_port"] != HANDSHAKE_PORT:
        raise ConfigError(
            f"{path}.handshake_port must equal the protocol port {HANDSHAKE_PORT}"
        )
    return parsed


def _geometry(value, path: str) -> dict:
    spec = {
        "wheelbase_m": _FLOAT,
        "lidar_x": _FLOAT,
        "lidar_y": _FLOAT,
        "lidar_z": _FLOAT,
        "lidar_yaw": _FLOAT,
        "head_x": _FLOAT,
        "head_y": _FLOAT,
        "head_z": _FLOAT,
    }
    return _section(_mapping(value, path), spec, path)


def _keyboard(value, path: str) -> dict:
    spec = {
        "forward_mps": _FLOAT,
        "backward_mps": _FLOAT,
        "fast_mps": _FLOAT,
        "steering_rad": _FLOAT,
        "yaw_damping_gain": _FLOAT,
        "yaw_damping_min": _FLOAT,
    }
    return _section(_mapping(value, path), spec, path)


def _mouse(value, path: str) -> dict:
    spec = {"pan_deg_per_px": _FLOAT, "tilt_deg_per_px": _FLOAT}
    return _section(_mapping(value, path), spec, path)


def _tracking(value, path: str) -> dict:
    spec = {
        "face_match_threshold": _FLOAT,
        "eye_confidence_threshold": _FLOAT,
        "head_tracking_gain": _FLOAT,
        "head_tracking_deadzone": _FLOAT,
        "bbox_target_x_frac": _FLOAT,
        "bbox_target_y_frac": _FLOAT,
        "latency_compensation_cap_frac": _FLOAT,
        "stable_track_frames": _INT,
        "frame_width_fallback": _INT,
        "frame_height_fallback": _INT,
    }
    return _section(_mapping(value, path), spec, path)


def _staring(value, path: str) -> dict:
    return _section(_mapping(value, path), {"gyro_compensation_gain": _FLOAT}, path)


def _silly(value, path: str) -> dict:
    spec = {
        "forward_mps": _FLOAT,
        "backward_mps": _FLOAT,
        "steering_gain": _FLOAT,
        "steering_deadzone_rad": _FLOAT,
        "steering_yaw_damping_gain": _FLOAT,
        "steering_yaw_damping_min": _FLOAT,
        "max_steering_rad": _FLOAT,
        "front_obstacle_distance_m": _FLOAT,
        "side_obstacle_distance_m": _FLOAT,
        "rear_clear_distance_m": _FLOAT,
        "side_steering_nudge_rad": _FLOAT,
        "reverse_timeout_sec": _FLOAT,
        "reverse_steering_multiplier": _FLOAT,
        "forward_speed_steering_reduction_gain": _FLOAT,
        "forward_speed_min_factor": _FLOAT,
        "stop_head_tilt_deg": _FLOAT,
        "stop_without_lidar": _BOOL,
        "front_sector_half_angle_deg": _FLOAT,
        "side_sector_outer_angle_deg": _FLOAT,
        "rear_sector_start_angle_deg": _FLOAT,
        "gyro_compensation_gain": _FLOAT,
        "search_timeout_sec": _FLOAT,
        "search_pan_deg_per_sec": _FLOAT,
        "search_tilt_deg_per_sec": _FLOAT,
        "search_dwell_sec": _FLOAT,
        "search_min_target_delta_deg": _FLOAT,
        "search_pan_min_deg": _FLOAT,
        "search_pan_max_deg": _FLOAT,
        "search_tilt_min_deg": _FLOAT,
        "search_tilt_max_deg": _FLOAT,
    }
    return _section(_mapping(value, path), spec, path)


def _modes(value, path: str) -> dict:
    spec = {
        "tick_hz": _FLOAT,
        "friends_db": _STR,
        "keyboard": _keyboard,
        "mouse": _mouse,
        "tracking": _tracking,
        "staring": _staring,
        "silly_following": _silly,
    }
    return _section(_mapping(value, path), spec, path)


def _odometry(value, path: str) -> dict:
    spec = {
        "publish_hz": _FLOAT,
        "process_noise_xy": _FLOAT,
        "process_noise_yaw": _FLOAT,
        "kinematic_enabled": _BOOL,
        "kinematic_var_xy": _FLOAT,
        "kinematic_var_yaw": _FLOAT,
        "scan_match_enabled": _BOOL,
        "scan_match_var_xy": _FLOAT,
        "scan_match_var_yaw": _FLOAT,
        "scan_match_max_points": _INT,
        "scan_match_max_iters": _INT,
        "scan_match_max_range_m": _FLOAT,
        "scan_match_min_range_m": _FLOAT,
        "scan_match_outlier_m": _FLOAT,
        "imu_enabled": _BOOL,
        "imu_var_yaw": _FLOAT,
        "encoder_enabled": _BOOL,
        "encoder_var_xy": _FLOAT,
        "visual_enabled": _BOOL,
        "visual_var_xy": _FLOAT,
        "visual_var_yaw": _FLOAT,
    }
    return _section(_mapping(value, path), spec, path)


_SLAM_TYPES = {
    "solver_plugin": _STR,
    "ceres_linear_solver": _STR,
    "ceres_preconditioner": _STR,
    "ceres_trust_strategy": _STR,
    "ceres_dogleg_type": _STR,
    "ceres_loss_function": _STR,
    "odom_frame": _STR,
    "map_frame": _STR,
    "base_frame": _STR,
    "scan_topic": _STR,
    "use_map_saver": _BOOL,
    "mode": _STR,
    "debug_logging": _BOOL,
    "throttle_scans": _INT,
    "transform_publish_period": _FLOAT,
    "map_update_interval": _FLOAT,
    "resolution": _FLOAT,
    "restamp_tf": _BOOL,
    "min_laser_range": _FLOAT,
    "max_laser_range": _FLOAT,
    "minimum_time_interval": _FLOAT,
    "transform_timeout": _FLOAT,
    "tf_buffer_duration": _FLOAT,
    "stack_size_to_use": _INT,
    "enable_interactive_mode": _BOOL,
    "use_scan_matching": _BOOL,
    "use_scan_barycenter": _BOOL,
    "minimum_travel_distance": _FLOAT,
    "minimum_travel_heading": _FLOAT,
    "check_min_dist_and_heading_precisely": _BOOL,
    "scan_buffer_size": _INT,
    "scan_buffer_maximum_scan_distance": _FLOAT,
    "link_match_minimum_response_fine": _FLOAT,
    "link_scan_maximum_distance": _FLOAT,
    "loop_search_maximum_distance": _FLOAT,
    "do_loop_closing": _BOOL,
    "loop_match_minimum_chain_size": _INT,
    "loop_match_maximum_variance_coarse": _FLOAT,
    "loop_match_minimum_response_coarse": _FLOAT,
    "loop_match_minimum_response_fine": _FLOAT,
    "correlation_search_space_dimension": _FLOAT,
    "correlation_search_space_resolution": _FLOAT,
    "correlation_search_space_smear_deviation": _FLOAT,
    "loop_search_space_dimension": _FLOAT,
    "loop_search_space_resolution": _FLOAT,
    "loop_search_space_smear_deviation": _FLOAT,
    "distance_variance_penalty": _FLOAT,
    "angle_variance_penalty": _FLOAT,
    "fine_search_angle_offset": _FLOAT,
    "coarse_search_angle_offset": _FLOAT,
    "coarse_angle_resolution": _FLOAT,
    "minimum_angle_penalty": _FLOAT,
    "minimum_distance_penalty": _FLOAT,
    "use_response_expansion": _BOOL,
    "min_pass_through": _INT,
    "occupancy_threshold": _FLOAT,
}


def _slam(value, path: str) -> dict:
    return _section(_mapping(value, path), _SLAM_TYPES, path)


def _vision(value, path: str) -> dict:
    spec = {
        "yolo_model": _STR,
        "yolo_device": _STR,
        "yolo_conf": _FLOAT,
        "yolo_imgsz": _INT,
        "tracker": _STR,
        "enable_face_embedding": _BOOL,
        "face_model": _STR,
        "embedding_interval": _INT,
        "face_min_confidence": _FLOAT,
        "head_yaw_min_deg": _FLOAT,
        "head_yaw_max_deg": _FLOAT,
        "head_yaw_scale_deg": _FLOAT,
        "face_crop_ratio": _FLOAT,
        "embedding_input_size": _INT,
        "embedding_norm_mean": _FLOAT,
        "embedding_norm_std": _FLOAT,
        "yaw_eye_dx_epsilon": _FLOAT,
        "log_level": _STR,
    }
    return _section(_mapping(value, path), spec, path)


def _host_port(value, path: str) -> dict:
    return _section(_mapping(value, path), {"host": _STR, "port": _INT}, path)


def _monitor(value, path: str) -> dict:
    spec = {"host": _STR, "port": _INT, "stale_sec": _FLOAT, "refresh_hz": _FLOAT}
    return _section(_mapping(value, path), spec, path)


def _person(value, path: str) -> dict:
    spec = {"x": _FLOAT, "y": _FLOAT, "radius": _FLOAT, "travel": _FLOAT}
    return _section(_mapping(value, path), spec, path)


def _persons(value, path: str) -> tuple:
    if not isinstance(value, list):
        raise ConfigError(f"{path} must be a list")
    return tuple(_person(item, f"{path}[{i}]") for i, item in enumerate(value))


def _world(value, path: str) -> dict:
    spec = {
        "start_x": _FLOAT,
        "start_y": _FLOAT,
        "start_yaw": _FLOAT,
        "walls": _segments,
        "boxes": _segments,
        "persons": _persons,
    }
    return _section(_mapping(value, path), spec, path)


def _worlds(value, path: str) -> dict:
    data = _mapping(value, path)
    if not data:
        raise ConfigError(f"{path} must name at least one world")
    return {str(name): _world(body, f"{path}.{name}") for name, body in data.items()}


def _route_step(value, path: str) -> dict:
    spec = {"speed_mps": _FLOAT, "steering_rad": _FLOAT, "duration_sec": _FLOAT}
    return _section(_mapping(value, path), spec, path)


def _route(value, path: str) -> tuple:
    if not isinstance(value, list) or not value:
        raise ConfigError(f"{path} must be a non-empty list")
    return tuple(_route_step(item, f"{path}[{i}]") for i, item in enumerate(value))


def _sim(value, path: str) -> dict:
    spec = {
        "physics_hz": _FLOAT,
        "slip": _FLOAT,
        "speed_noise_mps": _FLOAT,
        "yaw_noise_rad": _FLOAT,
        "lidar_noise_m": _FLOAT,
        "camera_width": _INT,
        "camera_height": _INT,
        "camera_fov_deg": _FLOAT,
        "camera_fps": _FLOAT,
        "jpeg_quality": _INT,
        "route_ate_limit_m": _FLOAT,
        "worlds": _worlds,
        "route": _route,
    }
    return _section(_mapping(value, path), spec, path)


_ROOT = {
    "network": _network,
    "client": _client,
    "geometry": _geometry,
    "modes": _modes,
    "odometry": _odometry,
    "slam_toolbox": _slam,
    "vision": _vision,
    "web_ui": _host_port,
    "monitor": _monitor,
    "sim": _sim,
}


@dataclass(frozen=True)
class RobotConfig:
    path: Path
    root: Path
    network: dict
    client: dict
    geometry: dict
    modes: dict
    odometry: dict
    slam_toolbox: dict
    vision: dict
    web_ui: dict
    monitor: dict
    sim: dict

    def friends_path(self) -> Path:
        raw = Path(self.modes["friends_db"])
        if raw.is_absolute():
            return raw
        return self.root / raw

    def client_payload(self) -> dict:
        return self.client

    def sim_payload(self, world: str) -> dict:
        worlds = self.sim["worlds"]
        if world not in worlds:
            known = ", ".join(sorted(worlds))
            raise ConfigError(f"unknown sim world {world!r}; known: {known}")
        return {
            "physics_hz": self.sim["physics_hz"],
            "slip": self.sim["slip"],
            "speed_noise_mps": self.sim["speed_noise_mps"],
            "yaw_noise_rad": self.sim["yaw_noise_rad"],
            "lidar_noise_m": self.sim["lidar_noise_m"],
            "camera_width": self.sim["camera_width"],
            "camera_height": self.sim["camera_height"],
            "camera_fov_deg": self.sim["camera_fov_deg"],
            "camera_fps": self.sim["camera_fps"],
            "jpeg_quality": self.sim["jpeg_quality"],
            "wheelbase_m": self.geometry["wheelbase_m"],
            "world": worlds[world],
            "route": self.sim["route"],
            "lidar": {
                "num_rays": self.client["lidar"]["num_rays"],
                "range_min_m": self.client["lidar"]["range_min_m"],
                "range_max_m": self.client["lidar"]["range_max_m"],
                "scan_hz": self.client["lidar"]["scan_hz"],
                "blind_start_deg": self.client["lidar"]["blind_start_deg"],
                "blind_end_deg": self.client["lidar"]["blind_end_deg"],
                "blind_max_m": self.client["lidar"]["blind_max_m"],
            },
            "calibration": self.client["calibration"],
        }


def load_config(path: str | Path) -> RobotConfig:
    config_path = Path(path)
    if not config_path.is_file():
        raise ConfigError(f"config file not found: {config_path}")
    loaded = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    data = _mapping(loaded, "config")
    parsed = _section(data, _ROOT, "config")
    return RobotConfig(
        path=config_path.resolve(),
        root=config_path.resolve().parent.parent,
        network=parsed["network"],
        client=parsed["client"],
        geometry=parsed["geometry"],
        modes=parsed["modes"],
        odometry=parsed["odometry"],
        slam_toolbox=parsed["slam_toolbox"],
        vision=parsed["vision"],
        web_ui=parsed["web_ui"],
        monitor=parsed["monitor"],
        sim=parsed["sim"],
    )


def config_from_env(argv_config: str | None = None) -> RobotConfig:
    chosen = argv_config or os.environ.get("LEO_CONFIG")
    if not chosen:
        raise ConfigError("pass --config or set LEO_CONFIG to config/robot.yaml")
    return load_config(chosen)
