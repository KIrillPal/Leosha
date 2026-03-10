from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class OperatingMode(str, Enum):
    PAUSE = "pause"
    TELEOPERATION = "teleoperation"
    TELEOP_SLAM = "teleop_slam"
    AUTONOMY_PROFILE_1 = "autonomy_profile_1"


class RobotStatus(str, Enum):
    RUNNING = "running"
    DEGRADED = "degraded"
    WAITING_FOR_SERVER = "waiting_for_server"
    EMERGENCY_STOP = "emergency_stop"


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


@dataclass
class Twist2D:
    linear: float = 0.0
    angular: float = 0.0


@dataclass
class ImuReading:
    timestamp_ns: int
    accel_x: float
    accel_y: float
    accel_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float


@dataclass
class WheelOdometry:
    timestamp_ns: int
    pose: Pose2D
    velocity: Twist2D
    steering_angle: float


@dataclass
class LaserScan:
    timestamp_ns: int
    angle_min: float
    angle_max: float
    angle_increment: float
    range_min: float
    range_max: float
    ranges: list[float] = field(default_factory=list)
    intensities: list[float] = field(default_factory=list)


@dataclass
class SensorTimingStats:
    sensor_name: str
    read_min_us: float = 0.0
    read_max_us: float = 0.0
    read_avg_us: float = 0.0
    read_last_us: float = 0.0
    cycle_min_us: float = 0.0
    cycle_max_us: float = 0.0
    cycle_avg_us: float = 0.0
    cycle_count: int = 0
    overrun_count: int = 0
    error_count: int = 0
    actual_hz: float = 0.0
    target_hz: float = 0.0


@dataclass
class SensorStatus:
    sensor_name: str
    enabled: bool
    active: bool
    healthy: bool
    failure_count: int = 0
    last_update_ns: int = 0
    last_error: str = ""
    disabled_reason: str = ""


@dataclass
class TelemetryPacket:
    seq: int
    timestamp_ns: int
    mode: OperatingMode
    status: RobotStatus
    frame_jpeg: bytes
    scan: LaserScan | None
    imu_readings: list[ImuReading]
    odometry: WheelOdometry
    ultrasonic_range_m: float
    battery_voltage: float
    cpu_temp_c: float
    wifi_rssi_dbm: float
    sensor_status: dict[str, SensorStatus]
    sensor_timing: dict[str, SensorTimingStats]
    telemetry_pack_us: float = 0.0
    telemetry_send_us: float = 0.0
    # Конфиг робота (углы головы и т.д.) — передаётся при первой связи для отображения на сервере
    robot_config: dict | None = None


@dataclass
class TrajectoryPoint:
    timestamp_ns: int
    x: float
    y: float
    theta: float
    target_speed: float
    curvature: float


@dataclass
class DynamicObstacle:
    object_id: int
    class_name: str
    x: float
    y: float
    vx: float
    vy: float
    radius: float


@dataclass
class MapUpdate:
    is_full: bool
    resolution: float
    origin: Pose2D
    width: int
    height: int
    data: bytes = b""
    changed_cells: list[tuple[int, int]] | None = None


@dataclass
class TeleoperationCommand:
    speed: float
    steering: float
    head_pan: float
    head_tilt: float


@dataclass
class AutonomyCommand:
    fused_pose: Pose2D
    fused_velocity: Twist2D
    trajectory: list[TrajectoryPoint]
    head_pan: float
    head_tilt: float
    dynamic_obstacles: list[DynamicObstacle]
    map_update: MapUpdate | None = None


@dataclass
class ServerPacketHeader:
    seq: int
    timestamp_ns: int
    mode: OperatingMode


@dataclass
class ServerPacket:
    header: ServerPacketHeader
    teleop_cmd: TeleoperationCommand | None = None
    autonomy_cmd: AutonomyCommand | None = None


@dataclass
class ActuatorCommand:
    motor_throttle: float = 0.0
    steering_throttle: float = 0.0
    head_pan_angle: float = 0.0
    head_tilt_angle: float = 0.0


@dataclass
class ActuatorFeedback:
    motor_throttle: float = 0.0
    steering_throttle: float = 0.0

