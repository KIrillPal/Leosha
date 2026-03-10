from __future__ import annotations

from dataclasses import asdict, dataclass, field

import yaml


@dataclass
class Vec3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Size3D:
    length: float = 0.0
    width: float = 0.0
    height: float = 0.0


@dataclass
class WheelConfig:
    radius: float = 0.0
    width: float = 0.0


@dataclass
class BlindZoneConfig:
    angle_start_deg: float = -20.0
    angle_end_deg: float = 20.0
    max_distance_m: float = 0.35


@dataclass
class LidarMountConfig:
    position: Vec3 = field(default_factory=Vec3)
    blind_zone: BlindZoneConfig = field(default_factory=BlindZoneConfig)


@dataclass
class HeadMountConfig:
    position: Vec3 = field(default_factory=Vec3)
    neck_min_deg: float = -70.0
    neck_max_deg: float = 70.0
    face_min_deg: float = -45.0
    face_max_deg: float = 45.0


@dataclass
class RobotGeometryConfig:
    body_size: Size3D = field(default_factory=Size3D)
    pose_on_body: Vec3 = field(default_factory=Vec3)
    wheel: WheelConfig = field(default_factory=WheelConfig)
    lidar: LidarMountConfig = field(default_factory=LidarMountConfig)
    head: HeadMountConfig = field(default_factory=HeadMountConfig)


@dataclass
class AppConfig:
    telemetry_hz: float = 30.0
    command_hz: float = 100.0


@dataclass
class WatchdogConfig:
    warn_ms: float = 50.0
    timeout_ms: float = 100.0
    critical_ms: float = 500.0


@dataclass
class DiagnosticsConfig:
    battery_voltage: float = 11.8
    cpu_temp_c: float = 56.0
    wifi_rssi_dbm: float = -44.0


@dataclass
class SensorFailPolicy:
    max_consecutive_failures: int = 3
    auto_disable_on_fail: bool = True
    retry_interval_sec: float = 1.0


@dataclass
class CameraConfig:
    enabled: bool = False
    required_for_motion: bool = False
    fps: float = 30.0
    resolution: tuple[int, int] = (640, 480)
    tuning_file: str = ""
    exposure_time: int = 8000
    analogue_gain: float = 2.0
    awb_enable: bool = False
    ae_enable: bool = False
    jpeg_quality: int = 70
    healthcheck_timeout_sec: float = 2.5
    healthcheck_test_timeout_sec: float = 60.0  # max time test waits for healthcheck (incl. blocking init)
    healthcheck_min_frames: int = 2
    fail_policy: SensorFailPolicy = field(default_factory=SensorFailPolicy)


@dataclass
class LidarConfig:
    enabled: bool = False
    required_for_motion: bool = True
    port: str = "/dev/ydlidar"
    auto_discover_port: bool = True
    baudrate: int = 230400
    scan_hz: float = 10.0
    min_angle_deg: float = -180.0
    max_angle_deg: float = 180.0
    range_min_m: float = 0.02
    range_max_m: float = 12.0
    intensity_enabled: bool = True
    invert_angle: bool = True
    zero_angle_deg: float = -90.0  # угол, который считается нулём (на нём будет 0 после сдвига)
    healthcheck_timeout_sec: float = 3.0
    healthcheck_test_timeout_sec: float = 30.0  # max time test waits for healthcheck (incl. blocking init)
    healthcheck_min_points: int = 32
    fail_policy: SensorFailPolicy = field(default_factory=SensorFailPolicy)


@dataclass
class ImuConfig:
    enabled: bool = True
    hz: float = 100.0
    required_for_motion: bool = False
    fail_policy: SensorFailPolicy = field(default_factory=SensorFailPolicy)


@dataclass
class EncoderConfig:
    enabled: bool = True
    hz: float = 100.0
    required_for_motion: bool = False
    fail_policy: SensorFailPolicy = field(default_factory=SensorFailPolicy)


@dataclass
class UltrasonicConfig:
    enabled: bool = True
    hz: float = 10.0
    emergency_stop_distance_m: float = 0.2
    required_for_motion: bool = True
    fail_policy: SensorFailPolicy = field(default_factory=SensorFailPolicy)


@dataclass
class SensorsConfig:
    camera: CameraConfig = field(default_factory=CameraConfig)
    lidar: LidarConfig = field(default_factory=LidarConfig)
    imu: ImuConfig = field(default_factory=ImuConfig)
    encoder: EncoderConfig = field(default_factory=EncoderConfig)
    ultrasonic: UltrasonicConfig = field(default_factory=UltrasonicConfig)


@dataclass
class NetworkConfig:
    server_host: str = "127.0.0.1"
    telemetry_port: int = 5550
    command_port: int = 5552
    report_port: int = 5553
    recv_timeout_ms: int = 0
    send_high_water_mark: int = 2
    recv_high_water_mark: int = 1


@dataclass
class PcaConfig:
    channels: int = 16
    frequency: int = 250


@dataclass
class MotorActuatorConfig:
    channel: int = 0
    zero_throttle: float = 0.05
    # Throttle для телеоперации (сервер читает из конфига и шлёт клиенту)
    forward_throttle: float = 0.43
    backward_throttle: float = -0.33
    forward_fast_throttle: float = 0.69
    pwm_min_pulse: int = 1000
    pwm_max_pulse: int = 2000


@dataclass
class WheelActuatorConfig:
    channel: int = 1
    min_throttle: float = -1.0
    zero_throttle: float = 0.15
    max_throttle: float = 0.92
    pwm_min_pulse: int = 1000
    pwm_max_pulse: int = 2000
    invert: bool = True


@dataclass
class HeadServoActuatorConfig:
    channel: int = 0
    angle_zero: float = 90.0
    actuation_range: int = 180
    pwm_min_pulse: int = 0
    pwm_max_pulse: int = 0


@dataclass
class ActuatorsConfig:
    backend: str = "mock"
    pca: PcaConfig = field(default_factory=PcaConfig)
    motor: MotorActuatorConfig = field(default_factory=MotorActuatorConfig)
    wheel: WheelActuatorConfig = field(default_factory=WheelActuatorConfig)
    neck: HeadServoActuatorConfig = field(default_factory=HeadServoActuatorConfig)
    face: HeadServoActuatorConfig = field(default_factory=HeadServoActuatorConfig)


@dataclass
class ClientConfig:
    app: AppConfig
    watchdog: WatchdogConfig
    diagnostics: DiagnosticsConfig
    robot_geometry: RobotGeometryConfig
    sensors: SensorsConfig
    network: NetworkConfig
    actuators: ActuatorsConfig


def client_config_to_dict(cfg: ClientConfig) -> dict:
    """Сериализация всего конфига клиента в dict для передачи на сервер (телеметрия)."""
    return asdict(cfg)


def _vec3(raw: dict | None, default: Vec3 | None = None) -> Vec3:
    default = default or Vec3()
    raw = raw or {}
    return Vec3(
        x=float(raw.get("x", default.x)),
        y=float(raw.get("y", default.y)),
        z=float(raw.get("z", default.z)),
    )


def _size3d(raw: dict | None, default: Size3D | None = None) -> Size3D:
    default = default or Size3D()
    raw = raw or {}
    return Size3D(
        length=float(raw.get("length", default.length)),
        width=float(raw.get("width", default.width)),
        height=float(raw.get("height", default.height)),
    )


def _fail_policy(raw: dict | None, default: SensorFailPolicy | None = None) -> SensorFailPolicy:
    default = default or SensorFailPolicy()
    raw = raw or {}
    return SensorFailPolicy(
        max_consecutive_failures=int(raw.get("max_consecutive_failures", default.max_consecutive_failures)),
        auto_disable_on_fail=bool(raw.get("auto_disable_on_fail", default.auto_disable_on_fail)),
        retry_interval_sec=float(raw.get("retry_interval_sec", default.retry_interval_sec)),
    )


def load_client_config(path: str) -> ClientConfig:
    with open(path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f) or {}
    app_raw = raw.get("app", {})
    wd_raw = raw.get("watchdog", {})
    diag_raw = raw.get("diagnostics", {})
    geom_raw = raw.get("robot_geometry", {})
    sensors_raw = raw.get("sensors", {})
    network_raw = raw.get("network", {})
    actuators_raw = raw.get("actuators", {})
    camera_raw = sensors_raw.get("camera", {})
    lidar_raw = sensors_raw.get("lidar", {})
    imu_raw = sensors_raw.get("imu", {})
    encoder_raw = sensors_raw.get("encoder", {})
    ultrasonic_raw = sensors_raw.get("ultrasonic", {})
    lidar_mount_raw = geom_raw.get("lidar", {})
    head_mount_raw = geom_raw.get("head", {})
    pca_raw = actuators_raw.get("pca", {})
    motor_raw = actuators_raw.get("motor", {})
    wheel_raw = actuators_raw.get("wheel", {})
    neck_raw = actuators_raw.get("neck", {})
    face_raw = actuators_raw.get("face", {})
    return ClientConfig(
        app=AppConfig(
            telemetry_hz=float(app_raw.get("telemetry_hz", 30.0)),
            command_hz=float(app_raw.get("command_hz", 100.0)),
        ),
        watchdog=WatchdogConfig(
            warn_ms=float(wd_raw.get("warn_ms", 50.0)),
            timeout_ms=float(wd_raw.get("timeout_ms", 100.0)),
            critical_ms=float(wd_raw.get("critical_ms", 500.0)),
        ),
        diagnostics=DiagnosticsConfig(
            battery_voltage=float(diag_raw.get("battery_voltage", 11.8)),
            cpu_temp_c=float(diag_raw.get("cpu_temp_c", 56.0)),
            wifi_rssi_dbm=float(diag_raw.get("wifi_rssi_dbm", -44.0)),
        ),
        robot_geometry=RobotGeometryConfig(
            body_size=_size3d(geom_raw.get("body_size"), Size3D(0.34, 0.24, 0.18)),
            pose_on_body=_vec3(geom_raw.get("pose_on_body"), Vec3(0.0, 0.0, 0.0)),
            wheel=WheelConfig(
                radius=float((geom_raw.get("wheel", {}) or {}).get("radius", 0.032)),
                width=float((geom_raw.get("wheel", {}) or {}).get("width", 0.026)),
            ),
            lidar=LidarMountConfig(
                position=_vec3(lidar_mount_raw.get("position"), Vec3(0.08, 0.0, 0.12)),
                blind_zone=BlindZoneConfig(
                    angle_start_deg=float((lidar_mount_raw.get("blind_zone", {}) or {}).get("angle_start_deg", -30.0)),
                    angle_end_deg=float((lidar_mount_raw.get("blind_zone", {}) or {}).get("angle_end_deg", 30.0)),
                    max_distance_m=float((lidar_mount_raw.get("blind_zone", {}) or {}).get("max_distance_m", 0.35)),
                ),
            ),
            head=HeadMountConfig(
                position=_vec3(head_mount_raw.get("position"), Vec3(0.12, 0.0, 0.17)),
                neck_min_deg=float(head_mount_raw.get("neck_min_deg", -70.0)),
                neck_max_deg=float(head_mount_raw.get("neck_max_deg", 70.0)),
                face_min_deg=float(head_mount_raw.get("face_min_deg", -45.0)),
                face_max_deg=float(head_mount_raw.get("face_max_deg", 45.0)),
            ),
        ),
        sensors=SensorsConfig(
            camera=CameraConfig(
                enabled=bool(camera_raw.get("enabled", False)),
                required_for_motion=bool(camera_raw.get("required_for_motion", False)),
                fps=float(camera_raw.get("fps", 30.0)),
                resolution=tuple(camera_raw.get("resolution", [640, 480])),
                tuning_file=str(camera_raw.get("tuning_file", "")),
                exposure_time=int(camera_raw.get("exposure_time", 8000)),
                analogue_gain=float(camera_raw.get("analogue_gain", 2.0)),
                awb_enable=bool(camera_raw.get("awb_enable", False)),
                ae_enable=bool(camera_raw.get("ae_enable", False)),
                jpeg_quality=int(camera_raw.get("jpeg_quality", 70)),
                healthcheck_timeout_sec=float(camera_raw.get("healthcheck_timeout_sec", 2.5)),
                healthcheck_test_timeout_sec=float(camera_raw.get("healthcheck_test_timeout_sec", 60.0)),
                healthcheck_min_frames=int(camera_raw.get("healthcheck_min_frames", 2)),
                fail_policy=_fail_policy(camera_raw.get("fail_policy")),
            ),
            lidar=LidarConfig(
                enabled=bool(lidar_raw.get("enabled", False)),
                required_for_motion=bool(lidar_raw.get("required_for_motion", True)),
                port=str(lidar_raw.get("port", "/dev/ydlidar")),
                auto_discover_port=bool(lidar_raw.get("auto_discover_port", True)),
                baudrate=int(lidar_raw.get("baudrate", 230400)),
                scan_hz=float(lidar_raw.get("scan_hz", 10.0)),
                min_angle_deg=float(lidar_raw.get("min_angle_deg", -180.0)),
                max_angle_deg=float(lidar_raw.get("max_angle_deg", 180.0)),
                range_min_m=float(lidar_raw.get("range_min_m", 0.02)),
                range_max_m=float(lidar_raw.get("range_max_m", 12.0)),
                intensity_enabled=bool(lidar_raw.get("intensity_enabled", True)),
                invert_angle=bool(lidar_raw.get("invert_angle", True)),
                zero_angle_deg=float(lidar_raw.get("zero_angle_deg", lidar_raw.get("zero_angle", -90.0))),
                healthcheck_timeout_sec=float(lidar_raw.get("healthcheck_timeout_sec", 3.0)),
                healthcheck_test_timeout_sec=float(lidar_raw.get("healthcheck_test_timeout_sec", 30.0)),
                healthcheck_min_points=int(lidar_raw.get("healthcheck_min_points", 32)),
                fail_policy=_fail_policy(lidar_raw.get("fail_policy")),
            ),
            imu=ImuConfig(
                enabled=bool(imu_raw.get("enabled", True)),
                hz=float(imu_raw.get("hz", 100.0)),
                required_for_motion=bool(imu_raw.get("required_for_motion", False)),
                fail_policy=_fail_policy(imu_raw.get("fail_policy")),
            ),
            encoder=EncoderConfig(
                enabled=bool(encoder_raw.get("enabled", True)),
                hz=float(encoder_raw.get("hz", 100.0)),
                required_for_motion=bool(encoder_raw.get("required_for_motion", False)),
                fail_policy=_fail_policy(encoder_raw.get("fail_policy")),
            ),
            ultrasonic=UltrasonicConfig(
                enabled=bool(ultrasonic_raw.get("enabled", True)),
                hz=float(ultrasonic_raw.get("hz", 10.0)),
                emergency_stop_distance_m=float(ultrasonic_raw.get("emergency_stop_distance_m", 0.2)),
                required_for_motion=bool(ultrasonic_raw.get("required_for_motion", True)),
                fail_policy=_fail_policy(ultrasonic_raw.get("fail_policy")),
            ),
        ),
        network=NetworkConfig(
            server_host=str(network_raw.get("server_host", "127.0.0.1")),
            telemetry_port=int(network_raw.get("telemetry_port", 5550)),
            command_port=int(network_raw.get("command_port", 5552)),
            report_port=int(network_raw.get("report_port", 5553)),
            recv_timeout_ms=int(network_raw.get("recv_timeout_ms", 0)),
            send_high_water_mark=int(network_raw.get("send_high_water_mark", 2)),
            recv_high_water_mark=int(network_raw.get("recv_high_water_mark", 1)),
        ),
        actuators=ActuatorsConfig(
            backend=str(actuators_raw.get("backend", "mock")),
            pca=PcaConfig(
                channels=int(pca_raw.get("channels", 16)),
                frequency=int(pca_raw.get("frequency", 250)),
            ),
            motor=MotorActuatorConfig(
                channel=int(motor_raw.get("channel", 0)),
                zero_throttle=float(motor_raw.get("zero_throttle", 0.05)),
                forward_throttle=float(motor_raw.get("forward_throttle", 0.43)),
                backward_throttle=float(motor_raw.get("backward_throttle", -0.33)),
                forward_fast_throttle=float(motor_raw.get("forward_fast_throttle", 0.69)),
                pwm_min_pulse=int(motor_raw.get("pwm_min_pulse", 1000)),
                pwm_max_pulse=int(motor_raw.get("pwm_max_pulse", 2000)),
            ),
            wheel=WheelActuatorConfig(
                channel=int(wheel_raw.get("channel", 1)),
                min_throttle=float(wheel_raw.get("min_throttle", -1.0)),
                zero_throttle=float(wheel_raw.get("zero_throttle", 0.15)),
                max_throttle=float(wheel_raw.get("max_throttle", 0.92)),
                pwm_min_pulse=int(wheel_raw.get("pwm_min_pulse", 1000)),
                pwm_max_pulse=int(wheel_raw.get("pwm_max_pulse", 2000)),
                invert=bool(wheel_raw.get("invert", True)),
            ),
            neck=HeadServoActuatorConfig(
                channel=int(neck_raw.get("channel", 3)),
                angle_zero=float(neck_raw.get("angle_zero", 135.0)),
                actuation_range=int(neck_raw.get("actuation_range", 360)),
                pwm_min_pulse=int(neck_raw.get("pwm_min_pulse", 300)),
                pwm_max_pulse=int(neck_raw.get("pwm_max_pulse", 1800)),
            ),
            face=HeadServoActuatorConfig(
                channel=int(face_raw.get("channel", 2)),
                angle_zero=float(face_raw.get("angle_zero", 15.0)),
                actuation_range=int(face_raw.get("actuation_range", 180)),
                pwm_min_pulse=int(face_raw.get("pwm_min_pulse", 0)),
                pwm_max_pulse=int(face_raw.get("pwm_max_pulse", 0)),
            ),
        ),
    )

