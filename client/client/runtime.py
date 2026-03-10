from __future__ import annotations

import logging
import threading
from time import monotonic_ns, sleep

from .config import client_config_to_dict
from .estimator import SimpleStateEstimator
from .executors import AutonomyExecutor, PauseExecutor, TeleoperationExecutor
from .local_state import LocalState
from .models import (
    ImuReading,
    OperatingMode,
    Pose2D,
    RobotStatus,
    TelemetryPacket,
    Twist2D,
    WheelOdometry,
)
from .sensor_hub import SensorHub
from .sensors import CameraSensorThread, SensorStatusRegistry, TMiniProPlusLidarThread
from .stats import StatsCollector
from .watchdog import Watchdog

LOGGER = logging.getLogger(__name__)


class ClientRuntime:
    """Оркестратор потоков клиента: telemetry 30Hz + command 100Hz."""

    def __init__(self, bridge, actuator_driver, config) -> None:
        self.bridge = bridge
        self.actuators = actuator_driver
        self.config = config
        self.sensor_hub = SensorHub()
        self.stats = StatsCollector()
        self.state = LocalState()
        self.watchdog = Watchdog(
            warn_ms=config.watchdog.warn_ms,
            timeout_ms=config.watchdog.timeout_ms,
            critical_ms=config.watchdog.critical_ms,
        )
        self.sensor_statuses = SensorStatusRegistry()
        self.estimator = SimpleStateEstimator()
        self._seq = 0
        self._lock = threading.Lock()
        self._last_gyro_z = 0.0
        self._last_status: RobotStatus | None = None
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        head_cfg = config.robot_geometry.head
        teleop_exec = TeleoperationExecutor(
            head_pan_min_deg=head_cfg.neck_min_deg,
            head_pan_max_deg=head_cfg.neck_max_deg,
            head_tilt_min_deg=head_cfg.face_min_deg,
            head_tilt_max_deg=head_cfg.face_max_deg,
        )
        self._executors = {
            OperatingMode.PAUSE: PauseExecutor(),
            OperatingMode.TELEOPERATION: teleop_exec,
            OperatingMode.TELEOP_SLAM: teleop_exec,
            OperatingMode.AUTONOMY_PROFILE_1: AutonomyExecutor(
                head_pan_min_deg=head_cfg.neck_min_deg,
                head_pan_max_deg=head_cfg.neck_max_deg,
                head_tilt_min_deg=head_cfg.face_min_deg,
                head_tilt_max_deg=head_cfg.face_max_deg,
            ),
        }
        for name, hz in {
            "camera": config.sensors.camera.fps,
            "lidar": config.sensors.lidar.scan_hz,
            "imu": config.sensors.imu.hz,
            "encoders": config.sensors.encoder.hz,
            "ultrasonic": config.sensors.ultrasonic.hz,
            "telemetry": config.app.telemetry_hz,
            "command": config.app.command_hz,
        }.items():
            self.stats.register(name, hz)
        for name, enabled in {
            "camera": config.sensors.camera.enabled,
            "lidar": config.sensors.lidar.enabled,
            "imu": config.sensors.imu.enabled,
            "encoders": config.sensors.encoder.enabled,
            "ultrasonic": config.sensors.ultrasonic.enabled,
        }.items():
            self.sensor_statuses.register(name, enabled)
        self._seed_sensor_defaults()

    def _seed_sensor_defaults(self) -> None:
        now = monotonic_ns()
        self.sensor_hub.put_frame(b"\xff\xd8\xff\xd9", now)
        odom = WheelOdometry(
            timestamp_ns=now,
            pose=Pose2D(),
            velocity=Twist2D(),
            steering_angle=0.0,
        )
        self.sensor_hub.put_odometry(odom)

    def start(self) -> None:
        if self._threads:
            LOGGER.warning("ClientRuntime.start() ignored: threads already running")
            return
        self._stop.clear()
        self._threads = [
            CameraSensorThread(
                camera_cfg=self.config.sensors.camera,
                sensor_hub=self.sensor_hub,
                stats=self.stats,
                statuses=self.sensor_statuses,
                stop_event=self._stop,
            ),
            TMiniProPlusLidarThread(
                lidar_cfg=self.config.sensors.lidar,
                geometry_cfg=self.config.robot_geometry,
                sensor_hub=self.sensor_hub,
                stats=self.stats,
                statuses=self.sensor_statuses,
                stop_event=self._stop,
            ),
            threading.Thread(target=self._mock_i2c_loop, daemon=True),
            threading.Thread(target=self._mock_ultrasonic_loop, daemon=True),
            threading.Thread(target=self._telemetry_loop, daemon=True),
            threading.Thread(target=self._command_loop, daemon=True),
        ]
        for t in self._threads:
            t.start()
        LOGGER.info("Client runtime started with %d worker threads", len(self._threads))

    def stop(self) -> None:
        self._stop.set()
        for t in self._threads:
            t.join(timeout=1.0)
        self._threads.clear()
        close_fn = getattr(self.bridge, "close", None)
        if callable(close_fn):
            close_fn()
        LOGGER.info("Client runtime stopped")

    def _mock_i2c_loop(self) -> None:
        period = 1.0 / max(1.0, self.config.sensors.imu.hz)
        pose = Pose2D()
        while not self._stop.is_set():
            t0 = monotonic_ns()
            read_start = monotonic_ns()
            if self.config.sensors.imu.enabled:
                imu = ImuReading(
                    timestamp_ns=read_start,
                    accel_x=0.0,
                    accel_y=0.0,
                    accel_z=9.81,
                    gyro_x=0.0,
                    gyro_y=0.0,
                    gyro_z=self._last_gyro_z,
                )
                self.sensor_hub.append_imu(imu)
                self.sensor_statuses.mark_ok("imu")
                self.stats.record("imu", monotonic_ns() - read_start, monotonic_ns() - t0)
            feedback = self.actuators.get_feedback_state()
            speed = feedback.motor_throttle * 0.6
            steering = feedback.steering_throttle * 0.5
            if self.config.sensors.encoder.enabled:
                wheelbase = max(0.05, self.config.robot_geometry.body_size.length * 0.6)
                pose = self.estimator.integrate_ackermann(pose, speed, steering, wheelbase, period)
                odom = WheelOdometry(
                    timestamp_ns=monotonic_ns(),
                    pose=pose,
                    velocity=Twist2D(linear=speed, angular=steering),
                    steering_angle=steering,
                )
                self.sensor_hub.put_odometry(odom)
                self.sensor_statuses.mark_ok("encoders")
                self.stats.record("encoders", monotonic_ns() - read_start, monotonic_ns() - t0)
                est = self.estimator.update(odom, latest_gyro_z=self._last_gyro_z)
                self.state.merge_local_pose(est.pose, est.velocity)
            cycle_end = monotonic_ns()
            sleep(max(0.0, period - (cycle_end - t0) / 1e9))

    def _mock_ultrasonic_loop(self) -> None:
        period = 1.0 / max(1.0, self.config.sensors.ultrasonic.hz)
        if not self.config.sensors.ultrasonic.enabled:
            self.sensor_statuses.mark_disabled("ultrasonic", "disabled_in_config")
            LOGGER.info("Ultrasonic loop disabled by config")
            return
        while not self._stop.is_set():
            t0 = monotonic_ns()
            read_start = monotonic_ns()
            self.sensor_hub.put_ultrasonic(1.4, monotonic_ns())
            self.sensor_statuses.mark_ok("ultrasonic")
            read_end = monotonic_ns()
            self.stats.record("ultrasonic", read_end - read_start, read_end - t0)
            sleep(max(0.0, period - (read_end - t0) / 1e9))

    def _telemetry_loop(self) -> None:
        period = 1.0 / self.config.app.telemetry_hz
        while not self._stop.is_set():
            t0 = monotonic_ns()
            snapshot = self.sensor_hub.snapshot()
            mode = self.state.snapshot()["mode"]
            status = self.state.snapshot()["status"]
            odom = snapshot.odometry
            if odom is None:
                odom = WheelOdometry(
                    timestamp_ns=snapshot.captured_at_ns,
                    pose=Pose2D(),
                    velocity=Twist2D(),
                    steering_angle=0.0,
                )
            robot_config = client_config_to_dict(self.config)
            packet = TelemetryPacket(
                seq=self._next_seq(),
                timestamp_ns=snapshot.captured_at_ns,
                mode=mode,
                status=status,
                frame_jpeg=snapshot.frame_jpeg,
                scan=snapshot.scan,
                imu_readings=snapshot.imu_readings,
                odometry=odom,
                ultrasonic_range_m=snapshot.ultrasonic_range_m,
                battery_voltage=self.config.diagnostics.battery_voltage,
                cpu_temp_c=self.config.diagnostics.cpu_temp_c,
                wifi_rssi_dbm=self.config.diagnostics.wifi_rssi_dbm,
                sensor_status=self.sensor_statuses.snapshot(),
                sensor_timing=self.stats.drain_all(),
                telemetry_pack_us=(monotonic_ns() - t0) / 1000.0,
                robot_config=robot_config,
            )
            send_start = monotonic_ns()
            self.bridge.send_telemetry(packet)
            packet.telemetry_send_us = (monotonic_ns() - send_start) / 1000.0
            self.stats.record("telemetry", monotonic_ns() - t0, monotonic_ns() - t0)
            sleep(max(0.0, period - (monotonic_ns() - t0) / 1e9))

    def _command_loop(self) -> None:
        period = 1.0 / self.config.app.command_hz
        rx_cmd_count = 0
        while not self._stop.is_set():
            t0 = monotonic_ns()
            packet = self.bridge.recv_packet()
            if packet is not None:
                rx_cmd_count += 1
                self.watchdog.feed()
                self.state.merge_server_packet(packet)
                if packet.teleop_cmd:
                    self._last_gyro_z = packet.teleop_cmd.steering * packet.teleop_cmd.speed
                    if rx_cmd_count <= 5 or rx_cmd_count % 300 == 0:
                        LOGGER.info(
                            "CMD RX #%d | seq=%d mode=%s speed=%.3f steer=%.3f",
                            rx_cmd_count, packet.header.seq, packet.header.mode.value,
                            packet.teleop_cmd.speed, packet.teleop_cmd.steering,
                        )
                elif rx_cmd_count <= 5:
                    LOGGER.info(
                        "CMD RX #%d | seq=%d mode=%s (no teleop_cmd)",
                        rx_cmd_count, packet.header.seq, packet.header.mode.value,
                    )
            status = self.watchdog.status()
            if self._must_stop_by_sensor_status():
                status = RobotStatus.EMERGENCY_STOP
            if status != self._last_status:
                LOGGER.info("Client runtime status changed: %s -> %s", self._last_status, status)
                if status == RobotStatus.EMERGENCY_STOP:
                    LOGGER.warning("Emergency stop active (watchdog or required sensors)")
                self._last_status = status
            self.state.set_status(status)
            if status in (RobotStatus.WAITING_FOR_SERVER, RobotStatus.EMERGENCY_STOP):
                self.actuators.emergency_stop()
                self.bridge.send_missing_packet_report(self.watchdog.elapsed_ms())
            else:
                state_snapshot = self.state.snapshot()
                mode = state_snapshot["mode"]
                executor = self._executors[mode]
                cmd = executor.compute(state_snapshot)
                if status == RobotStatus.DEGRADED:
                    cmd.motor_throttle *= self.watchdog.deceleration_factor()
                self.actuators.apply(cmd)
            self.stats.record("command", monotonic_ns() - t0, monotonic_ns() - t0)
            sleep(max(0.0, period - (monotonic_ns() - t0) / 1e9))

    def _next_seq(self) -> int:
        with self._lock:
            self._seq += 1
            return self._seq

    def _must_stop_by_sensor_status(self) -> bool:
        snapshot = self.sensor_statuses.snapshot()
        mode = self.state.snapshot()["mode"]
        required = []
        if self.config.sensors.lidar.required_for_motion and mode == OperatingMode.AUTONOMY_PROFILE_1:
            required.append("lidar")
        if self.config.sensors.ultrasonic.required_for_motion:
            required.append("ultrasonic")
        if self.config.sensors.camera.required_for_motion and mode == OperatingMode.AUTONOMY_PROFILE_1:
            required.append("camera")
        for name in required:
            status = snapshot.get(name)
            if status and (not status.enabled or not status.healthy):
                return True
        return False

