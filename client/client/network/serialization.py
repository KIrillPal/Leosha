from __future__ import annotations

import msgpack

from ..models import (
    AutonomyCommand,
    DynamicObstacle,
    ImuReading,
    LaserScan,
    MapUpdate,
    OperatingMode,
    Pose2D,
    SensorStatus,
    SensorTimingStats,
    ServerPacket,
    ServerPacketHeader,
    TelemetryPacket,
    TeleoperationCommand,
    TrajectoryPoint,
    Twist2D,
    WheelOdometry,
)


def pack_server_packet(packet: ServerPacket) -> bytes:
    payload = {
        "header": {
            "seq": packet.header.seq,
            "timestamp_ns": packet.header.timestamp_ns,
            "mode": packet.header.mode.value,
        },
        "teleop_cmd": None,
        "autonomy_cmd": None,
    }
    if packet.teleop_cmd:
        payload["teleop_cmd"] = {
            "speed": packet.teleop_cmd.speed,
            "steering": packet.teleop_cmd.steering,
            "head_pan": packet.teleop_cmd.head_pan,
            "head_tilt": packet.teleop_cmd.head_tilt,
        }
    if packet.autonomy_cmd:
        payload["autonomy_cmd"] = {
            "fused_pose": _pack_pose(packet.autonomy_cmd.fused_pose),
            "fused_velocity": _pack_twist(packet.autonomy_cmd.fused_velocity),
            "trajectory": [_pack_trajectory_point(p) for p in packet.autonomy_cmd.trajectory],
            "head_pan": packet.autonomy_cmd.head_pan,
            "head_tilt": packet.autonomy_cmd.head_tilt,
            "dynamic_obstacles": [_pack_obstacle(o) for o in packet.autonomy_cmd.dynamic_obstacles],
            "map_update": _pack_map_update(packet.autonomy_cmd.map_update),
        }
    return msgpack.packb(payload, use_bin_type=True)


def unpack_server_packet(raw: bytes) -> ServerPacket:
    payload = msgpack.unpackb(raw, raw=False)
    header = ServerPacketHeader(
        seq=int(payload["header"]["seq"]),
        timestamp_ns=int(payload["header"]["timestamp_ns"]),
        mode=OperatingMode(payload["header"]["mode"]),
    )
    teleop = None
    autonomy = None
    if payload.get("teleop_cmd"):
        t = payload["teleop_cmd"]
        teleop = TeleoperationCommand(
            speed=float(t["speed"]),
            steering=float(t["steering"]),
            head_pan=float(t["head_pan"]),
            head_tilt=float(t["head_tilt"]),
        )
    if payload.get("autonomy_cmd"):
        a = payload["autonomy_cmd"]
        autonomy = AutonomyCommand(
            fused_pose=_unpack_pose(a["fused_pose"]),
            fused_velocity=_unpack_twist(a["fused_velocity"]),
            trajectory=[_unpack_trajectory_point(p) for p in a["trajectory"]],
            head_pan=float(a["head_pan"]),
            head_tilt=float(a["head_tilt"]),
            dynamic_obstacles=[_unpack_obstacle(o) for o in a["dynamic_obstacles"]],
            map_update=_unpack_map_update(a["map_update"]),
        )
    return ServerPacket(header=header, teleop_cmd=teleop, autonomy_cmd=autonomy)


def _pack_pose(pose: Pose2D) -> dict:
    return {"x": pose.x, "y": pose.y, "theta": pose.theta}


def _unpack_pose(data: dict) -> Pose2D:
    return Pose2D(x=float(data["x"]), y=float(data["y"]), theta=float(data["theta"]))


def _pack_twist(twist: Twist2D) -> dict:
    return {"linear": twist.linear, "angular": twist.angular}


def _unpack_twist(data: dict) -> Twist2D:
    return Twist2D(linear=float(data["linear"]), angular=float(data["angular"]))


def _pack_trajectory_point(point: TrajectoryPoint) -> dict:
    return {
        "timestamp_ns": point.timestamp_ns,
        "x": point.x,
        "y": point.y,
        "theta": point.theta,
        "target_speed": point.target_speed,
        "curvature": point.curvature,
    }


def _unpack_trajectory_point(data: dict) -> TrajectoryPoint:
    return TrajectoryPoint(
        timestamp_ns=int(data["timestamp_ns"]),
        x=float(data["x"]),
        y=float(data["y"]),
        theta=float(data["theta"]),
        target_speed=float(data["target_speed"]),
        curvature=float(data["curvature"]),
    )


def _pack_obstacle(obs: DynamicObstacle) -> dict:
    return {
        "object_id": obs.object_id,
        "class_name": obs.class_name,
        "x": obs.x,
        "y": obs.y,
        "vx": obs.vx,
        "vy": obs.vy,
        "radius": obs.radius,
    }


def _unpack_obstacle(data: dict) -> DynamicObstacle:
    return DynamicObstacle(
        object_id=int(data["object_id"]),
        class_name=str(data["class_name"]),
        x=float(data["x"]),
        y=float(data["y"]),
        vx=float(data["vx"]),
        vy=float(data["vy"]),
        radius=float(data["radius"]),
    )


def _pack_map_update(update: MapUpdate | None) -> dict | None:
    if update is None:
        return None
    return {
        "is_full": update.is_full,
        "resolution": update.resolution,
        "origin": _pack_pose(update.origin),
        "width": update.width,
        "height": update.height,
        "data": update.data,
        "changed_cells": update.changed_cells,
    }


def _unpack_map_update(data: dict | None) -> MapUpdate | None:
    if data is None:
        return None
    return MapUpdate(
        is_full=bool(data["is_full"]),
        resolution=float(data["resolution"]),
        origin=_unpack_pose(data["origin"]),
        width=int(data["width"]),
        height=int(data["height"]),
        data=bytes(data.get("data", b"")),
        changed_cells=data.get("changed_cells"),
    )


def pack_scan(scan: LaserScan | None) -> dict | None:
    if scan is None:
        return None
    return {
        "timestamp_ns": scan.timestamp_ns,
        "angle_min": scan.angle_min,
        "angle_max": scan.angle_max,
        "angle_increment": scan.angle_increment,
        "range_min": scan.range_min,
        "range_max": scan.range_max,
        "ranges": scan.ranges,
        "intensities": scan.intensities,
    }


def pack_telemetry_packet(packet: TelemetryPacket) -> tuple[bytes, bytes]:
    """Упаковка телеметрии в multipart: header(msgpack) + jpeg bytes."""
    scan_data = pack_scan(packet.scan)
    imu_data = [_pack_imu(i) for i in packet.imu_readings]
    header = {
        "seq": packet.seq,
        "timestamp_ns": packet.timestamp_ns,
        "mode": packet.mode.value,
        "status": packet.status.value,
        "scan": scan_data,
        "imu_readings": imu_data,
        "odometry": _pack_odometry(packet.odometry),
        "ultrasonic_range_m": packet.ultrasonic_range_m,
        "battery_voltage": packet.battery_voltage,
        "cpu_temp_c": packet.cpu_temp_c,
        "wifi_rssi_dbm": packet.wifi_rssi_dbm,
        "sensor_status": {k: _pack_sensor_status(v) for k, v in packet.sensor_status.items()},
        "sensor_timing": {k: _pack_sensor_timing(v) for k, v in packet.sensor_timing.items()},
        "telemetry_pack_us": packet.telemetry_pack_us,
        "telemetry_send_us": packet.telemetry_send_us,
        "frame_size": len(packet.frame_jpeg),
    }
    if packet.robot_config is not None:
        header["robot_config"] = packet.robot_config
    # Размеры частей пакета для отображения на сервере (среднее по пакетам)
    header["parts_bytes"] = {
        "lidar": len(msgpack.packb(scan_data, use_bin_type=True)) if scan_data else 0,
        "imu": len(msgpack.packb(imu_data, use_bin_type=True)),
        "config": len(msgpack.packb(header["robot_config"], use_bin_type=True)) if packet.robot_config else 0,
        "camera": len(packet.frame_jpeg),
    }
    return msgpack.packb(header, use_bin_type=True), packet.frame_jpeg


def pack_missing_report(elapsed_ms: float, timestamp_ns: int) -> bytes:
    return msgpack.packb(
        {
            "type": "missing_packet_report",
            "elapsed_ms": float(elapsed_ms),
            "timestamp_ns": int(timestamp_ns),
        },
        use_bin_type=True,
    )


def _pack_imu(imu: ImuReading) -> dict:
    return {
        "timestamp_ns": imu.timestamp_ns,
        "accel_x": imu.accel_x,
        "accel_y": imu.accel_y,
        "accel_z": imu.accel_z,
        "gyro_x": imu.gyro_x,
        "gyro_y": imu.gyro_y,
        "gyro_z": imu.gyro_z,
    }


def _pack_odometry(odom: WheelOdometry) -> dict:
    return {
        "timestamp_ns": odom.timestamp_ns,
        "pose": _pack_pose(odom.pose),
        "velocity": _pack_twist(odom.velocity),
        "steering_angle": odom.steering_angle,
    }


def _pack_sensor_status(status: SensorStatus) -> dict:
    return {
        "sensor_name": status.sensor_name,
        "enabled": status.enabled,
        "active": status.active,
        "healthy": status.healthy,
        "failure_count": status.failure_count,
        "last_update_ns": status.last_update_ns,
        "last_error": status.last_error,
        "disabled_reason": status.disabled_reason,
    }


def _pack_sensor_timing(timing: SensorTimingStats) -> dict:
    return {
        "sensor_name": timing.sensor_name,
        "read_min_us": timing.read_min_us,
        "read_max_us": timing.read_max_us,
        "read_avg_us": timing.read_avg_us,
        "read_last_us": timing.read_last_us,
        "cycle_min_us": timing.cycle_min_us,
        "cycle_max_us": timing.cycle_max_us,
        "cycle_avg_us": timing.cycle_avg_us,
        "cycle_count": timing.cycle_count,
        "overrun_count": timing.overrun_count,
        "error_count": timing.error_count,
        "actual_hz": timing.actual_hz,
        "target_hz": timing.target_hz,
    }

