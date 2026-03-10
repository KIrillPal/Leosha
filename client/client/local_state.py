from __future__ import annotations

import threading
from dataclasses import dataclass, field

from .models import (
    AutonomyCommand,
    DynamicObstacle,
    MapUpdate,
    OperatingMode,
    Pose2D,
    RobotStatus,
    ServerPacket,
    TeleoperationCommand,
    TrajectoryPoint,
    Twist2D,
)


@dataclass
class LocalStateData:
    mode: OperatingMode = OperatingMode.PAUSE
    status: RobotStatus = RobotStatus.WAITING_FOR_SERVER
    pose: Pose2D = field(default_factory=Pose2D)
    velocity: Twist2D = field(default_factory=Twist2D)
    map_update: MapUpdate | None = None
    trajectory: list[TrajectoryPoint] = field(default_factory=list)
    dynamic_obstacles: list[DynamicObstacle] = field(default_factory=list)
    teleop_cmd: TeleoperationCommand | None = None
    autonomy_cmd: AutonomyCommand | None = None
    last_server_seq: int = -1


class LocalState:
    """Состояние робота, объединяющее локальные и серверные данные."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._data = LocalStateData()

    def set_status(self, status: RobotStatus) -> None:
        with self._lock:
            self._data.status = status

    def merge_server_packet(self, packet: ServerPacket) -> None:
        with self._lock:
            self._data.mode = packet.header.mode
            self._data.last_server_seq = packet.header.seq
            self._data.teleop_cmd = packet.teleop_cmd
            self._data.autonomy_cmd = packet.autonomy_cmd
            if packet.autonomy_cmd:
                self._data.pose = packet.autonomy_cmd.fused_pose
                self._data.velocity = packet.autonomy_cmd.fused_velocity
                self._data.trajectory = list(packet.autonomy_cmd.trajectory)
                self._data.dynamic_obstacles = list(packet.autonomy_cmd.dynamic_obstacles)
                if packet.autonomy_cmd.map_update:
                    self._data.map_update = packet.autonomy_cmd.map_update
            elif packet.teleop_cmd:
                self._data.trajectory = []
                self._data.dynamic_obstacles = []

    def merge_local_pose(self, pose: Pose2D, velocity: Twist2D) -> None:
        with self._lock:
            self._data.pose = pose
            self._data.velocity = velocity

    def snapshot(self) -> dict:
        with self._lock:
            data = self._data
            return {
                "mode": data.mode,
                "status": data.status,
                "pose": Pose2D(data.pose.x, data.pose.y, data.pose.theta),
                "velocity": Twist2D(data.velocity.linear, data.velocity.angular),
                "trajectory": list(data.trajectory),
                "dynamic_obstacles": list(data.dynamic_obstacles),
                "teleop_cmd": data.teleop_cmd,
                "autonomy_cmd": data.autonomy_cmd,
                "last_server_seq": data.last_server_seq,
                "map_update": data.map_update,
            }

