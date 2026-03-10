from __future__ import annotations

import math
from dataclasses import dataclass
from time import monotonic_ns

from ..models import (
    AutonomyCommand,
    DynamicObstacle,
    MapUpdate,
    OperatingMode,
    Pose2D,
    ServerPacket,
    ServerPacketHeader,
    TeleoperationCommand,
    TrajectoryPoint,
    Twist2D,
)


@dataclass
class MockServerConfig:
    mode: OperatingMode = OperatingMode.TELEOPERATION
    drop_every_nth: int = 0
    speed: float = 0.3
    steering: float = 0.2


class MockServer:
    """Убедительный mock сервера: режимы, периодическая потеря пакетов, траектория."""

    def __init__(self, config: MockServerConfig | None = None) -> None:
        self._config = config or MockServerConfig()
        self._seq = 0
        self._telemetry_count = 0
        self._missing_reports: list[float] = []

    @property
    def missing_reports(self) -> list[float]:
        return list(self._missing_reports)

    def set_mode(self, mode: OperatingMode) -> None:
        self._config.mode = mode

    def receive_telemetry(self, packet) -> None:
        self._telemetry_count += 1

    def create_packet(self) -> ServerPacket | None:
        self._seq += 1
        if self._config.drop_every_nth > 0 and self._seq % self._config.drop_every_nth == 0:
            return None
        header = ServerPacketHeader(
            seq=self._seq,
            timestamp_ns=monotonic_ns(),
            mode=self._config.mode,
        )
        if self._config.mode == OperatingMode.TELEOPERATION:
            cmd = TeleoperationCommand(
                speed=self._config.speed,
                steering=self._config.steering,
                head_pan=0.1 * math.sin(self._seq * 0.03),
                head_tilt=0.08 * math.cos(self._seq * 0.02),
            )
            return ServerPacket(header=header, teleop_cmd=cmd)
        if self._config.mode == OperatingMode.AUTONOMY_PROFILE_1:
            return ServerPacket(header=header, autonomy_cmd=self._build_autonomy())
        return ServerPacket(header=header)

    def report_missing(self, elapsed_ms: float) -> None:
        self._missing_reports.append(float(elapsed_ms))

    def _build_autonomy(self) -> AutonomyCommand:
        now = monotonic_ns()
        trajectory = []
        for i in range(15):
            t = now + int(i * 40_000_000)
            x = 0.2 * i
            y = 0.08 * math.sin(i * 0.4)
            trajectory.append(
                TrajectoryPoint(
                    timestamp_ns=t,
                    x=x,
                    y=y,
                    theta=0.03 * i,
                    target_speed=0.35,
                    curvature=0.02,
                )
            )
        obstacles = [
            DynamicObstacle(
                object_id=1,
                class_name="person",
                x=2.4,
                y=0.7,
                vx=0.1,
                vy=0.0,
                radius=0.35,
            )
        ]
        map_update = MapUpdate(
            is_full=False,
            resolution=0.05,
            origin=Pose2D(-5.0, -5.0, 0.0),
            width=200,
            height=200,
            changed_cells=[(101, 100), (102, 100), (103, 95)],
        )
        return AutonomyCommand(
            fused_pose=Pose2D(1.2, 0.4, 0.15),
            fused_velocity=Twist2D(0.35, 0.03),
            trajectory=trajectory,
            head_pan=0.0,
            head_tilt=-0.08,
            dynamic_obstacles=obstacles,
            map_update=map_update,
        )

