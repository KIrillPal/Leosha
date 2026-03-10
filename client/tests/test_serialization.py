from __future__ import annotations

from client.models import (
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
from client.network.serialization import pack_server_packet, unpack_server_packet


def test_pack_unpack_server_packet_teleoperation():
    packet = ServerPacket(
        header=ServerPacketHeader(seq=1, timestamp_ns=123, mode=OperatingMode.TELEOPERATION),
        teleop_cmd=TeleoperationCommand(speed=0.5, steering=-0.2, head_pan=0.1, head_tilt=-0.1),
    )
    raw = pack_server_packet(packet)
    decoded = unpack_server_packet(raw)
    assert decoded.header.mode == OperatingMode.TELEOPERATION
    assert decoded.teleop_cmd is not None
    assert decoded.teleop_cmd.speed == 0.5


def test_pack_unpack_server_packet_autonomy():
    packet = ServerPacket(
        header=ServerPacketHeader(seq=2, timestamp_ns=456, mode=OperatingMode.AUTONOMY_PROFILE_1),
        autonomy_cmd=AutonomyCommand(
            fused_pose=Pose2D(1.0, 2.0, 0.2),
            fused_velocity=Twist2D(0.4, 0.1),
            trajectory=[TrajectoryPoint(1, 1.0, 2.0, 0.2, 0.3, 0.01)],
            head_pan=0.0,
            head_tilt=-0.1,
            dynamic_obstacles=[DynamicObstacle(1, "person", 2.0, 0.5, 0.1, 0.0, 0.3)],
            map_update=MapUpdate(
                is_full=False,
                resolution=0.05,
                origin=Pose2D(),
                width=10,
                height=10,
                changed_cells=[(1, 50)],
            ),
        ),
    )
    raw = pack_server_packet(packet)
    decoded = unpack_server_packet(raw)
    assert decoded.header.mode == OperatingMode.AUTONOMY_PROFILE_1
    assert decoded.autonomy_cmd is not None
    assert len(decoded.autonomy_cmd.trajectory) == 1
    assert decoded.autonomy_cmd.map_update is not None

