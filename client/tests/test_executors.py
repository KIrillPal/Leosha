from __future__ import annotations

from client.executors import AutonomyExecutor, PauseExecutor, TeleoperationExecutor
from client.models import (
    AutonomyCommand,
    DynamicObstacle,
    Pose2D,
    TeleoperationCommand,
    TrajectoryPoint,
    Twist2D,
)


def test_pause_executor_returns_zero_command():
    cmd = PauseExecutor().compute({})
    assert cmd.motor_throttle == 0.0
    assert cmd.steering_throttle == 0.0


def test_teleoperation_executor_uses_server_command():
    executor = TeleoperationExecutor()
    cmd = executor.compute({"teleop_cmd": TeleoperationCommand(0.4, -0.3, 0.2, -0.1)})
    assert cmd.motor_throttle == 0.4
    assert cmd.steering_throttle == -0.3
    assert cmd.head_pan_angle != 0.0


def test_autonomy_executor_follows_trajectory():
    executor = AutonomyExecutor()
    state = {
        "pose": Pose2D(0.0, 0.0, 0.0),
        "trajectory": [TrajectoryPoint(1, 1.0, 0.2, 0.0, 0.35, 0.01)],
        "autonomy_cmd": AutonomyCommand(
            fused_pose=Pose2D(0.0, 0.0, 0.0),
            fused_velocity=Twist2D(0.0, 0.0),
            trajectory=[],
            head_pan=0.0,
            head_tilt=0.0,
            dynamic_obstacles=[DynamicObstacle(1, "person", 2.0, 0.0, 0.1, 0.0, 0.3)],
        ),
    }
    cmd = executor.compute(state)
    assert cmd.motor_throttle > 0.0

