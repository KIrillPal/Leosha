"""Start every LEO node. The launch file passes the config path and nothing else."""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_config() -> str:
    candidate = Path(__file__).resolve().parents[3] / "config" / "robot.yaml"
    if candidate.is_file():
        return str(candidate)
    return os.environ.get("LEO_CONFIG", "")


def _quat_z(yaw: float) -> tuple[str, str, str, str]:
    return ("0", "0", str(math.sin(yaw / 2.0)), str(math.cos(yaw / 2.0)))


def _static_tf(x, y, z, yaw, parent, child) -> Node:
    qx, qy, qz, qw = _quat_z(yaw)
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"tf_{child}",
        arguments=[str(x), str(y), str(z), qx, qy, qz, qw, parent, child],
    )


def _nodes(context, *args, **kwargs):
    del args, kwargs
    config_path = LaunchConfiguration("config").perform(context)
    sim = LaunchConfiguration("sim").perform(context)
    profile = LaunchConfiguration("profile").perform(context) == "true"
    root = Path(config_path).resolve().parent.parent
    logs = root / "logs"
    logs.mkdir(parents=True, exist_ok=True)
    sys.path[:0] = [str(root / "src" / "server"), str(root / "src" / "shared")]
    from leo_server.common.config import load_config

    config = load_config(config_path)
    names = ("robot_bridge", "mode_manager", "odometry", "vision", "web_ui", "monitor")
    actions = []
    for name in names:
        kwargs = {}
        if profile:
            kwargs["prefix"] = f"{sys.executable} -m cProfile -o {logs / (name + '.prof')}"
        actions.append(
            Node(
                package="leo_server",
                executable=name,
                name=name,
                arguments=["--config", config_path],
                output="own_log",
                additional_env={"LEO_CONFIG": config_path, "PYTHONPATH": os.pathsep.join([str(root / "src" / "server"), str(root / "src" / "shared"), os.environ.get("PYTHONPATH", "")])},
                **kwargs,
            )
        )
    geometry = config.geometry
    actions.append(_static_tf(geometry["lidar_x"], geometry["lidar_y"], geometry["lidar_z"], geometry["lidar_yaw"], "base_link", "laser"))
    actions.append(_static_tf(geometry["head_x"], geometry["head_y"], geometry["head_z"], 0.0, "base_link", "head_base"))
    if sim:
        python_path = os.pathsep.join([str(root / "src" / "client"), str(root / "src" / "shared"), os.environ.get("PYTHONPATH", "")])
        actions.append(
            ExecuteProcess(
                cmd=[sys.executable, "-m", "leo_client", "--server", "127.0.0.1", "--sim", sim],
                additional_env={"PYTHONPATH": python_path, "LEO_CONFIG": config_path},
                cwd=str(root),
                output="own_log",
            )
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=_default_config()),
            DeclareLaunchArgument("sim", default_value=""),
            DeclareLaunchArgument("profile", default_value="false"),
            OpaqueFunction(function=_nodes),
        ]
    )
