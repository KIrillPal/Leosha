"""Start every LEO node. The launch file passes the config path and an optional server address."""

from __future__ import annotations

import logging
import math
import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessIO, OnShutdown
from launch.logging import launch_config
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _enable_shared() -> None:
    for parent in Path(__file__).resolve().parents:
        shared = parent / "src" / "shared"
        if (shared / "leo_protocol" / "console.py").is_file():
            entry = str(shared)
            if entry not in sys.path:
                sys.path.insert(0, entry)
            return


_enable_shared()

from leo_protocol.console import (  # noqa: E402
    died_line,
    remember_output,
    short_name,
    shutdown_line,
    traceback_text,
    write,
    write_raw,
)

_STDERR: dict[str, str] = {}


def _quiet_launch_screen() -> None:
    launch_config.get_screen_handler().setLevel(logging.CRITICAL)


def _remember_stderr(event):
    raw = event.text
    if isinstance(raw, (bytes, bytearray)):
        text = raw.decode("utf-8", errors="replace")
    else:
        text = str(raw)
    name = event.process_name
    _STDERR[name] = remember_output(_STDERR.get(name, ""), text)


def _on_exit(event, context):
    name = event.process_name
    tail = _STDERR.pop(name, "")
    if context.is_shutdown or event.returncode == 0:
        return None
    write(died_line(short_name(name)))
    raw = traceback_text(tail)
    if raw:
        write_raw(raw)
    return None


def _on_shutdown(_event, _context):
    write(shutdown_line())
    return None


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
        output="own_log",
        arguments=[
            "--x",
            str(x),
            "--y",
            str(y),
            "--z",
            str(z),
            "--qx",
            qx,
            "--qy",
            qy,
            "--qz",
            qz,
            "--qw",
            qw,
            "--frame-id",
            parent,
            "--child-frame-id",
            child,
        ],
    )


def _nodes(context, *args, **kwargs):
    del args, kwargs
    config_path = LaunchConfiguration("config").perform(context)
    sim = LaunchConfiguration("sim").perform(context)
    server = LaunchConfiguration("server").perform(context).strip()
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
        arguments = ["--config", config_path]
        if name == "robot_bridge" and server:
            arguments.extend(["--server", server])
        actions.append(
            Node(
                package="leo_server",
                executable=name,
                name=name,
                arguments=arguments,
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
    _quiet_launch_screen()
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=_default_config()),
            DeclareLaunchArgument("sim", default_value=""),
            DeclareLaunchArgument("server", default_value=""),
            DeclareLaunchArgument("profile", default_value="false"),
            RegisterEventHandler(OnShutdown(on_shutdown=_on_shutdown)),
            RegisterEventHandler(OnProcessExit(on_exit=_on_exit)),
            RegisterEventHandler(OnProcessIO(on_stderr=_remember_stderr)),
            OpaqueFunction(function=_nodes),
        ]
    )
