from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


@dataclass
class AppSection:
    host: str
    port: int
    command_hz: float


@dataclass
class RobotSection:
    ip: str
    backend: str = "mock"


@dataclass
class ControlSection:
    head_sensitivity: float


@dataclass
class ProfilesSection:
    """Per-profile runtime configuration.

    Hardcoded profile classes are still defined in code, but each gets a
    dedicated YAML subsection for tuning behavior.
    """

    pause: dict[str, Any] = field(default_factory=dict)
    teleoperation: dict[str, Any] = field(default_factory=dict)
    teleop_slam: dict[str, Any] = field(default_factory=dict)
    autonomy_profile_1: dict[str, Any] = field(default_factory=dict)
    following: dict[str, Any] = field(default_factory=dict)


@dataclass
class NetworkSection:
    grafana_url: str
    bind_address: str = "0.0.0.0"
    telemetry_port: int = 5550
    command_port: int = 5552
    report_port: int = 5553
    recv_timeout_ms: int = 100
    send_high_water_mark: int = 2
    recv_high_water_mark: int = 1


@dataclass
class SlamSection:
    """Порог уверенности в одометрии: если odom_confidence < порог, в TF идёт pose из slam_toolbox."""
    odom_confidence_threshold: float = 0.5


@dataclass
class ServerConfig:
    app: AppSection
    robot: RobotSection
    control: ControlSection
    network: NetworkSection
    profiles: ProfilesSection = field(default_factory=ProfilesSection)
    slam: SlamSection = field(default_factory=SlamSection)


def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    result = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = value
    return result


def _configs_dir() -> Path:
    """Папка configs рядом с server: code/configs (из code/server/server/ на 2 уровня вверх = code)."""
    return Path(__file__).resolve().parents[2] / "configs"


def load_server_config(path: str | Path) -> ServerConfig:
    custom_path = Path(path)
    default_path = _configs_dir() / "server.yaml"

    if custom_path.exists():
        config_path = custom_path
    elif default_path.exists():
        config_path = default_path
    else:
        raise FileNotFoundError(
            f"Config not found: {custom_path} (from --config) or {default_path} (default)"
        )

    with config_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    slam_data = data.get("slam") or {}
    slam = SlamSection(**slam_data) if slam_data else SlamSection()
    profiles_data = data.get("profiles") or {}
    profiles = ProfilesSection(**profiles_data) if profiles_data else ProfilesSection()
    return ServerConfig(
        app=AppSection(**data["app"]),
        robot=RobotSection(**data["robot"]),
        control=ControlSection(**data["control"]),
        profiles=profiles,
        network=NetworkSection(**data["network"]),
        slam=slam,
    )
