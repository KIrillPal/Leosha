from __future__ import annotations

from dataclasses import dataclass
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


@dataclass
class ControlSection:
    max_speed_normal: float
    max_speed_fast: float
    max_steering: float
    head_sensitivity: float


@dataclass
class NetworkSection:
    grafana_url: str


@dataclass
class ServerConfig:
    app: AppSection
    robot: RobotSection
    control: ControlSection
    network: NetworkSection


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

    return ServerConfig(
        app=AppSection(**data["app"]),
        robot=RobotSection(**data["robot"]),
        control=ControlSection(**data["control"]),
        network=NetworkSection(**data["network"]),
    )
