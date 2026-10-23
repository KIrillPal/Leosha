from pathlib import Path

import pytest

from leo_server.common.config import ConfigError, load_config

ROOT = Path(__file__).resolve().parents[3]
CONFIG = ROOT / "config" / "robot.yaml"


def test_robot_yaml_loads():
    cfg = load_config(CONFIG)
    assert cfg.network["handshake_port"] == 5551
    assert cfg.client["lidar"]["invert_to_ccw"] is True
    assert cfg.slam_toolbox["minimum_travel_distance"] == 0.1
    assert "room" in cfg.sim["worlds"]


def test_missing_file_fails(tmp_path):
    with pytest.raises(ConfigError):
        load_config(tmp_path / "missing.yaml")


def test_unknown_key_fails(tmp_path):
    text = CONFIG.read_text(encoding="utf-8") + "\nextra: 1\n"
    path = tmp_path / "robot.yaml"
    path.write_text(text, encoding="utf-8")
    with pytest.raises(ConfigError, match="extra"):
        load_config(path)


def test_sim_payload_includes_one_world():
    cfg = load_config(CONFIG)
    payload = cfg.sim_payload("room")
    assert "walls" in payload["world"]
    assert payload["calibration"]["steering_invert"] is True
    with pytest.raises(ConfigError):
        cfg.sim_payload("missing")
