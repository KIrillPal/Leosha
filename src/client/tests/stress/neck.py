"""Manual neck control through the same path as leo-client.

A pan command in degrees is clamped to the calibrated limits and written as a
servo angle on neck_channel. The face, drive and steering channels are not
touched. Command 0 is the idle pose leo-client holds before the first packet.
"""

from __future__ import annotations

import argparse
import os
import select
import subprocess
import sys
import termios
import tty
from pathlib import Path

import yaml

from leo_client.calibration import pan_servo_deg

_ROOT = Path(__file__).resolve().parents[3]
_DEFAULT_CONFIG = _ROOT / "config" / "robot.yaml"


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def load_neck(path: Path) -> tuple[dict, dict]:
    with path.open() as handle:
        client = yaml.safe_load(handle)["client"]
    return client["servos"], client["calibration"]


def neck_limits(calibration: dict) -> tuple[float, float]:
    return float(calibration["head_pan_min_deg"]), float(calibration["head_pan_max_deg"])


def _open_neck(servos_cfg: dict):
    from adafruit_servokit import ServoKit

    kit = ServoKit(channels=int(servos_cfg["pca_channels"]), frequency=int(servos_cfg["pca_frequency"]))
    servo = kit.servo[int(servos_cfg["neck_channel"])]
    servo.actuation_range = int(servos_cfg["neck_actuation_range"])
    low = int(servos_cfg["neck_pwm_min"])
    high = int(servos_cfg["neck_pwm_max"])
    if low > 0 and high > 0:
        servo.set_pulse_width_range(low, high)
    return servo


def _read_key() -> str:
    char = os.read(sys.stdin.fileno(), 1)
    if char != b"\x1b":
        return char.decode("utf-8", "ignore")
    if select.select([sys.stdin], [], [], 0.05)[0]:
        char += os.read(sys.stdin.fileno(), 2)
    return char.decode("utf-8", "ignore")


def _command(key: str, command_deg: float, low: float, high: float, step: float) -> tuple[float | None, bool]:
    if key in ("q", "Q", "\x03"):
        return None, True
    if key in ("a", "A", "\x1b[D"):
        return _clamp(command_deg - (step * 5 if key == "A" else step), low, high), False
    if key in ("d", "D", "\x1b[C"):
        return _clamp(command_deg + (step * 5 if key == "D" else step), low, high), False
    if key in ("r", "R"):
        return 0.0, False
    return command_deg, False


def client_service_active() -> bool:
    try:
        result = subprocess.run(
            ["systemctl", "--user", "is-active", "leo-client.service"],
            capture_output=True,
            text=True,
            timeout=2,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    return result.stdout.strip() == "active"


def _format(command_deg: float, calibration: dict) -> str:
    return f"pan {command_deg:+6.1f}°  servo {pan_servo_deg(command_deg, calibration):5.1f}°"


def run(args: argparse.Namespace) -> None:
    servos_cfg, calibration = load_neck(args.config)
    low, high = neck_limits(calibration)
    command_deg = _clamp(0.0 if args.angle is None else args.angle, low, high)
    if args.dry_run:
        print(f"neck channel {servos_cfg['neck_channel']}  command {low:.0f}..{high:.0f}°  idle { _format(0.0, calibration) }")
        for sample in (low, -90.0, 0.0, 90.0, high):
            print(" ", _format(sample, calibration))
        print("a/left decreases pan, d/right increases pan, A/D step x5, r idle, q quit")
        return
    if client_service_active() and not args.force:
        raise SystemExit("leo-client.service is running and owns the motor board. Stop it, or pass --force.")

    servo = _open_neck(servos_cfg)
    servo.angle = pan_servo_deg(command_deg, calibration)
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    print(f"{_format(command_deg, calibration)}  a/left d/right  A/D x5  r idle  q quit", flush=True)
    try:
        tty.setcbreak(fd)
        while True:
            key = _read_key()
            updated, stop = _command(key, command_deg, low, high, args.step)
            if stop or updated is None:
                break
            if updated != command_deg:
                command_deg = updated
                servo.angle = pan_servo_deg(command_deg, calibration)
                print(f"\r{_format(command_deg, calibration)}", end="", flush=True)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        print(f"\nheld at {_format(command_deg, calibration)}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Move only the neck the way leo-client does")
    parser.add_argument("--config", type=Path, default=_DEFAULT_CONFIG, help="robot.yaml used by leo-client")
    parser.add_argument("--angle", type=float, default=None, help="starting pan command in degrees; default is 0")
    parser.add_argument("--step", type=float, default=1.0, help="command degrees per key press")
    parser.add_argument("--force", action="store_true", help="run even if leo-client.service is active")
    parser.add_argument("--dry-run", action="store_true", help="print the conversion and do not touch hardware")
    args = parser.parse_args()
    if args.step <= 0.0:
        raise SystemExit("--step must be positive")
    client = Path(__file__).resolve().parents[1]
    if str(client) not in sys.path:
        sys.path.insert(0, str(client))
    run(args)


if __name__ == "__main__":
    main()
