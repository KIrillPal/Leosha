"""Stress the Pi and every actuator: a fast tremble plus a CPU and memory load.

Uses the same PCA9685 map and neutral calibration as the robot client.
Drive and steering stay at the leo-client idle throttle. The neck is a
pan command: clamped to the calibrated limits, then written as a servo
angle on neck_channel. The face is the tilt command on face_channel.
Peak amplitude and CPU duty both grow linearly for the first 20 seconds.
Ctrl+C parks everything.
"""

from __future__ import annotations

import argparse
import logging
import math
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")

import numpy as np
import yaml

LOGGER = logging.getLogger("leo.stress")

_ROOT = Path(__file__).resolve().parents[3]
_DEFAULT_CONFIG = _ROOT / "config" / "robot.yaml"


_SLICE_SEC = 0.05


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def ramp_scale(elapsed: float, ramp_sec: float) -> float:
    """0 at the start, 1 after ramp_sec. Peak power tracks this fraction."""
    if ramp_sec <= 0.0:
        return 1.0
    return _clamp(elapsed / ramp_sec, 0.0, 1.0)


class _Ramp:
    def __init__(self) -> None:
        self.scale = 0.0


def load_client(path: Path) -> dict:
    with path.open() as handle:
        document = yaml.safe_load(handle)
    client = document["client"]
    return {"servos": client["servos"], "calibration": client["calibration"]}


# Command degrees, the same frame leo-client sends. neck_channel is 2 and
# receives the pan angle. face_channel is 3 and receives the tilt angle.
# Pan 0 is the neck's calibrated straight pose.
_HOME_PAN_DEG = 0.0
_HOME_TILT_DEG = 50.0


def home_pose(calibration: dict) -> tuple[float, float, float, float]:
    """Drive and steering match leo-client idle. Neck and face use its angle map."""
    from leo_client.calibration import pan_servo_deg, steering_throttle, throttle_from_speed, tilt_servo_deg

    return (
        throttle_from_speed(0.0, calibration),
        steering_throttle(0.0, calibration),
        pan_servo_deg(_HOME_PAN_DEG, calibration),
        tilt_servo_deg(_HOME_TILT_DEG, calibration),
    )


def tremble(
    t: float,
    *,
    freq: float,
    home: tuple[float, float, float, float],
    calibration: dict,
    motor_amp: float,
    steer_amp: float,
    pan_amp: float,
    tilt_amp: float,
) -> tuple[float, float, float, float]:
    """Shiver around the home pose. Neck and face offsets are command degrees."""
    from leo_client.calibration import pan_servo_deg, tilt_servo_deg

    motor_home, steer_home, _, _ = home
    wave = math.sin(2.0 * math.pi * freq * t)
    motor = _clamp(motor_home + motor_amp * wave, -1.0, 1.0)
    steer = _clamp(steer_home + steer_amp * wave, -1.0, 1.0)
    pan = pan_servo_deg(_HOME_PAN_DEG + pan_amp * wave, calibration)
    tilt = tilt_servo_deg(_HOME_TILT_DEG + tilt_amp * wave, calibration)
    return motor, steer, pan, tilt


def cpu_temp_c() -> float | None:
    path = Path("/sys/class/thermal/thermal_zone0/temp")
    try:
        return int(path.read_text().strip()) / 1000.0
    except OSError:
        return None


def mem_available() -> int:
    try:
        for line in Path("/proc/meminfo").read_text().splitlines():
            if line.startswith("MemAvailable:"):
                return int(line.split()[1]) * 1024
    except OSError:
        pass
    return 256 * 1024 * 1024


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


def _idle(stop: threading.Event, scale: float) -> bool:
    if scale >= 1.0:
        return False
    return stop.wait(_SLICE_SEC * (1.0 - scale))


def _burn(stop: threading.Event, ramp: _Ramp) -> None:
    left = np.random.random((160, 160)).astype(np.float32)
    right = np.random.random((160, 160)).astype(np.float32)
    while not stop.is_set():
        scale = _clamp(ramp.scale, 0.0, 1.0)
        if scale > 0.0:
            until = time.monotonic() + _SLICE_SEC * scale
            while time.monotonic() < until and not stop.is_set():
                product = left @ right
                left = product * (1.0 / (float(np.max(np.abs(product))) + 1e-6))
        if _idle(stop, scale):
            return


def _touch_memory(stop: threading.Event, ramp: _Ramp, nbytes: int) -> None:
    buffer = np.zeros(max(1, nbytes // 4), dtype=np.uint32)
    stride = max(1, 4096 // 4)
    index = 0
    while not stop.is_set():
        scale = _clamp(ramp.scale, 0.0, 1.0)
        if scale > 0.0:
            until = time.monotonic() + _SLICE_SEC * scale
            while time.monotonic() < until and not stop.is_set():
                buffer[index] = buffer[index] + 1
                index = (index + stride) % buffer.size
        if _idle(stop, scale):
            return


def _start_load(stop: threading.Event, ramp: _Ramp, workers: int, memory_bytes: int) -> list[threading.Thread]:
    threads = [threading.Thread(target=_burn, args=(stop, ramp), name=f"cpu-{i}", daemon=True) for i in range(workers)]
    if memory_bytes > 0:
        threads.append(threading.Thread(target=_touch_memory, args=(stop, ramp, memory_bytes), name="memory", daemon=True))
    for thread in threads:
        thread.start()
    return threads


def _stop_load(stop: threading.Event, threads: list[threading.Thread]) -> None:
    stop.set()
    for thread in threads:
        thread.join(timeout=1.0)


def run(args: argparse.Namespace) -> None:
    client = load_client(args.config)
    calibration = client["calibration"]
    servos_cfg = client["servos"]
    home = home_pose(calibration)
    workers = args.workers if args.workers > 0 else (os.cpu_count() or 1)
    if args.memory_mib < 0:
        memory_bytes = min(256 * 1024 * 1024, mem_available() // 5)
    else:
        memory_bytes = args.memory_mib * 1024 * 1024

    if args.dry_run:
        period = 1.0 / args.freq
        for step in range(8):
            sample = tremble(
                step * period / 8.0,
                freq=args.freq,
                home=home,
                calibration=calibration,
                motor_amp=args.motor_amp,
                steer_amp=args.steer_amp,
                pan_amp=args.pan_amp,
                tilt_amp=args.tilt_amp,
            )
            print(
                f"t={step * period / 8.0:.4f} motor={sample[0]:+.3f} steer={sample[1]:+.3f} "
                f"pan={sample[2]:.1f} tilt={sample[3]:.1f}"
            )
        print(
            f"home motor={home[0]:+.3f} steer={home[1]:+.3f} "
            f"neck ch{servos_cfg['neck_channel']} pan_cmd={_HOME_PAN_DEG:+.0f} servo={home[2]:.1f} "
            f"face ch{servos_cfg['face_channel']} tilt_cmd={_HOME_TILT_DEG:+.0f} servo={home[3]:.1f}"
        )
        print(f"dry-run: {workers} cpu workers, {memory_bytes / (1024 * 1024):.0f} MiB, ramp {args.ramp:.0f}s, no hardware")
        from leo_client.calibration import pan_servo_deg, tilt_servo_deg

        for second in (0.0, 5.0, 10.0, 15.0, 20.0, 25.0):
            scale = ramp_scale(second, args.ramp)
            pan_cmd = _HOME_PAN_DEG + args.pan_amp * scale
            tilt_cmd = _HOME_TILT_DEG + args.tilt_amp * scale
            print(
                f"t={second:4.0f}s power={scale * 100:5.1f}% "
                f"motor_peak={home[0] + args.motor_amp * scale:+.3f} "
                f"steer_peak={home[1] + args.steer_amp * scale:+.3f} "
                f"pan_cmd={pan_cmd:+.1f} servo={pan_servo_deg(pan_cmd, calibration):.1f} "
                f"tilt_cmd={tilt_cmd:+.1f} servo={tilt_servo_deg(tilt_cmd, calibration):.1f}"
            )
        return

    if client_service_active() and not args.force:
        raise SystemExit("leo-client.service is running and owns the motor board. Stop it, or pass --force.")

    stop = threading.Event()

    def _request_stop(signum: int, _frame: object) -> None:
        LOGGER.info("signal %s, parking", signum)
        stop.set()

    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    ramp = _Ramp()
    threads = _start_load(stop, ramp, workers, memory_bytes)
    from leo_client.hardware.servos import Servos

    board = Servos(servos_cfg, calibration)
    period = 1.0 / args.hz
    deadline = None
    LOGGER.info(
        "tremble %.1f Hz on motor/steer/neck/face, %d cpu workers, %.0f MiB, update %.0f Hz, "
        "peak power ramps linearly over %.0fs. Neck is channel %s, face is channel %s. Ctrl+C parks.",
        args.freq,
        workers,
        memory_bytes / (1024 * 1024),
        args.hz,
        args.ramp,
        servos_cfg["neck_channel"],
        servos_cfg["face_channel"],
    )
    LOGGER.info("home motor=%+.3f steer=%+.3f pan=%.1f tilt=%.1f", *home)
    try:
        board.start()
        board.apply(*home)
        started = time.monotonic()
        if args.duration > 0.0:
            deadline = started + args.duration
        next_tick = started
        next_report = started
        while not stop.is_set():
            now = time.monotonic()
            if deadline is not None and now >= deadline:
                break
            scale = ramp_scale(now - started, args.ramp)
            ramp.scale = scale
            motor, steer, pan, tilt = tremble(
                now,
                freq=args.freq,
                home=home,
                calibration=calibration,
                motor_amp=args.motor_amp * scale,
                steer_amp=args.steer_amp * scale,
                pan_amp=args.pan_amp * scale,
                tilt_amp=args.tilt_amp * scale,
            )
            board.apply(motor, steer, pan, tilt)
            if now >= next_report:
                temp = cpu_temp_c()
                temp_text = f"{temp:.1f}C" if temp is not None else "n/a"
                load = Path("/proc/loadavg").read_text().split()[0]
                LOGGER.info(
                    "temp=%s load=%s power=%.0f%% motor=%+.3f steer=%+.3f pan=%.1f tilt=%.1f",
                    temp_text,
                    load,
                    scale * 100.0,
                    motor,
                    steer,
                    pan,
                    tilt,
                )
                if temp is not None and temp >= 80.0:
                    LOGGER.warning("CPU temperature is %.1fC", temp)
                next_report = now + 2.0
            next_tick += period
            delay = next_tick - time.monotonic()
            if delay < 0.0:
                next_tick = time.monotonic()
                delay = 0.0
            if stop.wait(delay):
                break
    finally:
        try:
            board.apply(*home)
            time.sleep(0.15)
        except Exception:
            LOGGER.exception("failed to return to neutral")
        board.close()
        _stop_load(stop, threads)
        LOGGER.info("parked")


def main() -> None:
    parser = argparse.ArgumentParser(description="Tremble every LEO actuator and load the Raspberry Pi")
    parser.add_argument("--config", type=Path, default=_DEFAULT_CONFIG, help="robot.yaml used by leo-client")
    parser.add_argument("--duration", type=float, default=60.0, help="seconds to run after start; 0 runs until Ctrl+C")
    parser.add_argument("--ramp", type=float, default=20.0, help="seconds to grow peak power linearly from idle to full")
    parser.add_argument("--hz", type=float, default=100.0, help="actuator update rate")
    parser.add_argument("--freq", type=float, default=12.0, help="tremble frequency in Hz")
    parser.add_argument("--motor-amp", type=float, default=0.30, help="drive throttle tremble around neutral, max 0.60")
    parser.add_argument("--steer-amp", type=float, default=0.90, help="steering throttle tremble around 0, max 1.00")
    parser.add_argument("--pan-amp", type=float, default=15.0, help="neck pan command shiver in degrees, max 135")
    parser.add_argument("--tilt-amp", type=float, default=12.0, help="face tilt command shiver in degrees, max 24")
    parser.add_argument("--workers", type=int, default=0, help="CPU worker threads; 0 uses every core")
    parser.add_argument("--memory-mib", type=int, default=-1, help="extra RAM to stride through; -1 picks a safe slice")
    parser.add_argument("--force", action="store_true", help="run even if leo-client.service is active")
    parser.add_argument("--dry-run", action="store_true", help="print one tremble cycle and do not touch hardware")
    args = parser.parse_args()
    if args.hz <= 0.0 or args.freq <= 0.0:
        raise SystemExit("--hz and --freq must be positive")
    if args.ramp < 0.0:
        raise SystemExit("--ramp must be >= 0")
    args.motor_amp = _clamp(args.motor_amp, 0.0, 0.60)
    args.steer_amp = _clamp(args.steer_amp, 0.0, 1.00)
    args.pan_amp = _clamp(args.pan_amp, 0.0, 135.0)
    args.tilt_amp = _clamp(args.tilt_amp, 0.0, 24.0)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s %(message)s")
    if str(Path(__file__).resolve().parents[1]) not in sys.path:
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    run(args)


if __name__ == "__main__":
    main()
