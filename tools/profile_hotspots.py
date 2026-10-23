"""Profile the mock pipeline pieces the plan called out, and print a short table."""

from __future__ import annotations

import cProfile
import io
import pstats
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "src" / "shared"), str(ROOT / "src" / "client"), str(ROOT / "src" / "server")]

from leo_client.sim.camera_render import render
from leo_client.sim.lidar_sim import cast_ranges
from leo_protocol.protocol import Scan, Telemetry, pack_telemetry
from leo_server.common.config import load_config
from leo_server.nodes.odometry.scan_matcher import match_scans, scan_points
import numpy as np


def _bench(name, fn, loops):
    start = time.perf_counter()
    for _ in range(loops):
        fn()
    elapsed = (time.perf_counter() - start) / loops * 1000.0
    print(f"{name:22} {elapsed:8.3f} ms")
    return elapsed


def main() -> None:
    cfg = load_config(ROOT / "config" / "robot.yaml")
    sim = cfg.sim_payload("room")
    world = sim["world"]
    rays = int(sim["lidar"]["num_rays"])
    ranges = cast_ranges(1.5, 1.2, 0.2, rays, 6.0, world)
    ranges2 = cast_ranges(1.6, 1.25, 0.25, rays, 6.0, world)
    prev = scan_points(ranges, -3.1416, 2 * 3.1416 / rays, 0.08, 6.0, 180)
    curr = scan_points(ranges2, -3.1416, 2 * 3.1416 / rays, 0.08, 6.0, 180)
    packet = Telemetry(1, 2, Scan(3, 0.1, -3.14, 0.02, 0.02, 6, ranges.astype(np.float32)), b"x" * 8000, 4, 320, 180, 0, 0, 0, 0, None, 0, 0, 0, 0, None)
    print("mean time per call")
    _bench("lidar cast 360", lambda: cast_ranges(1.5, 1.2, 0.2, rays, 6.0, world), 50)
    _bench("camera render", lambda: render((1.5, 1.2, 0.2), 10.0, -5.0, sim, 1.0), 20)
    _bench("scan match", lambda: match_scans(prev, curr, (0.1, 0.0, 0.05), 12, 0.25), 20)
    _bench("pack telemetry", lambda: pack_telemetry(packet), 100)
    profiler = cProfile.Profile()
    profiler.enable()
    for _ in range(10):
        render((1.5, 1.2, 0.2), 0.0, 0.0, sim, 1.0)
        match_scans(prev, curr, (0.05, 0.0, 0.0), 12, 0.25)
    profiler.disable()
    stream = io.StringIO()
    pstats.Stats(profiler, stream=stream).sort_stats("cumulative").print_stats(12)
    print(stream.getvalue())


if __name__ == "__main__":
    main()
