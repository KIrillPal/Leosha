# Profiling

Measured with `python3 tools/profile_hotspots.py` on the `room` world (360 lidar rays, 320x180 camera). Times are the mean of many calls on this machine.

| Piece | Before | After | Note |
|---|---|---|---|
| Lidar ray cast | 0.10 ms | 0.10 ms | Already one numpy pass. Fine at 10 Hz. |
| Camera render | 1.53 ms | 1.02 ms | The per-column Python loop was the cost. Wall strips are now a mask. |
| Scan match (12 iters, 180 points) | 0.74 ms | 0.79 ms | Fine at 10 Hz. No change. |
| Telemetry pack, scan plus an 8 KB JPEG | 0.004 ms | 0.004 ms | Ranges are float32 bytes, not a msgpack list of floats. |

The camera still calls the ray caster once per frame. At the configured 15 Hz that is about 15 ms/s, which is not worth a more complicated renderer.

What was not kept, because it would have made the code harder to read for a small gain:

- a second, coarser ray grid for the camera
- skipping ICP iterations with a hand-tuned threshold beyond the existing 1e-4 stop
- caching the JPEG decoder size by opening the image in the mode manager (the vision message already carries width and height, so that decode is gone)

`ros2 launch leo_server leo.launch.py sim:=room profile:=true` wraps each LEO node with `cProfile` and writes `logs/<node>.prof`. Read one with `python3 -m pstats logs/odometry.prof`.

The closed-loop numbers to watch on the monitor page are system FPS (bridge telemetry rate) and the client loop / camera / lidar rates. They are different clocks: the client rates are measured on the robot, the system rate is measured on the server.
