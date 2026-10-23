# LEO2

Off-board control for the LEO robot. The robot runs a thin client. The server runs one ROS 2 node per job. Every setting is in [`config/robot.yaml`](config/robot.yaml).

The original tree in `../LEO` is unchanged.

## Install dependencies

`ros2` is `/usr/bin/python3`. A conda env or a Poetry virtualenv is a different interpreter, so `ros2 launch` still reports `No module named 'msgpack'`.

ROS packages come from apt. The Poetry projects in `src/client`, `src/server`, and `src/shared` describe the Python libraries. Install those into the user site of `/usr/bin/python3`, which `ros2` searches. Do not turn Poetry virtualenvs off: that install tries to write under `/usr` and stops on a permission error.

```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-slam-toolbox python3-pip python3-opencv
cd /home/kir/Leosha/LEO2
/usr/bin/python3 -m pip install --user -e "src/server[sim,vision]"
```

`-E` / `[sim]` pulls the client, which `sim:=room` starts with the same interpreter. `[vision]` adds YOLO and ONNX Runtime. Use `"src/server[sim]"` if you do not need poses.

On the robot, a virtualenv is the right place. The robot extra installs ServoKit, `lgpio`, and Picamera2, and builds the YDLidar SDK (`cmake` and `swig`). `libcamera` comes from conda-forge. The pip bindings do not compile against the apt headers, and apt `python3-libcamera` is built for the system Python, so the client virtualenv is set up to see the conda site-packages.

Picamera2 also imports `pykms`. That module is apt `python3-kms++`, built for the system Python 3.11. The pip package `rpi-kms` does not compile against the current kms++ headers, so this Python 3.10 virtualenv still cannot import Picamera2.

```bash
conda install -y -c conda-forge "libcamera>=0.5"
sudo apt install -y cmake swig
/usr/bin/python3 -m pip install --user poetry
export PATH="$HOME/.local/bin:$PATH"
cd /home/kir/Leosha/LEO2/src/client
poetry install -E robot
poetry run python -m leo_client --server SERVER_HOST
```

| Package | Path | What it installs |
|---|---|---|
| `leo-protocol` | `src/shared` | `msgpack`, `numpy`. Both sides import this. |
| `leo-client` | `src/client` | `pyzmq`, OpenCV. Extra `sim`: pygame. Extra `robot`: ServoKit, `lgpio`, Picamera2, YDLidar SDK. `libcamera` comes from conda-forge. |
| `leo-server` | `src/server` | Flask, PyYAML, `pyzmq`, SciPy, OpenCV, the protocol. Extra `sim`: the client. Extra `vision`: Ultralytics, ONNX Runtime. |

`colcon build` only registers the ROS package. Install the Python packages first.

## Run the server with the mock robot

```bash
source /opt/ros/humble/setup.bash
cd /home/kir/Leosha/LEO2/src/server
colcon build
source install/setup.bash
export LEO_CONFIG=/home/kir/Leosha/LEO2/config/robot.yaml
ros2 launch leo_server leo.launch.py sim:=room
```

- Control page: <http://127.0.0.1:8080>
- Node graph: <http://127.0.0.1:8081> (also `/graph`)
- Each process writes `stdout` and `stderr` to its own file under `~/.ros/log`. The launch terminal stays quiet. Rebuild with `colcon build` so this launch file is the one ROS runs.
- `sim:=room` starts the mock client against the `room` world in `robot.yaml`.
- `profile:=true` writes `logs/<node>.prof` for each LEO node.

## Run the real robot

On the server, launch without `sim`. Set `network.robot_ip` in `robot.yaml` to the robot, or type the address on the settings page (that override is not written back to the file).

On the robot:

```bash
python3 -m leo_client --server SERVER_HOST
```

The client has no config file. It learns calibration, sensor rates and ports from the handshake.

Optional mock flags, used on the server machine:

```bash
python3 -m leo_client --server 127.0.0.1 --sim room --show
python3 -m leo_client --server 127.0.0.1 --sim room --route
```

`--show` opens a top-down window. `--route` plays `sim.route` through the same throttle conversion as a real command.

## Tests

```bash
cd /home/kir/Leosha/LEO2
/usr/bin/python3 -m pytest -q
```

The ROS bringup test needs Humble sourced and `LEO_RUN_ROS=1`. Use `/usr/bin/python3` after the install above. Conda's Python cannot import `rclpy`. Pytest must stay on 7.x: Humble's `launch_testing` plugin does not load under pytest 8.

```bash
source /opt/ros/humble/setup.bash
LEO_RUN_ROS=1 /usr/bin/python3 -m pytest src/server/tests/test_ros_system.py -q
```

## Layout

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for nodes, topics, the mode switch, and where sensor fusion will plug in. Profiling notes are in [docs/PROFILING.md](docs/PROFILING.md).
