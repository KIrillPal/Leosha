#!/bin/bash
# Launch the LEO server the way README.md describes, against the mounted config.
set -euo pipefail

source /opt/ros/humble/setup.bash
source /opt/leo/src/server/install/setup.bash

export LEO_CONFIG="${LEO_CONFIG:-/opt/leo/config/robot.yaml}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/opt/leo/logs/ros}"
mkdir -p "$ROS_LOG_DIR"

cd /opt/leo
args=(ros2 launch leo_server leo.launch.py "config:=${LEO_CONFIG}")
if [[ -n "${LEO_SIM:-}" ]]; then
  args+=("sim:=${LEO_SIM}")
fi
exec "${args[@]}"
