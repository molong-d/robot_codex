#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source /opt/ros/jazzy/setup.bash
source install/setup.bash
exec ros2 launch robot_panda_gz_sim panda_pick_place.launch.py "$@"
