#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source /opt/ros/jazzy/setup.bash
source install/setup.bash
exec python3 scripts/test_gazebo.py "$@"
