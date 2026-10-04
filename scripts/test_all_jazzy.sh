#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

bash scripts/test_core.sh
source /opt/ros/jazzy/setup.bash
colcon build --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --verbose
python3 scripts/test_ros.py
python3 scripts/test_panda.py
