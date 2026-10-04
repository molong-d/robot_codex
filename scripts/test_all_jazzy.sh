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
python3 src/robot_panda_gz_sim/test/test_contact_semantics.py
python3 src/robot_panda_gz_sim/test/test_evidence_release_window.py
python3 src/robot_panda_gz_sim/test/test_offline_evidence.py
python3 src/robot_panda_gz_sim/test/test_gazebo_scenario_validation.py
