#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
if [[ -f /workspace/install/setup.bash ]]; then
  source /workspace/install/setup.bash
fi

exec "$@"
