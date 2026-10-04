#!/usr/bin/env bash
set -euo pipefail

if [[ "${ROS_DISTRO:-}" != "jazzy" ]]; then
  echo "此检查必须在 scripts/with_jazzy.sh 启动的 ROS 2 Jazzy 容器内运行。" >&2
  exit 2
fi

source /etc/os-release
if [[ "${VERSION_ID:-}" != "24.04" ]]; then
  echo "需要 Ubuntu 24.04；当前容器为 ${PRETTY_NAME:-unknown}" >&2
  exit 2
fi

echo "OS: ${PRETTY_NAME}"
echo "ROS_DISTRO: ${ROS_DISTRO}"
dpkg-query -W ros-jazzy-moveit-core ros-jazzy-behaviortree-cpp \
  ros-jazzy-controller-manager ros-jazzy-gz-ros2-control ros-jazzy-ros-gz-sim
echo "Gazebo Sim: $(gz sim --versions)"
for package in moveit_core behaviortree_cpp controller_manager gz_ros2_control ros_gz_sim; do
  printf '%s: ' "$package"
  ros2 pkg prefix "$package"
done
