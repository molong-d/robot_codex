#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${ROBOT_CODEX_JAZZY_IMAGE:-robot-codex-jazzy:local}"
UBUNTU_MIRROR="${ROBOT_CODEX_UBUNTU_MIRROR:-https://mirrors.ustc.edu.cn/ubuntu}"
ROS_MIRROR="${ROBOT_CODEX_ROS_MIRROR:-https://mirrors.ustc.edu.cn/ros2/ubuntu}"
BUILDX_CONFIG="${BUILDX_CONFIG:-${TMPDIR:-/tmp}/robot-codex-buildx-${UID:-$(id -u)}}"

command -v docker >/dev/null || {
  echo "需要安装 Docker Engine，并允许当前用户访问 Docker daemon。" >&2
  exit 127
}

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  mkdir -p "$BUILDX_CONFIG"
  export BUILDX_CONFIG
  docker build \
    --network host \
    --file "$ROOT/docker/jazzy.Dockerfile" \
    --build-arg "UBUNTU_MIRROR=$UBUNTU_MIRROR" \
    --build-arg "ROS_MIRROR=$ROS_MIRROR" \
    --tag "$IMAGE" \
    "$ROOT"
fi

if (($# == 0)); then
  set -- bash
fi

TTY_ARGS=()
if [[ -t 0 && -t 1 ]]; then
  TTY_ARGS=(-it)
fi

exec docker run --rm "${TTY_ARGS[@]}" \
  --network host \
  --user "$(id -u):$(id -g)" \
  --env ROS_DOMAIN_ID=42 \
  --env ROS_LOCALHOST_ONLY=1 \
  --volume "$ROOT:/workspace" \
  --workdir /workspace \
  "$IMAGE" "$@"
