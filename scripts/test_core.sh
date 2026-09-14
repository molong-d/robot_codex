#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$repo_root/.build"
export TMPDIR="$repo_root/.build"
"${CXX:-g++}" -std=c++17 -Wall -Wextra -Wpedantic -Werror \
  -I"$repo_root/src/robot_core/include" \
  "$repo_root/src/robot_core/test/core_test.cpp" -o "$repo_root/.build/core_test"
"$repo_root/.build/core_test"
