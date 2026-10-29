#!/usr/bin/env bash
set -e
TASK_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# rosidl's CMake file(STRINGS) handling breaks on non-ASCII build paths.
# Keep generated artifacts in an ASCII path; source files stay in the workspace.
TASK_BUILD_ROOT="${DUAL_ARM_BUILD_ROOT:-/tmp/dual-arm-ros2}"
source /opt/ros/jazzy/setup.bash
colcon --log-base "$TASK_BUILD_ROOT/log" build \
  --base-paths "$TASK_ROOT/src" --build-base "$TASK_BUILD_ROOT/build" \
  --install-base "$TASK_BUILD_ROOT/install" --symlink-install --executor sequential
echo "Built. Source: $TASK_BUILD_ROOT/install/setup.bash"
