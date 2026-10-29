#!/usr/bin/env bash
set -e
TASK_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
TASK_BUILD_ROOT="${DUAL_ARM_BUILD_ROOT:-/tmp/dual-arm-ros2}"
source "$TASK_BUILD_ROOT/install/setup.bash"
export ROS_DOMAIN_ID=164 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOG_DIR="$TASK_BUILD_ROOT/arm2_ros_logs"
exec python3 "$TASK_ROOT/scripts/arm2_integration.py" "$@"
