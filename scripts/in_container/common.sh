#!/usr/bin/env bash
set -euo pipefail

source /workspace/config/project.env

export PROJECT_ROOT=/workspace
export RUNTIME_ROOT=/workspace/.runtime
export RAICOM_DIR=/workspace/.runtime/raicom2026
export ROS_RUNTIME_DIR=/workspace/.runtime/ros
export ROS_DOMAIN_ID
export RMW_IMPLEMENTATION

source /opt/ros/humble/setup.bash
if [[ -f "${ROS_RUNTIME_DIR}/install/setup.bash" ]]; then
  source "${ROS_RUNTIME_DIR}/install/setup.bash"
fi

[[ -d "${RAICOM_DIR}" ]] || {
  echo "官方资源未初始化。请在宿主机运行 ./scripts/bootstrap_assets.sh" >&2
  return 1
}
