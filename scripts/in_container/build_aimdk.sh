#!/usr/bin/env bash
set -euo pipefail

source /workspace/scripts/in_container/common.sh

source_root="${RAICOM_DIR}/aimdk-aarch64-1bde262f-artifacts/src"
[[ -f "${source_root}/aimdk_msgs/package.xml" ]] || {
  echo "找不到 aimdk_msgs 源码：${source_root}" >&2
  exit 1
}

mkdir -p "${ROS_RUNTIME_DIR}"/{build,install,log}

colcon \
  --log-base "${ROS_RUNTIME_DIR}/log" \
  build \
  --base-paths "${source_root}" \
  --packages-select aimdk_msgs \
  --build-base "${ROS_RUNTIME_DIR}/build" \
  --install-base "${ROS_RUNTIME_DIR}/install" \
  --symlink-install

source "${ROS_RUNTIME_DIR}/install/setup.bash"
ros2 interface show aimdk_msgs/msg/McLocomotionVelocity >/dev/null
echo "aimdk_msgs 构建和接口检查通过。"
