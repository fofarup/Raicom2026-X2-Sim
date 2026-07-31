#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/lib/common.sh"

echo "=== Docker 容器 ==="
docker ps --filter "name=^/${DOCKER_CONTAINER}$" \
  --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}'

if ! docker ps --format '{{.Names}}' | grep -Fxq "${DOCKER_CONTAINER}"; then
  exit 1
fi

echo
echo "=== 仿真与 MC 进程 ==="
docker exec "${DOCKER_CONTAINER}" \
  bash -lc "pgrep -af 'aima-sim-app|mc_app_main' || true"

echo
echo "=== ROS 2 节点（ROS_DOMAIN_ID=${ROS_DOMAIN_ID}）==="
docker exec \
  --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
  --env "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}" \
  "${DOCKER_CONTAINER}" \
  bash -lc 'source /workspace/scripts/in_container/common.sh; ros2 node list' \
  || true
