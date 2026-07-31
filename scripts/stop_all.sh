#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/lib/common.sh"

if docker ps --format '{{.Names}}' | grep -Fxq "${DOCKER_CONTAINER}"; then
  docker exec \
    --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
    "${DOCKER_CONTAINER}" \
    bash -lc \
    'source /workspace/scripts/in_container/common.sh; python3 /workspace/control/safe_forward.py --stop-only' \
    >/dev/null 2>&1 || true
  docker stop "${DOCKER_CONTAINER}" >/dev/null
  echo "容器已停止：${DOCKER_CONTAINER}"
else
  echo "容器未运行。"
fi

if command -v xhost >/dev/null 2>&1; then
  xhost -local:docker >/dev/null 2>&1 || true
fi
