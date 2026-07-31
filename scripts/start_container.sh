#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/lib/common.sh"

require_linux
require_command docker
require_command xhost
require_assets

if docker ps --format '{{.Names}}' | grep -Fxq "${DOCKER_CONTAINER}"; then
  echo "容器已经运行：${DOCKER_CONTAINER}"
  exit 0
fi

if docker ps -a --format '{{.Names}}' | grep -Fxq "${DOCKER_CONTAINER}"; then
  docker start "${DOCKER_CONTAINER}" >/dev/null
  echo "已重新启动容器：${DOCKER_CONTAINER}"
  exit 0
fi

docker image inspect "${DOCKER_IMAGE}" >/dev/null 2>&1 \
  || die "镜像不存在，请先执行：./scripts/build_image.sh"

xhost +local:docker >/dev/null

docker_args=(
  run --detach --interactive --tty
  --name "${DOCKER_CONTAINER}"
  --privileged
  --network host
  --ipc host
  --pid host
  --env "DISPLAY=${DISPLAY}"
  --env "QT_X11_NO_MITSHM=1"
  --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
  --env "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
  --volume "/tmp/.X11-unix:/tmp/.X11-unix:rw"
  --volume "/dev/input:/dev/input"
  --volume "${PROJECT_ROOT}:/workspace"
  --workdir /workspace
)

if [[ "${USE_NVIDIA_GPU}" == "1" ]]; then
  docker_args+=(
    --gpus all
    --env NVIDIA_VISIBLE_DEVICES=all
    --env NVIDIA_DRIVER_CAPABILITIES=all
  )
fi

docker "${docker_args[@]}" "${DOCKER_IMAGE}" >/dev/null
echo "容器已启动：${DOCKER_CONTAINER}"
