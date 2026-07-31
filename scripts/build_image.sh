#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/lib/common.sh"

require_linux
require_command docker

docker build \
  --build-arg USER_UID="$(id -u)" \
  --build-arg USER_GID="$(id -g)" \
  --tag "${DOCKER_IMAGE}" \
  --file "${PROJECT_ROOT}/docker/Dockerfile" \
  "${PROJECT_ROOT}/docker"

echo "镜像构建完成：${DOCKER_IMAGE}"
