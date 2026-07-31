#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# shellcheck disable=SC1091
source "${PROJECT_ROOT}/config/project.env"

RUNTIME_ROOT="${PROJECT_ROOT}/.runtime"
OFFICIAL_ROOT="${RUNTIME_ROOT}/official"
RAICOM_DIR="${RUNTIME_ROOT}/raicom2026"
ROS_RUNTIME_DIR="${RUNTIME_ROOT}/ros"

export PROJECT_ROOT RUNTIME_ROOT OFFICIAL_ROOT RAICOM_DIR ROS_RUNTIME_DIR
export ROS_DOMAIN_ID RMW_IMPLEMENTATION

die() {
  echo "[错误] $*" >&2
  exit 1
}

require_command() {
  command -v "$1" >/dev/null 2>&1 || die "缺少命令：$1"
}

require_linux() {
  [[ "$(uname -s)" == "Linux" ]] || die "此脚本必须在 Ubuntu/Linux 中运行。"
}

require_assets() {
  [[ -L "${RAICOM_DIR}" || -d "${RAICOM_DIR}" ]] \
    || die "官方资源尚未初始化，请先运行：./scripts/bootstrap_assets.sh"
}

require_container() {
  docker ps --format '{{.Names}}' | grep -Fxq "${DOCKER_CONTAINER}" \
    || die "容器 ${DOCKER_CONTAINER} 未运行，请先执行：./scripts/start_container.sh"
}
