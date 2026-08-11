#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
target="${1:-}"
mode="${2:---dry-run}"

if [[ -z "${target}" || ("${mode}" != "--dry-run" && "${mode}" != "--execute") ]]; then
  echo "用法: $0 <ssh用户@真机IP> [--dry-run|--execute]" >&2
  echo "示例: $0 agi@192.168.1.20 --dry-run" >&2
  exit 2
fi
if ! command -v rsync >/dev/null || ! command -v ssh >/dev/null; then
  echo "FAIL 电脑需要安装 rsync 和 ssh" >&2
  exit 2
fi

destination="/home/agi/x2_deploy_workspace/raicom_real_robot"

options=(-az --itemize-changes --human-readable)
if [[ "${mode}" == "--dry-run" ]]; then
  options+=(--dry-run)
  echo "预览模式：不会写入真机"
  if ! ssh "${target}" "test -d '${destination}'"; then
    echo "目标目录尚不存在。先运行一次--execute可自动创建，或在真机手动mkdir。" >&2
    exit 3
  fi
else
  echo "执行模式：增量同步到 ${target}:${destination}"
  ssh "${target}" "mkdir -p '${destination}'"
fi

rsync "${options[@]}" \
  --exclude '/app/config/real_robot.json' \
  --exclude '/app/config/doubao_api_key' \
  --exclude '/app/config/*_api_key' \
  --exclude '/.env' \
  --exclude '**/__pycache__/' \
  --exclude '*.pyc' \
  "${script_dir}/" "${target}:${destination%/}/"

if [[ "${mode}" == "--execute" ]]; then
  echo "PASS 独立真机目录已同步；私有配置和密钥未被覆盖"
  echo "INFO 若本地已准备ASR模型，模型文件也已按rsync清单同步"
  echo "下一步：ssh ${target}，进入${destination}运行 ./preflight.sh"
fi
