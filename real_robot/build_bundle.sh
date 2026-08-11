#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
competition_dir="$(dirname "${script_dir}")"
output="${1:-${competition_dir}/raicom_real_robot.tar.zst}"

tar --zstd -cf "${output}" \
  --transform='s,^real_robot,raicom_real_robot,' \
  --exclude="real_robot/app/config/real_robot.json" \
  --exclude="real_robot/app/config/doubao_api_key" \
  --exclude="real_robot/app/config/*_api_key" \
  --exclude="real_robot/.env" \
  --exclude='__pycache__' --exclude='*.pyc' \
  -C "${competition_dir}" real_robot

echo "PASS 真机包: ${output}"
du -h "${output}"
echo "真机解压: tar --zstd -xf $(basename "${output}") -C /home/agi/x2_deploy_workspace/"
