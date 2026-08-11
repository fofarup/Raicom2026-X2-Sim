#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
template="${script_dir}/app/config/real_robot.template.json"
target="${script_dir}/app/config/real_robot.json"

if [[ -e "${target}" ]]; then
  echo "配置已存在，不覆盖: ${target}"
  exit 0
fi
cp "${template}" "${target}"
chmod 600 "${target}"
echo "已创建私有真机配置: ${target}"
echo "下一步：填写所有 null，再运行 ./preflight.sh"
