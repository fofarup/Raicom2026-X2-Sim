#!/usr/bin/env bash
set -eo pipefail

source /workspace/scripts/in_container/common.sh
cd "${RAICOM_DIR}/example/py"
python3 set_mode.py SD

echo
echo "已请求 SD 稳定站立模式。现在请回到 MuJoCo 窗口点击 Reset。"
