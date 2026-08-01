#!/usr/bin/env bash
# 一站到底：JD → SD → LD → 前进
# 注意：SD 后需人工在 MuJoCo 点 Reset，脚本会等待确认
set -eo pipefail
source /opt/ros/humble/setup.bash
source /workspace/.runtime/ros/install/setup.bash 2>/dev/null

SPEED="${1:-0.15}"
DURATION="${2:-2.0}"

cd /workspace/.runtime/raicom2026/example/py
echo ">>> 设置 JD + SD ..."
python3 set_mode.py JD
python3 set_mode.py SD

echo ""
echo "========================================"
echo "  去 MuJoCo 窗口点击 Reset！"
echo "  然后在这里按 Enter 继续..."
echo "========================================"
read -r _

echo ">>> 切换到 LD ..."
python3 set_mode.py LD

echo ">>> 前进 ${SPEED} m/s，${DURATION} 秒 ..."
cd /workspace/control
python3 safe_forward.py --speed "$SPEED" --duration "$DURATION"
