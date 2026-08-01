#!/usr/bin/env bash
# 让机器人稳定站立（JD → SD）
# 运行后必须去 MuJoCo 窗口点击 Reset！
set -eo pipefail
source /opt/ros/humble/setup.bash
source /workspace/.runtime/ros/install/setup.bash 2>/dev/null
cd /workspace/.runtime/raicom2026/example/py
python3 set_mode.py JD
python3 set_mode.py SD
echo ""
echo "========================================"
echo "  👉 去 MuJoCo 窗口点击 Reset！"
echo "========================================"
