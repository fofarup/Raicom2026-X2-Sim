#!/usr/bin/env bash
# 切换到运动模式（LD）
# 需要先点过 MuJoCo 的 Reset 按钮
set -eo pipefail
source /opt/ros/humble/setup.bash
source /workspace/.runtime/ros/install/setup.bash 2>/dev/null
cd /workspace/.runtime/raicom2026/example/py
python3 set_mode.py LD
echo "运动模式已就绪，可以运行 safe_forward.py"
