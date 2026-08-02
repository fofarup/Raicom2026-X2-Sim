#!/usr/bin/env bash
# 国赛一键运行：按顺序执行任务1→2→3
set -eo pipefail

SIM_FLAG=""
if [[ "${1:-}" == "--sim" ]] || [[ -z "${1:-}" ]]; then
    SIM_FLAG="--sim"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "╔══════════════════════════════════════════╗"
echo "║  睿抗2026 智慧养老组 国赛任务流程        ║"
echo "║  任务1 → 任务2 → 任务3                   ║"
echo "╚══════════════════════════════════════════╝"

echo ""
echo ">>> 任务1：自主导航与交互就位 (15分)"
cd "$SCRIPT_DIR/tasks"
python3 task1_navigation.py $SIM_FLAG

echo ""
echo ">>> 任务2：基础交互 (35分)"
python3 task2_interaction.py $SIM_FLAG

echo ""
echo ">>> 任务3：场景交互与自主服务 (50分)"
python3 task3_service.py $SIM_FLAG

echo ""
echo "═══════════════════════════════════════════"
echo "  全部任务执行完毕"
echo "═══════════════════════════════════════════"
