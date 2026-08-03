#!/usr/bin/env bash
# 国赛一键运行：单节点连续执行，比赛开始后不重启、不复位。
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "╔══════════════════════════════════════════╗"
echo "║  睿抗2026 智慧养老组 国赛任务流程        ║"
echo "║  任务1 → 任务2 → 任务3                   ║"
echo "╚══════════════════════════════════════════╝"

exec python3 "$SCRIPT_DIR/competition_node.py" "$@"
