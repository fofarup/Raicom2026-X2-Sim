#!/usr/bin/env bash
# X2 仿真分屏启动 — 单窗口三面板
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/../lib/common.sh"

# 容器检查
"${PROJECT_ROOT}/scripts/start_container.sh" >/dev/null 2>&1 || true

# xhost 权限
xhost +local:docker >/dev/null 2>&1 || true

# 确保 xhost 对所有容器开放
xhost + >/dev/null 2>&1 || true

# 如果已有同名 tmux 会话，直接接入
if tmux has-session -t "${TMUX_SESSION}" 2>/dev/null; then
  exec tmux attach-session -t "${TMUX_SESSION}"
fi

TMUX="${TMUX_SESSION}"

# 会话主窗口
tmux new-session -d -s "${TMUX}" -n "X2-Sim"

# 窗格 0：仿真（大窗格，上方/左方）
tmux send-keys -t "${TMUX}:0.0" \
  "echo '🖥️  启动 MuJoCo 仿真...'" Enter
sleep 0.5
tmux send-keys -t "${TMUX}:0.0" \
  "docker exec -it ${DOCKER_CONTAINER} bash -lc '/workspace/scripts/in_container/start_sim.sh'" Enter

# 右侧垂直分割 → 窗格 1：MC
sleep 2
tmux split-window -h -t "${TMUX}:0.0"
tmux send-keys -t "${TMUX}:0.1" \
  "echo '🔧 启动 MC 运动控制...'" Enter
sleep 0.5
tmux send-keys -t "${TMUX}:0.1" \
  "sleep 3; docker exec -it ${DOCKER_CONTAINER} bash -lc '/workspace/scripts/in_container/start_mc.sh'" Enter

# 下方水平分割窗格 0 → 窗格 2：控制终端
tmux split-window -v -t "${TMUX}:0.0"
tmux send-keys -t "${TMUX}:0.2" \
  "sleep 6; docker exec -it ${DOCKER_CONTAINER} bash -lc 'source /workspace/scripts/in_container/common.sh; echo \"\"; echo \"╔══════════════════════════════════════════╗\"; echo \"║  🎮  控制终端就绪                       ║\"; echo \"╠══════════════════════════════════════════╣\"; echo \"║  1. cd /workspace/.runtime/raicom2026/example/py\"; echo \"║  2. python3 set_mode.py SD               ║\"; echo \"║  3. 👉 去 MuJoCo 窗口点击 Reset         ║\"; echo \"║  4. python3 set_mode.py LD               ║\"; echo \"║  5. cd /workspace/control                ║\"; echo \"║  6. python3 safe_forward.py --speed 0.1 --duration 2\"; echo \"╚══════════════════════════════════════════╝\"; echo \"\"; exec bash'" Enter

# 调整大小
tmux resize-pane -t "${TMUX}:0.2" -D 12

# 聚焦到仿真窗格
tmux select-pane -t "${TMUX}:0.0"

exec tmux attach-session -t "${TMUX}"
