#!/usr/bin/env bash
# X2 仿真分屏启动 — 单窗口三面板，每个面板都自动进入 Docker 容器
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/../lib/common.sh"

# 容器检查
"${PROJECT_ROOT}/scripts/start_container.sh" >/dev/null 2>&1 || true

# xhost 权限
xhost +local:docker >/dev/null 2>&1 || true
xhost + >/dev/null 2>&1 || true

# 如果已有同名 tmux 会话，直接接入
if tmux has-session -t "${TMUX_SESSION}" 2>/dev/null; then
  exec tmux attach-session -t "${TMUX_SESSION}"
fi

# ---- 创建会话 ----
tmux new-session -d -s "${TMUX_SESSION}" -n "X2-Sim" \
  "docker exec -it ${DOCKER_CONTAINER} bash -lc '/workspace/scripts/in_container/start_sim.sh; exec bash'"

# ---- MC 窗格 ----
tmux split-window -h -t "${TMUX_SESSION}:0.0" \
  "sleep 3; docker exec -it ${DOCKER_CONTAINER} bash -lc '/workspace/scripts/in_container/start_mc.sh; exec bash'"

# ---- 控制终端窗格 ----
tmux split-window -v -t "${TMUX_SESSION}:0.0" \
  "sleep 6; docker exec -it ${DOCKER_CONTAINER} bash -lc '
source /workspace/scripts/in_container/common.sh 2>/dev/null
echo \"\"
echo \"╔══════════════════════════════════════╗\"
echo \"║  🎮  控制终端就绪（已在容器内）     ║\"
echo \"╠══════════════════════════════════════╣\"
echo \"║  1. bash /workspace/control/stand.sh ║\"
echo \"║  2. 👉 去 MuJoCo 点 Reset           ║\"
echo \"║  3. bash /workspace/control/ready.sh ║\"
echo \"║  4. bash /workspace/control/go.sh    ║\"
echo \"╚══════════════════════════════════════╝\"
echo \"\"
exec bash
'"

# 调整大小
tmux resize-pane -t "${TMUX_SESSION}:0.2" -D 12

# 聚焦到仿真窗格
tmux select-pane -t "${TMUX_SESSION}:0.0"

exec tmux attach-session -t "${TMUX_SESSION}"
