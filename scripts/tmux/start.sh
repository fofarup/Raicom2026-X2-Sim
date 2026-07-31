#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/../lib/common.sh"

require_command tmux
"${PROJECT_ROOT}/scripts/start_container.sh"
require_container

if tmux has-session -t "${TMUX_SESSION}" 2>/dev/null; then
  exec tmux attach-session -t "${TMUX_SESSION}"
fi

tmux new-session -d -s "${TMUX_SESSION}" -n sim \
  "docker exec -it ${DOCKER_CONTAINER} bash -lc '/workspace/scripts/in_container/start_sim.sh; exec bash'"

tmux new-window -t "${TMUX_SESSION}" -n mc \
  "sleep 5; docker exec -it ${DOCKER_CONTAINER} bash -lc '/workspace/scripts/in_container/start_mc.sh; exec bash'"

tmux new-window -t "${TMUX_SESSION}" -n control \
  "docker exec -it ${DOCKER_CONTAINER} bash -lc 'source /workspace/scripts/in_container/common.sh; echo \"控制终端已就绪。先按教程执行站立与 Reset。\"; exec bash'"

tmux select-window -t "${TMUX_SESSION}:sim"
exec tmux attach-session -t "${TMUX_SESSION}"
