#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/../lib/common.sh"

if tmux has-session -t "${TMUX_SESSION}" 2>/dev/null; then
  tmux kill-session -t "${TMUX_SESSION}"
  echo "tmux 会话已关闭：${TMUX_SESSION}"
else
  echo "tmux 会话不存在。"
fi

"${PROJECT_ROOT}/scripts/stop_all.sh"
