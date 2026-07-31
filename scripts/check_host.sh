#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/lib/common.sh"

require_linux

failures=0

check_equal() {
  local label="$1" actual="$2" expected="$3"
  if [[ "${actual}" == "${expected}" ]]; then
    printf '[通过] %-22s %s\n' "${label}" "${actual}"
  else
    printf '[失败] %-22s 当前=%s，期望=%s\n' "${label}" "${actual}" "${expected}"
    failures=$((failures + 1))
  fi
}

source /etc/os-release
check_equal "Ubuntu 版本" "${VERSION_ID:-unknown}" "22.04"
check_equal "CPU 架构" "$(uname -m)" "x86_64"
check_equal "桌面会话" "${XDG_SESSION_TYPE:-unknown}" "x11"

if [[ -n "${DISPLAY:-}" ]]; then
  printf '[通过] %-22s %s\n' "DISPLAY" "${DISPLAY}"
else
  echo "[失败] DISPLAY 未设置，MuJoCo 图形窗口无法显示。"
  failures=$((failures + 1))
fi

for cmd in git git-lfs docker tmux xhost sha256sum tar; do
  if command -v "${cmd}" >/dev/null 2>&1; then
    printf '[通过] %-22s %s\n' "${cmd}" "$(command -v "${cmd}")"
  else
    printf '[失败] %-22s 未安装\n' "${cmd}"
    failures=$((failures + 1))
  fi
done

if command -v docker >/dev/null 2>&1; then
  if docker info >/dev/null 2>&1; then
    echo "[通过] Docker 服务与当前用户权限正常"
  else
    echo "[失败] 无法执行 docker info；请确认 Docker 已启动并重新登录账户。"
    failures=$((failures + 1))
  fi
fi

archive="${PROJECT_ROOT}/${VENDOR_ARCHIVE}"
if [[ -f "${archive}" ]]; then
  actual_hash="$(sha256sum "${archive}" | awk '{print $1}')"
  check_equal "官方压缩包 SHA256" "${actual_hash}" "${VENDOR_ARCHIVE_SHA256}"
else
  echo "[失败] Git LFS 文件不存在：${archive}"
  echo "       请执行：git lfs pull"
  failures=$((failures + 1))
fi

echo
if (( failures == 0 )); then
  echo "宿主机检查全部通过。"
else
  echo "共有 ${failures} 项未通过，请按照 docs/01-从零安装.md 排查。"
  exit 1
fi
