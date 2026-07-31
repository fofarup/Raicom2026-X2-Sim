#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/lib/common.sh"

require_linux
require_command tar
require_command sha256sum

archive="${PROJECT_ROOT}/${VENDOR_ARCHIVE}"
[[ -f "${archive}" ]] || die "缺少 ${archive}，请先执行 git lfs pull。"

actual_hash="$(sha256sum "${archive}" | awk '{print $1}')"
[[ "${actual_hash}" == "${VENDOR_ARCHIVE_SHA256}" ]] \
  || die "压缩包校验失败。实际：${actual_hash}"

if [[ -f "${RUNTIME_ROOT}/archive.sha256" ]] \
  && [[ "$(cat "${RUNTIME_ROOT}/archive.sha256")" == "${actual_hash}" ]] \
  && [[ -d "${OFFICIAL_ROOT}/link_u_os_competition-main/Raicom2026" ]]; then
  ln -sfn "official/link_u_os_competition-main/Raicom2026" "${RAICOM_DIR}"
  echo "官方资源已经初始化，无需重复解压。"
  exit 0
fi

mkdir -p "${RUNTIME_ROOT}"
staging="${RUNTIME_ROOT}/extract.tmp.$$"
trap 'rm -rf -- "${staging}"' EXIT
mkdir -p "${staging}"

echo "正在解压官方资源（约 1.8 GB）……"
tar -xzf "${archive}" -C "${staging}"

expected="${staging}/link_u_os_competition-main/Raicom2026"
[[ -x "${expected}/sim_mujoco/bin/start_sim.sh" ]] \
  || die "仿真启动脚本缺失。"
[[ -x "${expected}/mc/bin/em_run.sh" ]] \
  || die "MC 启动脚本缺失。"
[[ -f "${expected}/example/py/set_mode.py" ]] \
  || die "Python 官方例程缺失。"

# 只清理参考包中的历史输出；地图、模型、库和源码不动。
rm -rf -- \
  "${expected}/mc/bin/log" \
  "${expected}/aimdk-aarch64-1bde262f-artifacts/log"
rm -f -- "${expected}/mc/bin/em_run.log"

if [[ -e "${OFFICIAL_ROOT}" ]]; then
  die "${OFFICIAL_ROOT} 已存在但校验标记不匹配，请先将 .runtime 备份后再处理。"
fi

mv "${staging}" "${OFFICIAL_ROOT}"
trap - EXIT
ln -sfn "official/link_u_os_competition-main/Raicom2026" "${RAICOM_DIR}"
printf '%s\n' "${actual_hash}" > "${RUNTIME_ROOT}/archive.sha256"
mkdir -p "${ROS_RUNTIME_DIR}"/{build,install,log}

echo "官方资源初始化完成：${RAICOM_DIR}"
