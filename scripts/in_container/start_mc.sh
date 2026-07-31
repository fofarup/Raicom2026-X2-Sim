#!/usr/bin/env bash
set -eo pipefail

source /workspace/scripts/in_container/common.sh

mc_dir="${RAICOM_DIR}/mc"
printf '%s\n' "${ROBOT_NAME}" > "${mc_dir}/bin/sim_robot_name_cache.txt"

cd "${mc_dir}/bin"
exec ./em_run.sh
