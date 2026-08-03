#!/usr/bin/env bash
set -eo pipefail

source /workspace/scripts/in_container/common.sh

mc_dir="${RAICOM_DIR}/mc"
# MuJoCo uses the project overlay name, while the unchanged MC parameters stay
# under the official base robot name (the added lidar/claws do not alter legs).
if [[ "${ROBOT_NAME}" == "lx2501_3_t2d5_raicom" ]]; then
  export ROBOT_NAME="lx2501_3_t2d5"
fi
printf '%s\n' "${ROBOT_NAME}" > "${mc_dir}/bin/sim_robot_name_cache.txt"

cd "${mc_dir}/bin"
exec ./em_run.sh
