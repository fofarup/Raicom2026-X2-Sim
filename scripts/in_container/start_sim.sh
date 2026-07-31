#!/usr/bin/env bash
set -euo pipefail

source /workspace/scripts/in_container/common.sh

sim_dir="${RAICOM_DIR}/sim_mujoco"
printf '%s\n' "${ROBOT_NAME}" > "${sim_dir}/bin/sim_robot_name_cache.txt"
printf '%s\n' "${sim_dir}/configuration" \
  > "${sim_dir}/bin/sim_configuration_directory_cache.txt"

cd "${sim_dir}/bin"
exec ./start_sim.sh
