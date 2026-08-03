#!/usr/bin/env bash
set -eo pipefail

source /workspace/scripts/in_container/common.sh

# Used by the autonomous pre-run recovery to click MuJoCo's real Reset button.
# The simulator's advertised option topic does not implement Reset in this
# official binary.
gcc /workspace/sim/x11_window_tool.c -o /tmp/raicom_x11_window_tool \
  -lX11 /lib/x86_64-linux-gnu/libXtst.so.6 -lpng

sim_dir="${RAICOM_DIR}/sim_mujoco"
launch_dir="${RAICOM_DIR}/project_sim_bin"

# 项目覆盖模型由官方模型可重复生成；官方目录始终保持不变。
if [[ "${ROBOT_NAME}" == "lx2501_3_t2d5_raicom" ]]; then
  python3 /workspace/sim/prepare_competition_assets.py \
    --configuration-root "${sim_dir}/configuration"
  # The official ray-caster loses its PointCloud metadata during mjData reset.
  # Build and load the audited project-owned implementation from an isolated
  # working directory; the official simulator and plugin remain untouched.
  mkdir -p "${launch_dir}/mujoco_plugin" "${launch_dir}/log"
  bash /workspace/sim/build_lidar_plugin.sh "${sim_dir}" \
    "${launch_dir}/mujoco_plugin/libsensor_ray.so"
  ln -sfn "${sim_dir}/bin/aima-sim-app" "${launch_dir}/aima-sim-app"
  relay_pid_file="${launch_dir}/arm_state_relay.pid"
  relay_running=0
  if [[ -f "${relay_pid_file}" ]]; then
    relay_pid="$(<"${relay_pid_file}")"
    if kill -0 "${relay_pid}" 2>/dev/null; then relay_running=1; fi
  fi
  if [[ "${relay_running}" -eq 0 ]]; then
    python3 /workspace/sim/arm_state_relay.py \
      >"${launch_dir}/log/arm_state_relay.log" 2>&1 &
    echo "$!" >"${relay_pid_file}"
  fi
else
  launch_dir="${sim_dir}/bin"
fi

# Write cache files for robot selection
printf '%s\n' "${ROBOT_NAME}" > "${launch_dir}/sim_robot_name_cache.txt"
printf '%s\n' "${sim_dir}/configuration" \
  > "${launch_dir}/sim_configuration_directory_cache.txt"

# Set environment variables
export SIM_ROBOT_PATH="${sim_dir}/configuration/robot/${ROBOT_NAME}"
export SIM_RESOURCE_MODEL_PATH="${sim_dir}/resource/model"
export DISPLAY="${DISPLAY:-:1}"
export LD_LIBRARY_PATH="${sim_dir}/bin:${sim_dir}/lib:${LD_LIBRARY_PATH}"

# Disable optional components that require extra executors in this config
export AGIBOT_ENABLE_HDS_COMPONENT=0
export AGIBOT_ENABLE_EVENT_COMPONENT=0
export AGIBOT_ENABLE_AUDIT_COMPONENT=0

# Ensure required directories
mkdir -p "${launch_dir}/cfg/tmp"
mkdir -p "${launch_dir}/log"

cd "${launch_dir}"
echo "Starting MuJoCo Simulation..."
echo "  Robot: ${ROBOT_NAME}"
echo "  Scene: ${SIM_ROBOT_PATH}/model_info/scene.xml"

exec ./aima-sim-app --cfg_file_path="${SIM_ROBOT_PATH}/simulator/default.yaml"
