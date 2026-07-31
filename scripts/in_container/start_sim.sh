#!/usr/bin/env bash
set -eo pipefail

source /workspace/scripts/in_container/common.sh

sim_dir="${RAICOM_DIR}/sim_mujoco"

# Write cache files for robot selection
printf '%s\n' "${ROBOT_NAME}" > "${sim_dir}/bin/sim_robot_name_cache.txt"
printf '%s\n' "${sim_dir}/configuration" \
  > "${sim_dir}/bin/sim_configuration_directory_cache.txt"

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
mkdir -p "${sim_dir}/bin/cfg/tmp"
mkdir -p "${sim_dir}/bin/log"

cd "${sim_dir}/bin"
echo "Starting MuJoCo Simulation..."
echo "  Robot: ${ROBOT_NAME}"
echo "  Scene: ${SIM_ROBOT_PATH}/model_info/scene.xml"

exec ./aima-sim-app --cfg_file_path="${SIM_ROBOT_PATH}/simulator/default.yaml"
