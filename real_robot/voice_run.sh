#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
app_dir="${script_dir}/app"
export RAICOM_ROBOT_PROFILE=real
export PYTHONPATH="${app_dir}${PYTHONPATH:+:${PYTHONPATH}}"

python3 "${app_dir}/profile_check.py"
readarray -t runtime < <(
  python3 -c 'from robot_profile import load_robot_profile; p=load_robot_profile(); print(p["runtime"]["ros_domain_id"]); print(p["runtime"]["rmw_implementation"]); print(p["runtime"]["ros_setup_path"]); print(p["runtime"]["aimdk_setup_path"])'
)
export ROS_DOMAIN_ID="${runtime[0]}"
export RMW_IMPLEMENTATION="${runtime[1]}"
source "${runtime[2]}"
source "${runtime[3]}"
tts_json="$(python3 -c 'from robot_profile import load_robot_profile; print(load_robot_profile()["audio"]["tts_command_json"] or "")')"
if [[ -z "${tts_json}" ]]; then
  echo "FAIL app/config/real_robot.json尚未填写audio.tts_command_json" >&2
  exit 2
fi
export RAICOM_TTS_COMMAND_JSON="${tts_json}"

python3 "${app_dir}/speech_gateway.py" &
gateway_pid=$!
trap 'kill -INT "${gateway_pid}" 2>/dev/null || true' EXIT INT TERM
python3 "${app_dir}/real_voice_controller.py"
