#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
competition_dir="${script_dir}/app"
export RAICOM_ROBOT_PROFILE=real
export PYTHONPATH="${competition_dir}${PYTHONPATH:+:${PYTHONPATH}}"

readarray -t runtime < <(
  python3 -c 'from robot_profile import load_robot_profile; p=load_robot_profile(); print(p["runtime"]["ros_domain_id"]); print(p["runtime"]["rmw_implementation"]); print(p["runtime"]["ros_setup_path"]); print(p["runtime"]["aimdk_setup_path"])'
)
export ROS_DOMAIN_ID="${runtime[0]}"
export RMW_IMPLEMENTATION="${runtime[1]}"
source "${runtime[2]}"
source "${runtime[3]}"

python3 "${competition_dir}/profile_check.py"
python3 "${competition_dir}/relocate_agent.py" check \
  --map-id "${RAICOM_SITE_MAP_ID:-1786066723179}"
python3 "${script_dir}/preflight.py"
"${script_dir}/voice_preflight.sh"
echo "PASS 真机只读预检完成；仍需按 FILES_AND_CALIBRATION.md 做人工标定"
