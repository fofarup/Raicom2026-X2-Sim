#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
competition_dir="${script_dir}/app"

export RAICOM_ROBOT_PROFILE=real
export PYTHONPATH="${competition_dir}${PYTHONPATH:+:${PYTHONPATH}}"
python3 "${competition_dir}/profile_check.py"

readarray -t runtime < <(
  python3 -c 'from robot_profile import load_robot_profile; p=load_robot_profile(); print(p["runtime"]["ros_domain_id"]); print(p["runtime"]["rmw_implementation"])'
)
export ROS_DOMAIN_ID="${runtime[0]}"
export RMW_IMPLEMENTATION="${runtime[1]}"

exec "${competition_dir}/competition_run.sh" "$@"
