#!/usr/bin/env bash
set -eo pipefail

export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8
export PYTHONIOENCODING=utf-8
export RAICOM_ROBOT_PROFILE=real
cd "$(dirname "${BASH_SOURCE[0]}")"

readarray -t setup_paths < <(
  python3 -c 'import json; p=json.load(open("config/real_robot.json")); r=p["runtime"]; print(r["ros_setup_path"]); print(r["aimdk_setup_path"]); print(r["ros_domain_id"]); print(r["rmw_implementation"])'
)
export ROS_DOMAIN_ID="${setup_paths[2]}"
export RMW_IMPLEMENTATION="${setup_paths[3]}"
source "${setup_paths[0]}"
source "${setup_paths[1]}"

exec python3 competition_agent.py "$@"
