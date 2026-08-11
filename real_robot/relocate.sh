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

if [[ "${1:-}" == "execute" ]]; then
  active_pattern="${app_dir}/(speech_gateway|real_voice_controller|competition_agent|task1_agent|task2_agent|task3_scene_agent|voice_robot_actions)\\.py"
  if pgrep -f "${active_pattern}" >/dev/null 2>&1; then
    echo "FAIL 已有比赛或语音进程运行；请在原终端按Ctrl+C并确认全部退出" >&2
    pgrep -af "${active_pattern}" >&2 || true
    exit 2
  fi
fi

exec python3 "${app_dir}/relocate_agent.py" "$@"
