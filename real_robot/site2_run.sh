#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
site2_map_id="1786066723179"

if [[ "$#" -eq 0 ]]; then
  echo "用法: $0 --pixel-x X --pixel-y Y --yaw-deg DEG --confirm-at-pose [重定位选项]" >&2
  echo "执行前还必须设置 RAICOM_CONFIRM_REAL_ROBOT=YES 和 RAICOM_CONFIRM_RELOCALIZATION=YES" >&2
  exit 2
fi

for argument in "$@"; do
  case "${argument}" in
    --map-id|--map-id=*)
      echo "FAIL site2_run.sh固定使用二号地图ID ${site2_map_id}，不得覆盖--map-id" >&2
      exit 2
      ;;
  esac
done

"${script_dir}/relocate.sh" execute --map-id "${site2_map_id}" "$@"
exec "${script_dir}/voice_run.sh"
