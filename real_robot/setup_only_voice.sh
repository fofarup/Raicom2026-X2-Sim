#!/usr/bin/env bash
set -eo pipefail

if [[ "${RAICOM_CONFIRM_REAL_ROBOT:-}" != "YES" ]]; then
  echo "拒绝执行：这会修改真机agent配置。确认真机后先 export RAICOM_CONFIRM_REAL_ROBOT=YES" >&2
  exit 2
fi

mode="${1:-only_voice}"
if [[ "${mode}" != "only_voice" && "${mode}" != "normal" ]]; then
  echo "用法: $0 [only_voice|normal]" >&2
  exit 2
fi

ros2 service call \
  /aimdk_5Fmsgs/srv/SetAgentPropertiesRequest \
  aimdk_msgs/srv/SetAgentPropertiesRequest "
contents:
  properties:
    - key:
        value: 2
      value: '${mode}'
"

echo "正在按官方流程重启agent（不会重启MC）"
aima em stop-app agent
aima em start-app agent
echo "agent模式已请求切换为 ${mode}；请运行 voice_preflight.sh 并进行一次唤醒词测试"
