#!/usr/bin/env bash
set -eo pipefail

expected_type="aimdk_msgs/msg/ProcessedAudioOutput"
actual_type="$(ros2 topic type /agent/process_audio_output 2>/dev/null || true)"
if [[ "${actual_type}" != "${expected_type}" ]]; then
  echo "FAIL 麦克风话题类型: expected=${expected_type} actual=${actual_type:-missing}" >&2
  exit 1
fi

for service in \
  /aimdk_5Fmsgs/srv/RequestAudioFocus \
  /aimdk_5Fmsgs/srv/AbandonAudioFocus; do
  if [[ -z "$(ros2 service type "${service}" 2>/dev/null || true)" ]]; then
    echo "FAIL 服务不存在: ${service}" >&2
    exit 1
  fi
done

playback_type="$(ros2 topic type /aima/hal/audio/playback 2>/dev/null || true)"
if [[ -n "${playback_type}" && "${playback_type}" != "aimdk_msgs/msg/AudioPlayback" ]]; then
  echo "FAIL 播放话题类型异常: ${playback_type}" >&2
  exit 1
fi

echo "PASS 音频接口存在"
echo "下一步：说一次唤醒词后运行："
echo "python3 ../robot_audio_input.py --timeout 20"
