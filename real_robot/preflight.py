#!/usr/bin/env python3
"""Read-only real-X2 topic/service/config preflight."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


HERE = Path(__file__).resolve().parent
CONFIG = HERE / "app" / "config" / "real_robot.json"
TOPIC_TYPES = {
    "hardware.rgb_topic": (
        "sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage",
    ),
    "hardware.depth_topic": ("sensor_msgs/msg/Image",),
    "hardware.camera_info_topic": ("sensor_msgs/msg/CameraInfo",),
    "hardware.depth_camera_info_topic": ("sensor_msgs/msg/CameraInfo",),
    "hardware.arm_state_topic": ("aimdk_msgs/msg/JointStateArray",),
    "hardware.upper_body_command_topic": ("aimdk_msgs/msg/UpperBodyCommandArray",),
    "topics.odometry": ("nav_msgs/msg/Odometry",),
    "topics.vad_audio": ("aimdk_msgs/msg/ProcessedAudioOutput",),
    "topics.audio_playback": ("aimdk_msgs/msg/AudioPlayback",),
    "topics.audio_focus_response": ("aimdk_msgs/msg/FocusResponse",),
}
CONFIGURED_TYPE_TOPICS = {
    "topics.localization_pose": "topics.localization_pose_type",
    "hardware.gripper_command_topic": "hardware.gripper_command_type",
    "hardware.gripper_state_topic": "hardware.gripper_state_type",
}
CONFIGURED_SERVICES = (
    "services.mc_action", "services.mc_input_source", "services.preset_motion",
    "services.play_emoji", "services.audio_focus_request", "services.audio_focus_release",
)


def nested(data: dict, path: str):
    value = data
    for part in path.split("."):
        value = value.get(part) if isinstance(value, dict) else None
    return value


def ros2(*args: str) -> str:
    try:
        result = subprocess.run(
            ["ros2", *args], text=True, stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, timeout=4, check=False,
        )
        return result.stdout.strip()
    except subprocess.TimeoutExpired as exc:
        output = exc.stdout or ""
        if isinstance(output, bytes):
            output = output.decode(errors="replace")
        return output.strip()


def main() -> int:
    if not CONFIG.is_file():
        print("FAIL 缺少 app/config/real_robot.json，请先运行 ./create_config.sh")
        return 2
    data = json.loads(CONFIG.read_text(encoding="utf-8"))
    failures = 0
    print(f"CONFIG {CONFIG}")
    for path, expected_types in TOPIC_TYPES.items():
        topic = nested(data, path)
        if not topic:
            print(f"FAIL 未填写 {path}")
            failures += 1
            continue
        actual = ros2("topic", "type", str(topic))
        ok = actual in expected_types
        expected = "|".join(expected_types)
        print(f"{'PASS' if ok else 'FAIL'} {path}={topic} type={actual or 'missing'} expected={expected}")
        failures += int(not ok)
    for topic_path, type_path in CONFIGURED_TYPE_TOPICS.items():
        topic, expected = nested(data, topic_path), nested(data, type_path)
        if not topic or not expected:
            print(f"FAIL 未填写 {topic_path} / {type_path}")
            failures += 1
            continue
        actual = ros2("topic", "type", str(topic))
        ok = actual == expected
        print(f"{'PASS' if ok else 'FAIL'} {topic_path}={topic} type={actual or 'missing'} expected={expected}")
        failures += int(not ok)
    for service_path in CONFIGURED_SERVICES:
        service = nested(data, service_path)
        if not service:
            print(f"FAIL 未填写 {service_path}")
            failures += 1
            continue
        actual = ros2("service", "type", service)
        ok = bool(actual)
        print(f"{'PASS' if ok else 'FAIL'} service={service} type={actual or 'missing'}")
        failures += int(not ok)
    rgb = nested(data, "hardware.rgb_topic")
    if rgb:
        hz = ros2("topic", "hz", "--window", "3", str(rgb))
        print("INFO RGB频率检查请现场持续观察：", hz.splitlines()[-1] if hz else "未取得样本")
    print(f"SUMMARY failures={failures}")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
