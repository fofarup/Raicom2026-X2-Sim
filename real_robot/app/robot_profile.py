#!/usr/bin/env python3
"""Standalone real-X2 profile loader; simulation profiles are rejected."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any


COMPETITION_DIR = Path(__file__).resolve().parent
REAL_PROFILE = COMPETITION_DIR / "config" / "real_robot.json"


class ProfileError(RuntimeError):
    pass


def _required(data: dict[str, Any], dotted_key: str) -> Any:
    value: Any = data
    for key in dotted_key.split("."):
        if not isinstance(value, dict) or key not in value:
            raise ProfileError(f"机器人配置缺少字段: {dotted_key}")
        value = value[key]
    if value is None:
        raise ProfileError(f"机器人配置尚未标定: {dotted_key}")
    return value


def validate_profile(data: dict[str, Any]) -> None:
    required = (
        "profile_name",
        "robot_kind",
        "runtime.ros_domain_id",
        "runtime.rmw_implementation",
        "runtime.ros_setup_path",
        "runtime.aimdk_setup_path",
        "topics.localization_pose",
        "topics.localization_pose_type",
        "topics.odometry",
        "topics.locomotion_velocity",
        "topics.speech_text",
        "audio.input_backend",
        "audio.output_backend",
        "audio.sample_rate",
        "audio.sample_format",
        "audio.channels",
        "hardware.rgb_topic",
        "navigation.zones.出发区",
        "navigation.zones.交互区-I",
        "navigation.zones.交互区-II",
        "navigation.zones.作业区",
        "navigation.task1_turn_drift_precompensation_x_m",
        "navigation.task1_final_yaw_deg",
        "navigation.task1_turn_precomp_xy",
        "navigation.task1_turn_point_xy",
        "navigation.task1_interaction_dock_offset_y_m",
        "navigation.start_yaw_deg",
        "navigation.work_position_tolerance_m",
        "navigation.table_position_xy",
        "navigation.footprint_dock_offset_y_m",
        "navigation.footprint_dock_tolerance_m",
        "navigation.prealign_position_tolerance_m",
        "navigation.max_table_realign_attempts",
        "navigation.max_safe_zone_distance_m",
        "navigation.realign_lateral_trigger_m",
        "navigation.max_uncorrectable_lateral_m",
        "navigation.supports_lateral_velocity",
        "navigation.max_forward_velocity_mps",
        "navigation.max_lateral_velocity_mps",
        "navigation.max_angular_velocity_rps",
    )
    for key in required:
        _required(data, key)
    if data["robot_kind"] != "real":
        raise ProfileError("独立真机程序只接受 robot_kind=real")
    for name, xy in data["navigation"]["zones"].items():
        if not isinstance(xy, list) or len(xy) != 2:
            raise ProfileError(f"区域坐标必须为 [x, y]: {name}")

    if data["robot_kind"] == "real":
        real_required = (
            "topics.vad_audio", "topics.audio_playback",
            "topics.audio_focus_response",
            "services.audio_focus_request", "services.audio_focus_release",
            "services.mc_action", "services.mc_input_source",
            "services.preset_motion", "services.play_emoji",
            "hardware.depth_topic", "hardware.camera_info_topic",
            "hardware.depth_camera_info_topic",
            "hardware.arm_state_topic", "hardware.upper_body_command_topic",
            "hardware.gripper_command_topic", "hardware.gripper_command_type",
            "hardware.gripper_state_topic", "hardware.gripper_state_type",
            "grasp.joint_tracking_tolerance_rad", "grasp.ee_tracking_tolerance_m",
            "grasp.camera_to_robot_transform",
            "grasp.calibrated", "grasp.direct_hand_control_requires_mc_stopped",
        )
        for key in real_required:
            _required(data, key)


def selected_profile_path() -> Path:
    explicit = os.environ.get("RAICOM_PROFILE_FILE")
    if explicit:
        return Path(explicit).expanduser().resolve()
    return REAL_PROFILE


def load_robot_profile() -> dict[str, Any]:
    path = selected_profile_path()
    if not path.is_file():
        raise ProfileError(
            f"机器人配置不存在: {path}。请先复制 real_robot.template.json "
            "为 real_robot.json 并完成标定。"
        )
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ProfileError(f"无法读取机器人配置 {path}: {exc}") from exc
    validate_profile(data)
    data["_path"] = str(path)
    return data


def nested(profile: dict[str, Any], dotted_key: str) -> Any:
    return _required(profile, dotted_key)
