#!/usr/bin/env python3
"""Robot-side actions used by the host voice controller."""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException

from task1_agent import ROBOT_PROFILE, Task1Agent, ZONES, normalize_angle
from task3_scene_agent import Task3SceneAgent


GRASP_ACTIONS = {
    "grasp-medicine": "药盒",
    "grasp-cup": "一次性纸杯",
    "grasp-bread": "小面包",
}


def nearest_zone(x: float, y: float) -> tuple[str, float]:
    distances = [
        (math.hypot(x - zone_x, y - zone_y), name)
        for name, (zone_x, zone_y) in ZONES.items()
    ]
    distance, name = min(distances)
    return name, distance


def report_status(agent: Task1Agent) -> bool:
    if not agent.wait_for_pose():
        return False
    pose, age, source = agent._current_pose()
    if pose is None:
        return False
    x, y, yaw = pose
    zone, distance = nearest_zone(x, y)
    description = zone if distance <= 0.35 else f"{zone}附近"
    print(
        f"[位置] 区域={description} x={x:.3f} y={y:.3f} "
        f"yaw={math.degrees(yaw):.1f} distance={distance:.3f} source={source} age={age:.3f}",
        flush=True,
    )
    return True


def emergency_stop(agent: Task1Agent) -> bool:
    # Register when possible so zero velocity has an accepted source.  Even when
    # registration fails, publishing zero and requesting SD are both attempted.
    agent.wait_for_pose(timeout_s=2.0)
    registered = agent.register_source()
    agent.stop()
    mode_ok = agent.set_mode("STAND_DEFAULT")
    agent.stop()
    print(
        f"[急停] zero_velocity=sent input_source={registered} SD={mode_ok}",
        flush=True,
    )
    return mode_ok


def navigate_home(agent: Task1Agent) -> bool:
    target_x, target_y = ZONES["出发区"]
    navigation = ROBOT_PROFILE["navigation"]
    target_yaw = math.radians(float(navigation["start_yaw_deg"]))
    # Complete the large turn outside the circle. The pre-bias is a real-robot
    # calibration value and must be measured at low speed on the competition floor.
    staging_x = target_x + float(navigation["task1_turn_drift_precompensation_x_m"])
    staging_y = target_y + 0.50
    agent.get_logger().info("语音恢复：闭环返回出发区")
    if not agent.wait_for_pose() or not agent.register_source():
        return False
    pose, age, source = agent._current_pose()
    if pose is not None and age <= agent.POSE_MAX_AGE_S:
        distance = math.hypot(pose[0] - target_x, pose[1] - target_y)
        yaw_error = abs(normalize_angle(target_yaw - pose[2]))
        if distance <= agent.POSITION_TOLERANCE_M and yaw_error <= agent.YAW_TOLERANCE_RAD:
            agent.stop()
            success = agent.set_mode("STAND_DEFAULT")
            agent.get_logger().info(
                f"已经位于出发区[{source}]：distance={distance:.3f}m，"
                f"yaw_error={math.degrees(yaw_error):.1f}°，无需重复导航"
            )
            return success
    if not agent.set_mode("STAND_DEFAULT"):
        return False
    time.sleep(1.0)
    if not agent.set_mode("LOCOMOTION_DEFAULT"):
        return False
    try:
        agent.get_logger().info(
            f"返回阶段1：前往圈外预就位点 ({staging_x:.2f}, {staging_y:.2f})"
        )
        if not agent.drive_to(staging_x, staging_y, timeout_s=55.0):
            return False
        agent.get_logger().info("返回阶段2：圈外调整出发朝向")
        if not agent.rotate_to(target_yaw, timeout_s=18.0):
            return False
        agent.get_logger().info("返回阶段3：保持朝向倒退闭环进入出发区")
        if not agent.dock_at(target_x, target_y, target_yaw, timeout_s=35.0):
            return False
        pose, age, source = agent._current_pose()
        if pose is None or age > agent.POSE_MAX_AGE_S:
            return False
        distance = math.hypot(pose[0] - target_x, pose[1] - target_y)
        agent.get_logger().info(
            f"返回出发区完成[{source}]：x={pose[0]:.3f}, y={pose[1]:.3f}, distance={distance:.3f}m"
        )
        return distance <= agent.POSITION_TOLERANCE_M
    finally:
        agent.stop()
        agent.set_mode("STAND_DEFAULT")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "action",
        choices=("status", "stop", "start", "interaction", "work", *GRASP_ACTIONS),
    )
    args = parser.parse_args()
    rclpy.init()
    if args.action == "work" or args.action in GRASP_ACTIONS:
        agent = Task3SceneAgent()
    else:
        agent = Task1Agent("voice_robot_actions")
    success = False
    try:
        if args.action == "status":
            success = report_status(agent)
        elif args.action == "stop":
            success = emergency_stop(agent)
        elif args.action == "start":
            success = navigate_home(agent)
        elif args.action == "interaction":
            success = agent.run_task1()
        elif args.action == "work":
            success = agent.navigate_and_align_to_table()
        elif args.action in GRASP_ACTIONS:
            item = GRASP_ACTIONS[args.action]
            success = agent.navigate_and_align_to_table()
            if success:
                agent.get_logger().error(
                    f"已到达作业区，但真机{item}抓取尚未完成视觉/外参/OmniPicker标定，拒绝抬臂"
                )
                success = False
    except (KeyboardInterrupt, ExternalShutdownException, RuntimeError) as exc:
        agent.get_logger().error(str(exc))
    finally:
        if rclpy.ok():
            agent.stop()
        agent.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
