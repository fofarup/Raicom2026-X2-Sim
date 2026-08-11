#!/usr/bin/env python3
"""Real-X2 grasp interface probe and protected OmniPicker test.

This is intentionally not a complete grasp planner.  It verifies the official
RGB-D, arm and hand interfaces and refuses autonomous grasping until calibration
is explicitly complete in the real profile.
"""

from __future__ import annotations

import argparse
import os
import time

import rclpy
from aimdk_msgs.msg import (
    HandCommand, HandCommandArray, HandStateArray, HandType, JointStateArray,
    MessageHeader,
)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from robot_profile import load_robot_profile


class RealGraspInterface(Node):
    def __init__(self) -> None:
        super().__init__("raicom_real_grasp_interface")
        self.profile = load_robot_profile()
        hw = self.profile["hardware"]
        self.seen = {
            name: 0.0
            for name in ("rgb", "depth", "rgb_info", "depth_info", "arm", "hand")
        }
        self.create_subscription(Image, hw["rgb_topic"], lambda _: self._mark("rgb"), qos_profile_sensor_data)
        self.create_subscription(Image, hw["depth_topic"], lambda _: self._mark("depth"), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, hw["camera_info_topic"], lambda _: self._mark("rgb_info"), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, hw["depth_camera_info_topic"], lambda _: self._mark("depth_info"), qos_profile_sensor_data)
        self.create_subscription(JointStateArray, hw["arm_state_topic"], lambda _: self._mark("arm"), qos_profile_sensor_data)
        self.create_subscription(HandStateArray, hw["gripper_state_topic"], lambda _: self._mark("hand"), qos_profile_sensor_data)
        self.hand_pub = self.create_publisher(HandCommandArray, hw["gripper_command_topic"], 10)
        self.get_logger().info("真机RGB-D/OmniPicker接口适配器已启动（默认只读）")

    def _mark(self, name: str) -> None:
        self.seen[name] = time.monotonic()

    def preflight(self, timeout_s: float = 8.0) -> bool:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(self.seen.values()):
                break
        missing = [name for name, stamp in self.seen.items() if not stamp]
        if missing:
            self.get_logger().error("未收到接口数据: " + ", ".join(missing))
            return False
        self.get_logger().info("RGB、深度、双CameraInfo、机械臂和OmniPicker状态均已收到")
        return True

    def publish_gripper(self, position: float) -> bool:
        if os.environ.get("RAICOM_CONFIRM_MC_STOPPED") != "YES":
            self.get_logger().error(
                "官方要求直接手部控制前停止PC1原生MC；未设置RAICOM_CONFIRM_MC_STOPPED=YES，拒绝发布"
            )
            return False
        message = HandCommandArray()
        message.header = MessageHeader()
        message.header.frame_id = "raicom_real_omnipicker_test"
        message.left_hand_type = HandType(value=HandType.NONE)
        message.right_hand_type = HandType(value=HandType.CLAW)
        command = HandCommand()
        command.name = "right_hand"
        command.position = float(position)
        command.velocity = 0.2
        command.acceleration = 0.2
        command.deceleration = 0.2
        command.effort = 0.2
        message.right_hands = [command]
        deadline = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < deadline:
            message.header.stamp = self.get_clock().now().to_msg()
            self.hand_pub.publish(message)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)
        return True


def main() -> None:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--preflight", action="store_true")
    group.add_argument("--open-right", action="store_true")
    group.add_argument("--close-right", action="store_true")
    args = parser.parse_args()
    rclpy.init()
    node = RealGraspInterface()
    try:
        if args.open_right:
            success = node.publish_gripper(1.0)
        elif args.close_right:
            success = node.publish_gripper(0.0)
        else:
            success = node.preflight()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
