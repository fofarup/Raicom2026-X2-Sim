#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OmniPicker 双夹爪控制——完整版。

任务目标：
  通过智元灵犀 X2 AimDK ROS 2 接口控制左右夹爪打开/闭合。

支持命令：
  python3 omnipicker_hand.py --publish open left
  python3 omnipicker_hand.py --publish close left
  python3 omnipicker_hand.py --publish open right
  python3 omnipicker_hand.py --publish close right

安全：不带 --publish 时不发布命令。
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from aimdk_msgs.msg import HandCommand, HandCommandArray, HandType, MessageHeader


COMMAND_TOPIC = "/aima/hal/joint/hand/command"
LEFT_JOINT_NAME = "left_claw_joint"
RIGHT_JOINT_NAME = "right_claw_joint"
PUBLISH_FREQUENCY_HZ = 50.0
PUBLISH_DURATION_SECONDS = 2.0
LEFT_HAND_TYPE = HandType(value=0x2)
RIGHT_HAND_TYPE = HandType(value=0x2)


def create_hand_command(joint_name: str, target_position: float) -> HandCommand:
    """TODO 1：创建单侧夹爪命令。"""
    cmd = HandCommand()
    cmd.name = joint_name
    cmd.position = float(target_position)
    cmd.velocity = 0.3
    cmd.acceleration = 0.5
    cmd.deceleration = 0.5
    cmd.effort = 0.0
    return cmd


def build_hand_message(hand: str, target_position: float) -> HandCommandArray:
    """TODO 2：组装单侧夹爪的 HandCommandArray 消息。

    hand: "left" 或 "right"
    target_position: 0.0（闭合）或 1.0（打开）
    """
    msg = HandCommandArray()
    msg.header = MessageHeader()
    msg.header.stamp = rclpy.clock.Clock().now().to_msg()

    if hand == "left":
        msg.left_hand_type = LEFT_HAND_TYPE
        msg.left_hands.append(
            create_hand_command(LEFT_JOINT_NAME, target_position)
        )
        msg.right_hand_type = HandType(value=0x0)  # 无设备
    else:
        msg.left_hand_type = HandType(value=0x0)  # 无设备
        msg.right_hand_type = RIGHT_HAND_TYPE
        msg.right_hands.append(
            create_hand_command(RIGHT_JOINT_NAME, target_position)
        )

    return msg


class OmniPickerNode(Node):
    """双夹爪控制 ROS 2 节点。"""

    def __init__(self):
        super().__init__("omnipicker_hand")
        command_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            HandCommandArray, COMMAND_TOPIC, command_qos
        )

    def publish_command(self, hand: str, target_position: float):
        """TODO 3：按指定频率和时长持续发布夹爪命令。"""
        period = 1.0 / PUBLISH_FREQUENCY_HZ
        deadline = time.monotonic() + PUBLISH_DURATION_SECONDS
        frame_count = 0

        self.get_logger().info(
            f"控制 {'左' if hand == 'left' else '右'}夹爪 "
            f"{'打开' if target_position > 0.5 else '闭合'} "
            f"(pos={target_position:.1f}) 持续 {PUBLISH_DURATION_SECONDS}s"
        )

        while time.monotonic() < deadline:
            msg = build_hand_message(hand, target_position)
            self.publisher.publish(msg)
            frame_count += 1
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

        self.get_logger().info(f"发布完成，共 {frame_count} 帧。")


def parse_arguments():
    parser = argparse.ArgumentParser(description="OmniPicker 双夹爪控制")
    parser.add_argument(
        "--publish", action="store_true",
        help="允许发布夹爪控制命令（安全开关）"
    )
    parser.add_argument(
        "action", choices=("open", "close"),
        help="夹爪动作：open 打开 / close 闭合"
    )
    parser.add_argument(
        "hand", choices=("left", "right"),
        help="目标夹爪：left 左 / right 右"
    )
    return parser.parse_args()


def main():
    args = parse_arguments()

    if not args.publish:
        print("未指定 --publish，程序不会发布控制命令。")
        print("确认安全后使用 --publish 运行。")
        return

    target_position = 1.0 if args.action == "open" else 0.0

    rclpy.init()
    node = OmniPickerNode()
    try:
        node.publish_command(args.hand, target_position)
    except KeyboardInterrupt:
        print("已停止控制程序。")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
