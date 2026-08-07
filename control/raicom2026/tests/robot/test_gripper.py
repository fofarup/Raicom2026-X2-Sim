#!/usr/bin/env python3
"""真机夹爪控制测试 —— 开/合/半开。

用法（在 PC2 真机上）：
  python3 test_gripper.py                  # 交互模式
  python3 test_gripper.py --left open      # 左爪张开
  python3 test_gripper.py --left close     # 左爪闭合
  python3 test_gripper.py --right open     # 右爪张开
  python3 test_gripper.py --both open      # 双爪张开
  python3 test_gripper.py --test           # 自动: 张开→闭合→张开

依赖：ROS 2 Humble + AimDK SDK
  话题: /aima/hal/joint/hand/command (HandCommandArray)
  关节: left_claw_joint / right_claw_joint
  位置: 0.0=合(close)  1.0=开(open)  0.5=半开

  仿真模型没有 OmniPicker 夹爪关节，本脚本仅真机有效。
"""

import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from aimdk_msgs.msg import HandCommand, HandCommandArray, HandType


class GripperTester(Node):
    def __init__(self):
        super().__init__("test_gripper")
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self._pub = self.create_publisher(
            HandCommandArray, "/aima/hal/joint/hand/command", qos)
        self.get_logger().info("夹爪测试节点就绪")

    def grip(self, hand: str, position: float):
        """hand: 'left' | 'right' | 'both', position: 0.0(合)~1.0(开)"""
        hands = ["left", "right"] if hand == "both" else [hand]
        if hand not in ("left", "right", "both"):
            self.get_logger().error(f"hand 必须为 left/right/both, 收到: {hand}")
            return

        msg = HandCommandArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.seq += 1

        for h in hands:
            cmd = HandCommand()
            cmd.name = f"{h}_claw_joint"
            cmd.pos = position
            cmd.vel = 0.05
            cmd.tor = 0.5

            if h == "left":
                msg.left_hand_type = HandType(value=0x2)
                msg.left_hands.append(cmd)
            else:
                msg.right_hand_type = HandType(value=0x2)
                msg.right_hands.append(cmd)

        self._pub.publish(msg)
        desc = "开" if position > 0.5 else ("半开" if position > 0.1 else "合")
        self.get_logger().info(f"夹爪 {hand} → {desc} (pos={position:.1f})")


def main():
    rclpy.init()
    t = GripperTester()

    if "--test" in sys.argv:
        print("=== 自动测试: 双爪开 → 合 → 开 ===")
        for step, (desc, pos) in enumerate(
            [("张开", 1.0), ("闭合", 0.0), ("张开", 1.0)]):
            print(f"\n步骤{step+1}: {desc}")
            t.grip("both", pos)
            time.sleep(2.0)
        print("\n完成。")

    elif len(sys.argv) >= 3:
        for i, arg in enumerate(sys.argv):
            if arg in ("--left", "--right", "--both"):
                hand = arg.lstrip("--")
                action = sys.argv[i + 1] if i + 1 < len(sys.argv) else "open"
                pos = {"open": 1.0, "close": 0.0, "half": 0.5}.get(action, 1.0)
                t.grip(hand, pos)
                break

    else:
        print("=== 真机夹爪测试 ===")
        print("命令: left/right/both open/close/half | test | q")
        print()

        while True:
            try:
                cmd = input(">>> ").strip().split()
                if not cmd:
                    continue
                if cmd[0] == "q":
                    break
                elif cmd[0] == "test":
                    for pos, desc in [(1.0, "开"), (0.0, "合"), (1.0, "开")]:
                        t.grip("both", pos)
                        time.sleep(2.0)
                elif len(cmd) >= 2 and cmd[0] in ("left", "right", "both"):
                    pos = {"open": 1.0, "close": 0.0, "half": 0.5}.get(cmd[1], 1.0)
                    t.grip(cmd[0], pos)
                else:
                    print("? 用: left open | right close | both half | test | q")
            except (EOFError, KeyboardInterrupt):
                break

    t.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
