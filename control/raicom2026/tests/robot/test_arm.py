#!/usr/bin/env python3
"""真机手臂控制测试 —— 逐个关节/姿态验证。

用法（在 PC2 真机上）：
  python3 test_arm.py                    # 交互模式
  python3 test_arm.py --ready            # 回到预备姿态
  python3 test_arm.py --pose 挥左手      # 执行指定手势
  python3 test_arm.py --joint left_shoulder_pitch --angle -0.5  # 单关节

依赖：ROS 2 Humble + AimDK SDK
  话题: /mc/upper_body_command (UpperBodyCommandArray)
  模式: UPPERBODY_REMOTE_SPLIT (US)
  需要先切换到 US 模式: competition_node 或手工通过 SetMcAction
"""

import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from aimdk_msgs.msg import JointCommand, JointCommandArray, UpperBodyCommandArray

ARM_JOINTS = [
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint",
    "left_wrist_yaw_joint", "left_wrist_pitch_joint", "left_wrist_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_yaw_joint", "right_wrist_pitch_joint", "right_wrist_roll_joint",
]

# 预备姿态（安全位置，双臂自然下垂微前伸）
READY = [
    -0.35, 0.45, 0.0, -1.00, 0.0, 0.15, 0.0,   # 左臂
    -0.35, -0.45, 0.0, -1.00, 0.0, 0.15, 0.0,  # 右臂
]

# 五种比赛手势
POSES = {
    "预备": READY,
    "挥左手": [-1.32, 0.87, 0.28, -1.40, -0.64, 0.00, -0.12] + READY[7:],
    "挥右手": READY[:7] + [-1.32, -0.87, -0.28, -1.40, 0.64, 0.00, 0.12],
    "左手敬礼": [-1.37, 1.57, 0.04, -2.20, 1.38, 0.15, 0.0] + READY[7:],
    "右手敬礼": READY[:7] + [-1.37, -1.57, -0.04, -2.20, -1.38, 0.15, 0.0],
    "双手打叉": [
        -0.92, -0.061, -0.679, -1.767, 0.00, 0.15, 0.00,
        -1.04, 0.061, 0.660, -1.196, 0.12, -0.158, -0.114,
    ],
}


class ArmTester(Node):
    def __init__(self):
        super().__init__("test_arm")
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self._pub = self.create_publisher(
            UpperBodyCommandArray, "/mc/upper_body_command", qos)
        self.get_logger().info("手臂测试节点就绪（需 US 模式: UPPERBODY_REMOTE_SPLIT）")

    def move(self, positions: list[float], duration: float = 1.5, interval: float = 0.02):
        """把 14 个关节角平滑发送到 /mc/upper_body_command。"""
        if len(positions) != 14:
            self.get_logger().error(f"需要14个关节角，收到{len(positions)}个")
            return

        steps = max(1, int(duration / interval))
        for step in range(steps + 1):
            t = step / max(steps, 1)
            msg = UpperBodyCommandArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.command_source = "rc"

            for i, name in enumerate(ARM_JOINTS):
                cmd = JointCommand()
                cmd.name = name
                cmd.pos = positions[i]
                cmd.vel = 0.0
                cmd.tor = 0.0
                cmd.kp = 80.0
                cmd.kd = 2.0
                msg.joint_cmds.append(cmd)

            self._pub.publish(msg)
            if step < steps:
                time.sleep(interval)

        self.get_logger().info(
            f"移动完成: {[f'{v:.2f}' for v in positions[:4]]}...")


def main():
    rclpy.init()
    tester = ArmTester()

    if "--ready" in sys.argv:
        print(">>> 回到预备姿态")
        tester.move(READY, duration=1.5)

    elif "--pose" in sys.argv:
        idx = sys.argv.index("--pose")
        name = sys.argv[idx + 1] if idx + 1 < len(sys.argv) else "预备"
        if name in POSES:
            print(f">>> 执行: {name}")
            tester.move(POSES[name], duration=2.0)
        else:
            print(f"未知手势: {name}, 可用: {list(POSES.keys())}")

    elif "--joint" in sys.argv:
        try:
            ji = sys.argv.index("--joint")
            ai = sys.argv.index("--angle")
            jname = sys.argv[ji + 1]
            angle = float(sys.argv[ai + 1])
        except (ValueError, IndexError):
            print("用法: --joint left_shoulder_pitch --angle -0.5")
            sys.exit(1)

        if jname not in ARM_JOINTS:
            print(f"未知关节: {jname}")
            print(f"可用: {ARM_JOINTS}")
            sys.exit(1)

        pos = list(READY)
        pos[ARM_JOINTS.index(jname)] = angle
        print(f">>> {jname} = {angle}")
        tester.move(pos, duration=1.0)

    else:
        print("=== 真机手臂测试 ===")
        print("关节:", ", ".join(ARM_JOINTS[:4]), "... (共14个)")
        print("手势:", ", ".join(POSES.keys()))
        print()
        print("命令: ready | pose <名> | joint <关节> <角度> | list | q")
        print()

        while True:
            try:
                cmd = input(">>> ").strip()
                if not cmd:
                    continue
                parts = cmd.split()
                if parts[0] == "q":
                    break
                elif parts[0] == "ready":
                    tester.move(READY, duration=1.5)
                elif parts[0] == "pose" and len(parts) >= 2:
                    name = parts[1]
                    if name in POSES:
                        tester.move(POSES[name], duration=2.0)
                    else:
                        print(f"未知: {name}")
                elif parts[0] == "joint" and len(parts) >= 3:
                    jname, angle = parts[1], float(parts[2])
                    if jname in ARM_JOINTS:
                        pos = list(READY)
                        pos[ARM_JOINTS.index(jname)] = angle
                        tester.move(pos, duration=1.0)
                    else:
                        print(f"未知关节: {jname}")
                elif parts[0] == "list":
                    print("手势:", ", ".join(POSES.keys()))
                    print("关节:", ", ".join(ARM_JOINTS))
                else:
                    print("? 用 ready/pose/joint/list/q")
            except (EOFError, KeyboardInterrupt):
                break

    tester.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
