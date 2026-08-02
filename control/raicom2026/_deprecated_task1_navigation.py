#!/usr/bin/env python3
"""任务1：定向移动 — 出发区 → 交互区-I。

评分点：
  1) 机器人机身投影覆盖目标位置点
  2) 遥控还是自主（自主导航得分更高）

实现：
  - 基于里程计反馈的航向修正直线行走
  - 支持遥控模式（键盘 WASD）和自主导航模式
  - 到达后语音播报确认
"""

import argparse
import sys

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.node import Node

from x2_utils import (
    SimConfig,
    ModeSwitch,
    InputSource,
    MotionController,
    SpeechController,
    set_ready,
    init_robot,
)


class Task1Navigation(Node):
    def __init__(self, sim: bool = True):
        super().__init__("task1_navigation")
        self._sim = sim
        self.mode = ModeSwitch(self)
        self.input_src = InputSource(self, "task1_nav", 40)
        self.motion = MotionController(self, "task1_nav")
        self.speech = SpeechController(self, sim)

        # 订阅里程计
        from nav_msgs.msg import Odometry
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(
            Odometry, "/aima/hal/odom/state", self.motion.on_odom, qos_profile=qos
        )

    def auto_navigate(self, target_x: float, target_y: float) -> bool:
        """自主导航到目标点。"""
        self.get_logger().info(f"🚀 自主导航: → ({target_x:.2f}, {target_y:.2f})")
        if not set_ready(self.mode, self.input_src):
            return False
        return self.motion.move_toward(target_x, target_y, speed=0.18)

    def manual_control(self):
        """键盘遥控模式。"""
        print("\n" + "=" * 50)
        print("  遥控模式 — 键盘控制")
        print("  W=前进  S=后退  A=左转  D=右转  Q=退出")
        print("=" * 50 + "\n")

        if not set_ready(self.mode, self.input_src):
            return

        speed = 0.15
        try:
            import tty, termios
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
            tty.setraw(fd)
        except Exception:
            self.get_logger().warn("非 TTY 终端，使用简化输入模式")
            while True:
                cmd = input("命令(W/A/S/D/Q): ").strip().upper()
                if cmd == "Q":
                    break
                self._handle_key(cmd, speed)
            return

        try:
            while True:
                ch = sys.stdin.read(1).upper()
                if ch == "Q":
                    break
                rclpy.spin_once(self, timeout_sec=0.0)
                self._handle_key(ch, speed)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

        self.motion.stop(1.0)

    def _handle_key(self, key: str, speed: float):
        if key == "W":
            self.motion.publish(speed, 0.0, 0.0)
        elif key == "S":
            self.motion.publish(-speed, 0.0, 0.0)
        elif key == "A":
            self.motion.publish(0.0, 0.0, 0.3)
        elif key == "D":
            self.motion.publish(0.0, 0.0, -0.3)
        else:
            self.motion.publish(0.0)

    def run(self, mode: str = "auto", target: tuple = None):
        if target is None:
            target = SimConfig.INTERACT_I

        self.get_logger().info(
            f"\n{'='*50}\n"
            f"  任务1：定向移动  出发区 → 交互区-I\n"
            f"  目标: ({target[0]:.2f}, {target[1]:.2f})\n"
            f"  模式: {'自主导航' if mode == 'auto' else '遥控'}\n"
            f"{'='*50}"
        )

        if mode == "manual":
            self.manual_control()
        else:
            ok = self.auto_navigate(*target)
            if ok:
                self.speech.say("已到达交互区-I，等待下一步指令。")
            else:
                self.get_logger().error("导航失败！")

        self.motion.stop(1.0)
        self.get_logger().info("任务1结束。")


def main():
    parser = argparse.ArgumentParser(description="任务1：定向移动")
    parser.add_argument(
        "--mode", choices=["auto", "manual"], default="auto",
        help="auto=自主导航 manual=键盘遥控"
    )
    parser.add_argument("--target-x", type=float, default=SimConfig.INTERACT_I[0])
    parser.add_argument("--target-y", type=float, default=SimConfig.INTERACT_I[1])
    parser.add_argument("--sim", action="store_true", default=True,
                        help="仿真模式（默认）")
    args = parser.parse_args()

    rclpy.init()
    node = Task1Navigation(sim=args.sim)
    try:
        node.run(mode=args.mode, target=(args.target_x, args.target_y))
    except KeyboardInterrupt:
        node.get_logger().info("用户中断")
    finally:
        node.motion.stop(1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
