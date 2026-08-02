#!/usr/bin/env python3
"""任务2：动作执行 — 在交互区-I 做目标舞蹈动作。

评分点：
  1) 动作执行的稳定性
  2) 动作执行的准确性

实现：
  - 舞蹈动作通过灵创平台 (LinkCraft) 配置和调用
  - 本脚本提供框架：调用预置动作、等待完成、确认姿态
  - 具体动作名称需根据灵创平台配置填入 DANCE_ACTIONS

使用：
  在交互区-I 确保机器人处于 LD 模式后：
    python3 task2_dance.py
    或指定动作序列：
    python3 task2_dance.py --actions "wave,dance_01,bow"
"""

import argparse
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.node import Node

from x2_utils import (
    ModeSwitch,
    InputSource,
    MotionController,
    ExpressionController,
    SpeechController,
    set_ready,
    init_robot,
)


# ── 灵创平台预置动作名称 ──────────────────────────────────
# 在灵创平台 (https://linkcraft.agibot.com) 中录制/配置的
# 动作名称。参赛者需根据实际配置填入。
DANCE_ACTIONS = [
    "dance_01",   # 示例：舞蹈动作1 — 需要在灵创配置
    "dance_02",   # 示例：舞蹈动作2
    "dance_03",   # 示例：舞蹈动作3
]


class Task2Dance(Node):
    def __init__(self, sim: bool = True):
        super().__init__("task2_dance")
        self._sim = sim

        tools = init_robot(self, sim)
        self.mode = tools["mode"]
        self.input_src = tools["input_src"]
        self.motion = tools["motion"]
        self.expr = tools["expr"]
        self.speech = tools["speech"]

        from nav_msgs.msg import Odometry
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(
            Odometry, "/aima/hal/odom/state", self.motion.on_odom, qos_profile=qos
        )

    def play_action(self, action_name: str) -> bool:
        """调用灵创平台预置动作。

        TODO: 替换为实际的灵创平台 API 调用。
        参考: /aimdk_5Fmsgs/srv/ 下的预置动作服务
        """
        self.get_logger().info(f"🎬 执行动作: {action_name}")
        self.speech.say(f"开始执行 {action_name}")

        if self._sim:
            # 仿真模式：模拟动作执行（持续 2 秒）
            self.get_logger().info(f"[仿真] 机器人正在做 {action_name} ...")
            time.sleep(2.0)
            self.get_logger().info(f"[仿真] {action_name} 完成")
        else:
            # 真机：调用灵创/预置动作 API
            # TODO: 替换为实际 API
            self._call_linkcraft_action(action_name)

        return True

    def _call_linkcraft_action(self, action_name: str):
        """真机调用灵创平台动作（待实现）。"""
        # 示例：可能通过 ROS 2 service 调用
        # client = self.create_client(..., "/aimdk_5Fmsgs/srv/PlayMotion")
        # req.action_name = action_name
        # ...
        self.get_logger().warn(f"灵创动作 API 尚未实现: {action_name}")
        time.sleep(2.0)

    def run(self, actions: list = None):
        if actions is None:
            actions = DANCE_ACTIONS

        self.get_logger().info(
            f"\n{'='*50}\n"
            f"  任务2：动作执行\n"
            f"  动作序列: {actions}\n"
            f"{'='*50}"
        )

        if not set_ready(self.mode, self.input_src):
            self.get_logger().error("无法进入运动模式")
            return

        self.expr.show("快乐")

        for i, action in enumerate(actions):
            self.get_logger().info(f"--- 动作 {i+1}/{len(actions)}: {action} ---")
            self.play_action(action)
            # 动作间短暂稳定
            time.sleep(0.5)
            rclpy.spin_once(self, timeout_sec=0.0)

        self.speech.say("舞蹈动作执行完毕。")
        self.expr.show("平静")
        self.get_logger().info("任务2结束。")


def main():
    parser = argparse.ArgumentParser(description="任务2：动作执行（舞蹈）")
    parser.add_argument(
        "--actions", type=str, default=None,
        help="逗号分隔的动作名称列表，如 'wave,bow,dance_01'"
    )
    parser.add_argument("--sim", action="store_true", default=True)
    args = parser.parse_args()

    actions = None
    if args.actions:
        actions = [a.strip() for a in args.actions.split(",")]

    rclpy.init()
    node = Task2Dance(sim=args.sim)
    try:
        node.run(actions)
    except KeyboardInterrupt:
        node.get_logger().info("用户中断")
    finally:
        node.motion.stop(1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
