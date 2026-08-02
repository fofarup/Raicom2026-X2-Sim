#!/usr/bin/env python3
"""任务1：自主导航与交互就位（15分）

国赛要求：
  裁判下达开始指令后，参赛队员通过语音指令要求机器人前往交互区I。
  机器人接收语音指令后，从出发区自主导航至交互区I。
  到达后整体进入交互区I并停止移动，身体正面朝向交互区II。

评分：
  - 整体进入交互区I: 10分
  - 停止移动+正面朝向交互区II: 5分

使用：
  python3 tasks/task1_navigation.py
  python3 tasks/task1_navigation.py --sim
"""

import argparse
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from core.mode_switch import ModeSwitch
from core.locomotion import MotionController, InputSource
from core.navigator import Navigator, INTERACT_I, INTERACT_II
from core.speech import SpeechController


class Task1Node(Node):
    def __init__(self, sim: bool = True):
        super().__init__("task1_navigation")
        self._sim = sim

        self.mode = ModeSwitch(self)
        self.input_src = InputSource(self, "task1")
        self.mc = MotionController(self, "task1")
        self.nav = Navigator(self, self.mc, sim)
        self.speech = SpeechController(self, sim)

    def run(self):
        self.get_logger().info("\n" + "=" * 50)
        self.get_logger().info("  任务1：自主导航与交互就位")
        self.get_logger().info("=" * 50)

        # 等待语音指令
        self.speech.say("请下达前往交互区I的指令。")
        cmd = self.speech.listen("请说'前往交互区'或按回车:")

        # 模式准备：JD→SD（需用户点Reset）
        self.mode.set("JD")
        self.mode.set("SD")
        self.speech.say("请在 MuJoCo 窗口点击 Reset 按钮！")
        self.speech.listen("点完 Reset 后按回车继续:")

        if not self.mode.set("LD"):
            self.get_logger().error("无法进入LD模式")
            return
        if not self.input_src.register():
            self.get_logger().error("无法注册输入源")
            return

        # 自主导航到交互区I
        self.speech.say("正在自主导航至交互区I。")
        self.get_logger().info("出发区 → 交互区-I")
        ok = self.nav.goto(*INTERACT_I, speed=0.15, timeout=45.0)

        if ok:
            self.speech.say("已到达交互区I。")
            self.get_logger().info("✅ 任务1完成")
        else:
            self.get_logger().error("导航失败")

        self.mc.stop(1.0)


def main():
    parser = argparse.ArgumentParser(description="任务1：自主导航与交互就位")
    parser.add_argument("--sim", action="store_true", default=True)
    args = parser.parse_args()

    rclpy.init()
    node = Task1Node(sim=args.sim)
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("用户中断")
    finally:
        node.mc.stop(1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
