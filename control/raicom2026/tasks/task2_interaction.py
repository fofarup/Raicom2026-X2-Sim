#!/usr/bin/env python3
"""任务2：基础交互（35分）

子任务（均在交互区I执行）：
  ① 时间问答 (7分)：问"请问现在几点了" → 回答当前时间
  ② 数字颜色识别 (11分)：展示数字图片 → 识别数字+颜色
  ③ 表情切换 (8分)：悲伤/睡觉/愤怒/快乐/充电 → 面部显示
  ④ 动作执行 (9分)：挥左手/右手/左手敬礼/右手敬礼/双手打叉
     → 执行动作 + 语音"我正在执行xxx动作"

使用：
  python3 tasks/task2_interaction.py
  python3 tasks/task2_interaction.py --sim
"""

import argparse
import datetime
import os
import random
import sys
import time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from core.mode_switch import ModeSwitch
from core.locomotion import MotionController, InputSource
from core.navigator import Navigator
from core.speech import SpeechController
from core.vision import VisionController
from core.expression import ExpressionController, EXPRESSIONS

GESTURES = {
    "挥左手": "wave_left",
    "挥右手": "wave_right",
    "左手敬礼": "salute_left",
    "右手敬礼": "salute_right",
    "双手打叉": "cross_arms",
}


class Task2Node(Node):
    def __init__(self, sim: bool = True):
        super().__init__("task2_interaction")
        self._sim = sim

        self.mode = ModeSwitch(self)
        self.input_src = InputSource(self, "task2")
        self.mc = MotionController(self, "task2")
        self.nav = Navigator(self, self.mc, sim)
        self.speech = SpeechController(self, sim)
        self.vision = VisionController(self, sim)
        self.expr = ExpressionController(self, sim)

    def subtask_time(self):
        self.get_logger().info("--- ① 时间问答 ---")
        cmd = self.speech.listen("请说'请问现在几点了'或按回车:")
        now = datetime.datetime.now()
        answer = f"现在是{now.hour}点{now.minute}分。"
        self.speech.say(answer)

    def subtask_vision(self):
        self.get_logger().info("--- ② 数字颜色识别 ---")
        cmd = self.speech.listen("展示图片后说'图中的数字是什么'或按回车:")
        result = self.vision.recognize_number()
        self.speech.say(
            f"图中的数字是 {result['digit']}，"
            f"它的颜色是 {result['color']}。"
        )

    def subtask_expression(self):
        self.get_logger().info("--- ③ 表情切换 ---")
        self.speech.say(f"可以说: {', '.join(EXPRESSIONS[:5])}")
        cmd = self.speech.listen("请说出一个表情:")

        matched = next((e for e in EXPRESSIONS[:5] if e in cmd), None)
        if not matched and self._sim:
            matched = random.choice(EXPRESSIONS[:5]) if not cmd.strip() else None

        if matched:
            self.expr.show(matched)
            self.speech.say(f"好的，{matched}")
        else:
            self.speech.say(f"无法识别表情'{cmd}'")

    def subtask_gesture(self):
        self.get_logger().info("--- ④ 动作执行 ---")
        names = list(GESTURES.keys())
        self.speech.say(f"可以说: {', '.join(names)}")
        cmd = self.speech.listen("请说出一个动作:")

        matched = next((n for n in names if n in cmd), None)
        if not matched and self._sim:
            matched = random.choice(names) if not cmd.strip() else None

        if matched:
            self._do_gesture(matched)
            self.speech.say(f"我正在执行 {matched} 动作。")
        else:
            self.speech.say(f"无法执行动作'{cmd}'")

    def _do_gesture(self, name: str):
        self.get_logger().info(f"执行动作: {name} ({GESTURES[name]})")
        if self._sim:
            time.sleep(1.5)

    def run(self):
        self.get_logger().info("\n" + "=" * 50)
        self.get_logger().info("  任务2：基础交互")
        self.get_logger().info("=" * 50)

        self.mode.set("JD")
        self.mode.set("SD")
        self.speech.say("请在 MuJoCo 窗口点击 Reset 按钮！")
        self.speech.listen("点完 Reset 后按回车继续:")

        if not self.mode.set("LD"):
            return
        if not self.input_src.register():
            return

        self.expr.show("平静")

        self.subtask_time()
        self.subtask_vision()
        self.subtask_expression()
        self.subtask_gesture()

        self.speech.say("基础交互流程结束。")
        self.get_logger().info("✅ 任务2完成")


def main():
    parser = argparse.ArgumentParser(description="任务2：基础交互")
    parser.add_argument("--sim", action="store_true", default=True)
    args = parser.parse_args()

    rclpy.init()
    node = Task2Node(sim=args.sim)
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
