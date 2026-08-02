#!/usr/bin/env python3
"""任务3：基础交互（仅限全国总决赛）。

子任务：
  ① 时间问答："请问现在几点了" → 回答当前时间
  ② 数字颜色识别：展示数字图片 → 回答数字+颜色
  ③ 表情切换：悲伤/睡觉/愤怒/快乐/充电 → 机器人做对应表情
  ④ 动作执行：挥左手/挥右手/左手敬礼/右手敬礼/双手打叉
     → 机器人执行动作 + 语音回复"我正在执行 xxx 动作"

交互模式：
  - 仿真模式：键盘输入模拟语音
  - 真机模式：TTS + ASR
"""

import argparse
import datetime
import random
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
    init_robot,
    set_ready,
)

# ── 可用表情 ──────────────────────────────────────────────────
EXPRESSIONS = ["悲伤", "睡觉", "愤怒", "快乐", "充电"]

# ── 可用动作 ──────────────────────────────────────────────────
GESTURES = {
    "挥左手":   "wave_left",
    "挥右手":   "wave_right",
    "左手敬礼": "salute_left",
    "右手敬礼": "salute_right",
    "双手打叉": "cross_arms",
}

# ── 模拟数字图片（真机用 CV 模型） ───────────────────────────
MOCK_NUMBERS = [
    {"digit": 5, "color": "红色"},
    {"digit": 3, "color": "蓝色"},
    {"digit": 8, "color": "绿色"},
    {"digit": 0, "color": "黄色"},
    {"digit": 7, "color": "白色"},
]


class Task3Interaction(Node):
    def __init__(self, sim: bool = True):
        super().__init__("task3_interaction")
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

    # ── ① 时间问答 ─────────────────────────────────────────
    def subtask_time(self):
        """回答当前时间。"""
        self.get_logger().info("--- ① 时间问答 ---")
        cmd = self.speech.listen("请向机器人提问（输入 '几点了' 或回车）:")
        now = datetime.datetime.now()
        answer = f"现在是{now.hour}点{now.minute}分。"
        self.speech.say(answer)

    # ── ② 数字颜色识别 ─────────────────────────────────────
    def subtask_vision(self):
        """识别图片中的数字和颜色。仿真用随机模拟，真机用 CV。"""
        self.get_logger().info("--- ② 数字颜色识别 ---")
        cmd = self.speech.listen("请展示图片，输入 '图中的数字是什么' 或回车:")

        if self._sim:
            card = random.choice(MOCK_NUMBERS)
            self.speech.say(
                f"图中的数字是 {card['digit']}，"
                f"数字颜色是 {card['color']}。"
            )
        else:
            # TODO: 真机调用摄像头 + CV 识别
            self.speech.say("正在识别图片...")
            time.sleep(1.0)
            self.speech.say("图中的数字是 5，它的颜色是红色。")

    # ── ③ 表情切换 ─────────────────────────────────────────
    def subtask_expression(self):
        """接收表情指令，机器人做对应表情。"""
        self.get_logger().info("--- ③ 表情切换 ---")
        self.speech.say(f"可以说以下表情：{', '.join(EXPRESSIONS)}")
        cmd = self.speech.listen("请说出一个表情:")

        matched = None
        for expr in EXPRESSIONS:
            if expr in cmd:
                matched = expr
                break

        if matched:
            self.expr.show(matched)
            self.speech.say(f"好的，{matched}")
        elif self._sim and (cmd.isdigit() or cmd == ""):
            # 模拟：随机表情
            expr = random.choice(EXPRESSIONS)
            self.expr.show(expr)
            self.speech.say(f"好的，{expr}")
        else:
            self.speech.say(f"抱歉，我无法识别 '{cmd}' 这个表情。")

    # ── ④ 动作执行 ─────────────────────────────────────────
    def subtask_gesture(self):
        """接收动作指令，机器人执行并语音确认。"""
        self.get_logger().info("--- ④ 动作执行 ---")
        gestures_list = list(GESTURES.keys())
        self.speech.say(f"可以说以下动作：{', '.join(gestures_list)}")
        cmd = self.speech.listen("请说出一个动作:")

        matched = None
        for name in gestures_list:
            if name in cmd:
                matched = name
                break

        if not matched and self._sim and (cmd.isdigit() or cmd == ""):
            matched = random.choice(gestures_list)

        if matched:
            # 执行动作
            self._execute_gesture(matched)
            self.speech.say(f"我正在执行 {matched} 动作。")
        else:
            self.speech.say(f"抱歉，我无法执行 '{cmd}' 这个动作。")

    def _execute_gesture(self, gesture_name: str):
        """执行指定动作（仿真模拟）。"""
        action_id = GESTURES[gesture_name]
        self.get_logger().info(f"🎬 执行动作: {gesture_name} ({action_id})")
        if self._sim:
            time.sleep(1.5)
        else:
            # TODO: 真机调用关节控制或灵创预置动作
            time.sleep(1.5)

    # ── 主流程 ─────────────────────────────────────────────
    def run(self, subtasks: str = "all"):
        self.get_logger().info(
            f"\n{'='*50}\n"
            f"  任务3：基础交互\n"
            f"  模式: {'仿真(键盘模拟)' if self._sim else '真机'}\n"
            f"{'='*50}"
        )

        if not set_ready(self.mode, self.input_src):
            self.get_logger().error("无法进入运动模式")
            return

        # 重置表情
        self.expr.show("平静")

        if subtasks == "all" or "time" in subtasks:
            self.subtask_time()
        if subtasks == "all" or "vision" in subtasks:
            self.subtask_vision()
        if subtasks == "all" or "expression" in subtasks:
            self.subtask_expression()
        if subtasks == "all" or "gesture" in subtasks:
            self.subtask_gesture()

        self.speech.say("基础交互流程结束。")
        self.get_logger().info("任务3结束。")


def main():
    parser = argparse.ArgumentParser(description="任务3：基础交互")
    parser.add_argument(
        "--subtasks", type=str, default="all",
        help="all / time,vision,expression,gesture"
    )
    parser.add_argument("--sim", action="store_true", default=True)
    args = parser.parse_args()

    rclpy.init()
    node = Task3Interaction(sim=args.sim)
    try:
        node.run(args.subtasks)
    except KeyboardInterrupt:
        node.get_logger().info("用户中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
