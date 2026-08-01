#!/usr/bin/env python3
"""任务4：场景交互（仅限全国总决赛）。

流程：
  1. 机器人识别目标服务对象（排除裁判）
  2. 主动发起语音对话："今天状态怎么样？"
  3. 面部显示"疑惑"表情
  4. 服务对象反馈不适（头部/手臂/腰部/肚子）
  5. 机器人匹配康复训练方案 → 语音说明 + 表情"平静-卖萌"

康复训练方案（预置）：
  ① 头部疼痛 → 头部舒缓操
  ② 手臂酸痛 → 手臂拉伸操
  ③ 腰部疼痛 → 腰部放松操
  ④ 肚子疼痛 → 腹部按摩操

交互模式：
  - 仿真: 键盘模拟语音输入
  - 真机: TTS + ASR
"""

import argparse
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

# ── 康复训练方案映射 ──────────────────────────────────────────
REHAB_PLANS = {
    "头部": {
        "keywords": ["头", "脑袋", "头疼", "头痛"],
        "action": "head_relief",       # 灵创动作名
        "desc": "头部舒缓操",
        "tts": "如果最近头部难受，可以尝试和我一起做这个动作，"
               "能在一定程度上帮你缓解症状。",
    },
    "手臂": {
        "keywords": ["手臂", "胳膊", "手", "酸"],
        "action": "arm_stretch",
        "desc": "手臂拉伸操",
        "tts": "如果最近手臂酸痛，可以尝试和我一起做这个动作，"
               "能在一定程度上帮你缓解症状。",
    },
    "腰部": {
        "keywords": ["腰", "腰椎", "酸疼", "酸痛"],
        "action": "waist_relief",
        "desc": "腰部放松操",
        "tts": "如果最近腰部酸疼，可以尝试和我一起做这个动作，"
               "能在一定程度上帮你缓解症状。",
    },
    "肚子": {
        "keywords": ["肚子", "腹部", "胃", "胀", "疼"],
        "action": "belly_relief",
        "desc": "腹部按摩操",
        "tts": "如果最近肚子不舒服，可以尝试和我一起做这个动作，"
               "能在一定程度上帮你缓解症状。",
    },
}

# ── 服务对象反馈语料 ─────────────────────────────────────────
FEEDBACK_TEXTS = {
    "头部": "早上起来头就隐隐作痛，到现在还没缓过来。",
    "手臂": "昨天干活多了一点，今天胳膊酸酸的。",
    "腰部": "这两天腰总是酸酸疼疼的，坐久了更难受。",
    "肚子": "肚子有点胀胀的，感觉挺难受的。",
}


class Task4Scenario(Node):
    def __init__(self, sim: bool = True):
        super().__init__("task4_scenario")
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

    def _identify_target(self) -> bool:
        """识别目标服务对象（非裁判）。
        仿真模式：直接确认。
        真机：通过视觉/语音确认。"""
        self.get_logger().info("正在识别目标服务对象...")
        self.expr.show("疑惑")

        if self._sim:
            self.speech.say("已识别服务对象。")
            return True
        else:
            # TODO: 真机视觉识别服务对象（人脸/姿势）
            time.sleep(1.0)
            return True

    def _match_rehab(self, feedback: str) -> dict:
        """根据反馈文本匹配康复方案。"""
        for plan_name, plan in REHAB_PLANS.items():
            for kw in plan["keywords"]:
                if kw in feedback:
                    self.get_logger().info(f"匹配康复方案: {plan_name} ({plan['desc']})")
                    return plan
        return None

    def _play_rehab_action(self, plan: dict):
        """播放康复训练动作（灵创平台调用）。"""
        action = plan["action"]
        self.get_logger().info(f"🎬 执行康复动作: {action} ({plan['desc']})")
        if self._sim:
            time.sleep(2.0)
        else:
            # TODO: 真机调用灵创预置动作
            time.sleep(2.0)

    def run(self):
        self.get_logger().info(
            f"\n{'='*50}\n"
            f"  任务4：场景交互\n"
            f"  {'仿真模式' if self._sim else '真机模式'}\n"
            f"{'='*50}"
        )

        if not set_ready(self.mode, self.input_src):
            self.get_logger().error("无法进入运动模式")
            return

        # 第1步：识别服务对象
        if not self._identify_target():
            self.speech.say("未能识别服务对象。")
            return

        # 第2步：主动对话
        self.expr.show("疑惑")
        self.speech.say("今天状态怎么样？")

        # 第3步：接收反馈
        feedback = ""
        if self._sim:
            print("\n可选反馈（输入序号或文本）:")
            for i, (key, text) in enumerate(FEEDBACK_TEXTS.items(), 1):
                print(f"  {i}. ({key}) {text}")
            feedback = self.speech.listen("请输入序号或描述不适:")
            try:
                idx = int(feedback.strip()) - 1
                keys = list(FEEDBACK_TEXTS.keys())
                if 0 <= idx < len(keys):
                    feedback = FEEDBACK_TEXTS[keys[idx]]
            except ValueError:
                pass  # 直接使用用户输入
        else:
            feedback = self.speech.listen("请说出您的感受:")

        # 第4步：匹配康复方案
        plan = self._match_rehab(feedback)
        if plan is None:
            self.speech.say("抱歉，我不太理解您的不适情况，能再说一遍吗？")
            feedback = self.speech.listen("请再说一遍:")
            plan = self._match_rehab(feedback)

        if plan is None:
            # 默认匹配
            self.speech.say("我为您推荐一套放松动作。")
            plan = REHAB_PLANS["头部"]

        # 第5步：执行康复训练
        self.expr.show("平静-卖萌")
        self.speech.say(plan["tts"])
        self._play_rehab_action(plan)

        self.speech.say("康复训练完成，希望对您有帮助。")
        self.expr.show("平静")
        self.get_logger().info("任务4结束。")


def main():
    parser = argparse.ArgumentParser(description="任务4：场景交互")
    parser.add_argument("--sim", action="store_true", default=True)
    args = parser.parse_args()

    rclpy.init()
    node = Task4Scenario(sim=args.sim)
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("用户中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
