#!/usr/bin/env python3
"""任务3：场景交互与自主服务（50分）

国赛新规则（合并旧任务3-6）：

流程：
  1. 参赛队员表达需求（三选一随机抽取）：
     ① 头部不适 → "听起来不太舒服，我去帮您拿药" → 抓药盒
     ② 口渴     → "好的，我去帮您拿杯水"       → 抓纸杯
     ③ 饥饿     → "好的，我去帮您拿点吃的"     → 抓面包
  2. 机器人语音应答
  3. 自主导航至作业区
  4. 识别并抓取对应物品（主：视觉定位 / 保底：订阅3D坐标）
  5. 语音播报结果

评分：
  - 语音应答正确: 7分
  - 物品识别正确: 10分
  - 自主到达作业区: 8分
  - 识别并抓取物品: 12分
  - 抓取成功+语音播报: 13分

使用：
  python3 tasks/task3_service.py
  python3 tasks/task3_service.py --sim
"""

import argparse
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
from core.navigator import Navigator, WORK_ZONE
from core.speech import SpeechController
from core.expression import ExpressionController
from core.grasp import GraspController

# ── 需求映射 ──────────────────────────────────────────────────
REQUESTS = {
    "头部不适": {
        "keywords": ["头", "脑袋", "头疼", "头痛", "不舒服", "隐隐作痛"],
        "object": "药盒",
        "response": "听起来不太舒服，我去帮您拿药。",
        "done": "已帮您拿到药盒。",
        "pos": (1.3, -1.2, 0.65),
    },
    "口渴": {
        "keywords": ["口", "渴", "喝水", "水", "口温"],
        "object": "杯子",
        "response": "好的，我去帮您拿杯水。",
        "done": "已帮您拿到水杯。",
        "pos": (1.5, -1.3, 0.65),
    },
    "饥饿": {
        "keywords": ["饿", "吃", "面包", "食物", "零食", "肚子饿"],
        "object": "面包",
        "response": "好的，我去帮您拿点吃的。",
        "done": "已帮您拿到面包。",
        "pos": (1.7, -1.2, 0.65),
    },
}

FEEDBACK_TEXTS = {
    "头部不适": "早上起来头就隐隐作痛，到现在还没缓过来。",
    "口渴": "我有点口渴了。",
    "饥饿": "我有点饿了。",
}


class Task3Node(Node):
    def __init__(self, sim: bool = True):
        super().__init__("task3_service")
        self._sim = sim

        self.mode = ModeSwitch(self)
        self.input_src = InputSource(self, "task3")
        self.mc = MotionController(self, "task3")
        self.nav = Navigator(self, self.mc, sim)
        self.speech = SpeechController(self, sim)
        self.expr = ExpressionController(self, sim)
        self.grasp = GraspController(self, sim)

    def _parse_request(self, text: str) -> dict:
        for intent, info in REQUESTS.items():
            for kw in info["keywords"]:
                if kw in text:
                    return info
        return None

    def run(self):
        self.get_logger().info("\n" + "=" * 50)
        self.get_logger().info("  任务3：场景交互与自主服务")
        self.get_logger().info("=" * 50)

        self.mode.set("JD")
        self.mode.set("SD")
        self.speech.say("请在 MuJoCo 窗口点击 Reset 按钮！")
        self.speech.listen("点完 Reset 后按回车继续:")

        if not self.mode.set("LD"):
            return
        if not self.input_src.register():
            return

        # 1. 接收需求
        if self._sim:
            print("\n可选需求（输入序号或文本）:")
            for i, (key, text) in enumerate(FEEDBACK_TEXTS.items(), 1):
                print(f"  {i}. ({key}) {text}")
        cmd = self.speech.listen("请表达您的需求:")

        # 模拟：数字输入转换
        if self._sim and cmd.strip().isdigit():
            idx = int(cmd.strip()) - 1
            keys = list(FEEDBACK_TEXTS.keys())
            if 0 <= idx < len(keys):
                cmd = FEEDBACK_TEXTS[keys[idx]]
        elif self._sim and not cmd.strip():
            cmd = random.choice(list(FEEDBACK_TEXTS.values()))

        # 2. 解析需求
        info = self._parse_request(cmd)
        if info is None:
            self.speech.say("抱歉，我不太理解您的需求。")
            return

        # 3. 语音应答
        self.expr.show("疑惑")
        self.speech.say(info["response"])
        self.get_logger().info(f"需求: {info['object']}")

        # 4. 自主导航到作业区
        self.speech.say("正在前往作业区。")
        self.get_logger().info(f"导航: → 作业区 ({WORK_ZONE[0]:.2f}, {WORK_ZONE[1]:.2f})")
        ok = self.nav.goto(*WORK_ZONE, speed=0.15, timeout=45.0)
        if not ok:
            self.speech.say("无法到达作业区。")
            return
        self.speech.say("已到达作业区。")

        # 5. 识别并抓取
        obj_pos = info["pos"]
        self.get_logger().info(f"目标物品: {info['object']} at {obj_pos}")

        # 视觉定位（仿真用固定坐标，真机用视觉/CV）
        if not self._sim:
            # TODO: 真机视觉定位 → obj_pos
            pass

        # 抓取（右手）
        self.grasp.grasp_object("right", list(obj_pos))

        # 6. 语音播报结果
        time.sleep(0.5)
        self.speech.say(info["done"])
        self.expr.show("快乐")

        self.get_logger().info("✅ 任务3完成")


def main():
    parser = argparse.ArgumentParser(description="任务3：场景交互与自主服务")
    parser.add_argument("--sim", action="store_true", default=True)
    args = parser.parse_args()

    rclpy.init()
    node = Task3Node(sim=args.sim)
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
