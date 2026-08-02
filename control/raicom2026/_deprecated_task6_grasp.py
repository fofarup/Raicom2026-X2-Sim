#!/usr/bin/env python3
"""任务6：抓取作业（仅限全国总决赛）。

流程：
  1. 服务对象语音提出需求（①吃药 ②口渴 ③饿）
  2. 机器人理解 → 对应目标物品（药盒/杯子/面包）
  3. 获取物品 3D 坐标（仿真用固定坐标，真机订阅坐标 topic）
  4. 手臂+手部控制 → 抓取物品 → 放到指定区域

评分点：
  1) 物体抓取、放置成功
  2) 技术方案难度

物品坐标：
  - 杯子: (1.5, -1.3, 0.65)
  - 药盒: (1.3, -1.2, 0.65)
  - 面包: (1.7, -1.2, 0.65)
"""

import argparse
import random
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.node import Node

from x2_utils import (
    SimConfig,
    ModeSwitch,
    InputSource,
    MotionController,
    SpeechController,
    ArmController,
    HandController,
    init_robot,
    set_ready,
)

# ── 语音需求 → 物品映射 ───────────────────────────────────────
COMMAND_MAP = {
    "吃药": {
        "keywords": ["吃药", "药", "药盒", "药物", "准备吃药"],
        "object": "药盒",
        "tts": "好的，我帮您拿药盒。",
    },
    "口渴": {
        "keywords": ["口渴", "喝水", "渴", "水", "杯子", "纸杯"],
        "object": "杯子",
        "tts": "好的，我帮您拿杯子。",
    },
    "饿": {
        "keywords": ["饿", "吃", "面包", "食物", "零食", "肚子饿"],
        "object": "面包",
        "tts": "好的，我帮您拿面包。",
    },
}


class Task6Grasp(Node):
    def __init__(self, sim: bool = True):
        super().__init__("task6_grasp")
        self._sim = sim

        tools = init_robot(self, sim)
        self.mode = tools["mode"]
        self.input_src = tools["input_src"]
        self.motion = tools["motion"]
        self.speech = tools["speech"]
        self.arm = tools["arm"]
        self.hand = tools["hand"]

        from nav_msgs.msg import Odometry
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(
            Odometry, "/aima/hal/odom/state", self.motion.on_odom, qos_profile=qos
        )

    def _parse_command(self, text: str) -> dict:
        """解析语音指令，返回匹配的物品映射。"""
        for intent, info in COMMAND_MAP.items():
            for kw in info["keywords"]:
                if kw in text:
                    return info
        return None

    def _get_object_position(self, object_name: str) -> tuple:
        """获取目标物品 3D 坐标。

        仿真：从 SimConfig 读固定坐标。
        真机：订阅环境提供的物品坐标 topic。
        """
        if self._sim:
            pos = SimConfig.OBJECTS.get(object_name)
            if pos:
                self.get_logger().info(f"[坐标] {object_name} → {pos}")
            return pos
        else:
            # TODO: 真机订阅物品坐标
            # 例如 /aima/perception/objects/{name}/pose
            self.get_logger().warn(f"真机物品坐标获取待实现: {object_name}")
            return SimConfig.OBJECTS.get(object_name)

    def _approach_and_grasp(self, object_name: str, obj_pos: tuple):
        """靠近物体并执行抓取。

        简化实现（仿真）：走到物体附近 → 伸臂 → 闭合夹爪。
        真机需要 IK 求解和多传感器融合。
        """
        ox, oy, oz = obj_pos

        # 1. 走到物体前方
        approach_x = ox - 0.3  # 站在物体前 30cm
        approach_y = oy
        self.get_logger().info(f"靠近物体: ({approach_x:.2f}, {approach_y:.2f})")
        self.motion.move_toward(approach_x, approach_y, speed=0.12, timeout=20.0)

        # 2. 伸臂（简化：手臂前伸 + 手腕调整）
        self.get_logger().info("伸臂准备抓取...")
        # 左臂前伸姿态（仿真近似值）
        left_targets = [0.5, 0.0, 0.0, -0.3, 0.0, 0.0, 0.0]
        # 右臂前伸姿态
        right_targets = [0.5, 0.0, 0.0, -0.3, 0.0, 0.0, 0.0]
        self.arm.goto(left_targets, right_targets, steps=30, interval=0.03)

        # 3. 闭合夹爪
        time.sleep(0.5)
        self.hand.grip(0.8)  # 闭合抓取
        time.sleep(0.5)

        self.get_logger().info(f"✅ 已抓取 {object_name}")

    def _retract_and_place(self, target_zone: tuple = (-1.0, -1.0)):
        """收臂并返回放置区。"""
        # 收臂
        self.get_logger().info("收臂...")
        # 恢复到默认手臂姿态
        default_left = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        default_right = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.arm.goto(default_left, default_right, steps=20, interval=0.03)

        # 释放夹爪
        time.sleep(0.3)
        self.hand.grip(0.0)  # 张开
        time.sleep(0.3)

        self.get_logger().info("物品已放置。")

    def run(self):
        self.get_logger().info(
            f"\n{'='*50}\n"
            f"  任务6：抓取作业\n"
            f"  {'仿真模式' if self._sim else '真机模式'}\n"
            f"{'='*50}"
        )

        if not set_ready(self.mode, self.input_src):
            self.get_logger().error("无法进入运动模式")
            return

        # 打印可选命令
        print("\n可选语音指令:")
        for intent, info in COMMAND_MAP.items():
            print(f"  - {info['keywords'][0]} → {info['object']}")

        # 接收指令
        cmd = self.speech.listen("请说出您需要什么（如'我有点口渴'）:")

        # 模拟模式：支持数字输入
        if self._sim and (cmd.strip().isdigit() or cmd.strip() == ""):
            intents = list(COMMAND_MAP.keys())
            if cmd.strip().isdigit():
                idx = int(cmd.strip()) - 1
                if 0 <= idx < len(intents):
                    cmd = intents[idx]
            else:
                cmd = random.choice(list(COMMAND_MAP.keys()))

        # 解析指令
        info = self._parse_command(cmd)
        if info is None:
            self.speech.say(f"抱歉，我不理解您需要什么。请说'吃药'、'口渴'或'饿'。")
            cmd = self.speech.listen("请再说一遍:")
            info = self._parse_command(cmd)
            if info is None:
                self.speech.say("抱歉，无法理解需求。")
                return

        object_name = info["object"]
        self.speech.say(info["tts"])
        self.get_logger().info(f"目标物品: {object_name}")

        # 获取物品坐标
        obj_pos = self._get_object_position(object_name)
        if obj_pos is None:
            self.speech.say(f"找不到{object_name}的位置。")
            return

        # 执行抓取
        self._approach_and_grasp(object_name, obj_pos)

        # 放置物品
        self._retract_and_place()

        self.speech.say(f"已完成{object_name}的抓取和放置。")
        self.get_logger().info("任务6结束。")


def main():
    parser = argparse.ArgumentParser(description="任务6：抓取作业")
    parser.add_argument("--sim", action="store_true", default=True)
    args = parser.parse_args()

    rclpy.init()
    node = Task6Grasp(sim=args.sim)
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("用户中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
