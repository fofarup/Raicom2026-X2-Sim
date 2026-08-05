#!/usr/bin/env python3
"""睿抗 2026 国赛统一入口：一次准备、一次开始、三任务连续自主执行。"""
from __future__ import annotations

import argparse
import datetime
import math
import os
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from core.expression import ExpressionController
from core.gesture import GestureController
from core.grasp import GraspController, world_to_base
from core.locomotion import InputSource, MotionController
from core.mode_switch import ModeSwitch
from core.navigator import INTERACT_I, INTERACT_II, WORK_ZONE, Navigator
from core.scenario import (EXPRESSIONS, GESTURES, CompetitionState, NEEDS,
                           parse_need, validate_draw)
from core.speech import SpeechController
from core.vision import RESOURCES_DIR, VisionController


class CompetitionNode(Node):
    def __init__(self, args):
        super().__init__("raicom2026_competition")
        self.args, self.state = args, CompetitionState.PREPARE
        self.motion = MotionController(self, "raicom2026_nav")
        # A small non-zero command keeps the MC's auto-transitioning locomotion
        # action alive until it reaches RUNNING; it is replaced by zero after.
        self.mode = ModeSwitch(self, lambda: self.motion.publish(0.02, 0.0))
        self.input_source = InputSource(self, "raicom2026_nav", priority=50)
        self.navigator = Navigator(self, self.motion, sim=args.sim)
        self.speech = SpeechController(self, sim=args.sim)
        self.expression = ExpressionController(self, sim=args.sim)
        self.grasp = GraspController(self, sim=args.sim)
        self.gesture = GestureController(self.grasp)
        self.vision = VisionController(self, sim=args.sim, image_path=args.number_image)
        self.sim_option = self.create_publisher(String, "/aima/sim/option/command", 10)

    def transition(self, state: CompetitionState):
        self.state = state
        self.get_logger().info(f"[STATE] {state.value}")

    def prepare(self) -> bool:
        """准备发生在裁判开始口令之前，不计入自主任务过程。"""
        if self.args.sim and self.args.auto_prepare:
            # 先锁关节防止 Reset 后立刻摔倒，再请求站立。
            self.mode.set("JD")
            if not self.mode.request("SD"):
                return False
            helper = "/tmp/raicom_x11_window_tool"
            try:
                result = subprocess.run(
                    [helper, "MuJoCo", "reset"], check=False,
                    capture_output=True, text=True, timeout=3.0,
                    env={**os.environ, "DISPLAY": os.environ.get("DISPLAY", ":1")},
                )
            except (OSError, subprocess.SubprocessError) as exc:
                self.get_logger().error(f"MuJoCo GUI Reset 调用失败: {exc}")
                return False
            if result.returncode != 0:
                self.get_logger().error(
                    f"MuJoCo GUI Reset 失败: {result.stderr.strip()}")
                return False
            self.navigator.reset_map()
            for _ in range(20):
                rclpy.spin_once(self, timeout_sec=0.1)
            # The GUI command is processed on MuJoCo's render thread; discard
            # any queued pre-reset cloud frames that arrived during that handoff.
            self.navigator.reset_map()
            if not self.mode.wait("SD", timeout=15.0):
                self.get_logger().error("Reset 后 SD 未进入 RUNNING")
                return False
            self.get_logger().info("MuJoCo 已在赛前执行真实 GUI Reset")
        else:
            if not self.mode.set("SD"):
                return False
        if not self.args.auto_prepare:
            self.speech.listen("请在 MuJoCo 点击 Reset；机器人站稳后按回车")
        # RUNNING describes the action state, not that the stand-up motion has
        # already reached full height.  Require one continuous second upright
        # before enabling locomotion, especially after a cold MC start.
        stable_since = None
        stable_deadline = time.monotonic() + 20.0
        while time.monotonic() < stable_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            height = self.motion.position[2] if self.motion.position else 0.0
            if height >= 0.55:
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= 1.0:
                    break
            else:
                stable_since = None
        if stable_since is None or time.monotonic() - stable_since < 1.0:
            self.get_logger().error("机器人未站稳，拒绝进入行走模式")
            return False
        if self.args.sim:
            x, y, _ = self.motion.position
            # 如果机器人距起源点过远，说明上次运行后仿真没有重置位置。
            # 这里仅警告，不再拒绝；任务是导航到目标区，起始位置偏差
            # 可以被自主导航吸收。真机比赛每次有工作人员恢复出发区。
            if math.hypot(x + 1.5, y + 1.5) > 0.80:
                self.get_logger().warn(
                    f"Reset 后位置 ({x:.2f}, {y:.2f}) 距出发区较远，"
                    "建议重启仿真进程以获得干净的起始状态。")
        # LD needs an active velocity source during its transition on the X2 MC.
        if not self.input_source.register() or not self.mode.set("LD"):
            return False
        time.sleep(2.0)
        self.transition(CompetitionState.WAIT_START)
        if not self.args.auto_start:
            self.speech.listen("裁判下达开始指令后按回车")
        return True

    def task1(self) -> bool:
        self.transition(CompetitionState.NAVIGATE_INTERACTION_I)
        self.speech.say("正在自主导航至交互区I。")
        if not self.navigator.goto(*INTERACT_I, speed=0.50, timeout=240.0):
            return False
        self.transition(CompetitionState.FACE_INTERACTION_II)
        if not self.navigator.face(*INTERACT_II):
            return False
        self.speech.say("已进入交互区I并面向交互区II。")
        return True

    def _select(self, supplied: str | None, choices: tuple[str, ...], prompt: str) -> str:
        if supplied:
            return supplied
        while True:
            value = self.speech.listen(f"{prompt}（{' / '.join(choices)}）")
            match = next((choice for choice in choices if choice in value), None)
            if match:
                return match
            self.speech.say("输入未匹配，请重试。")

    def task2(self) -> bool:
        self.transition(CompetitionState.BASIC_INTERACTION)
        now = datetime.datetime.now()
        self.speech.say(f"现在是{now.hour}点{now.minute}分。")
        try:
            result = self.vision.recognize_number()
        except Exception as exc:
            self.get_logger().error(f"数字识别失败: {exc}")
            return False
        self.speech.say(f"图中的数字是{result['digit']}，颜色是{result['color']}。")
        expression = self._select(self.args.expression, EXPRESSIONS, "请输入抽中的表情")
        gesture = self._select(self.args.gesture, GESTURES, "请输入抽中的动作")
        validate_draw(expression, gesture, self.args.hand)
        self.expression.show(expression)
        self.speech.say(f"我正在执行{gesture}动作。")
        if not self.mode.set("US"):
            return False
        if not self.gesture.perform(gesture):
            return False
        if not self.gesture.return_to_ready():
            return False
        # Task 3 starts with locomotion, so hand lower-body control back to LD.
        return self.mode.set("LD")

    def task3(self) -> bool:
        self.transition(CompetitionState.UNDERSTAND_NEED)
        text = self.args.need or self.speech.listen("请模拟说出养老服务需求")
        need = parse_need(text)
        if need is None:
            self.speech.say("抱歉，我没有听清需求，请再说一次。")
            need = parse_need(self.speech.listen("请重新表达需求"))
        if need is None:
            return False
        self.speech.say(need.response)
        self.transition(CompetitionState.NAVIGATE_WORK_ZONE)
        if not self.navigator.goto(
                *WORK_ZONE, speed=0.50, timeout=240.0, tolerance=0.40):
            return False
        # 从安全工作区低速进入抓取停靠位；左/右手各自保留 25 cm 横向偏置。
        if not self.navigator.dock_for_grasp(need.object_world_xyz, self.args.hand):
            return False
        self.transition(CompetitionState.GRASP_AND_LIFT)
        if not self.mode.set("US"):
            return False
        # LD -> SD -> US changes stance and therefore the pelvis frame. Build
        # the IK target only after the new mode has physically settled.
        for _ in range(40):
            rclpy.spin_once(self, timeout_sec=0.025)
        px, py, pz = self.motion.position
        observed = self.grasp.object_position(need.object_name)
        object_world = ((observed[0], observed[1], need.object_world_xyz[2])
                        if observed else need.object_world_xyz)
        base_target = world_to_base(
            object_world, (px, py, pz, self.motion.yaw))
        self.get_logger().info(f"识别到{need.object_name}，base坐标={base_target}")
        if not self.grasp.grasp_and_lift(
                self.args.hand, base_target, object_name=need.object_name):
            return False
        self.transition(CompetitionState.ANNOUNCE_WHILE_HOLDING)
        self.speech.say(need.done)
        if not self.grasp.hold_grip(self.args.hand, duration=3.0):
            return False
        self.expression.show("快乐")
        return True

    def run(self) -> bool:
        if not self.prepare():
            self.transition(CompetitionState.FAILED)
            return False
        for task in (self.task1, self.task2, self.task3):
            if not task():
                self.transition(CompetitionState.FAILED)
                self.speech.say("任务执行失败，机器人已安全停止。")
                self.motion.stop(1.0)
                return False
        self.transition(CompetitionState.COMPLETE)
        self.speech.say("全部比赛任务完成。")
        self.motion.stop(1.0)
        return True


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--auto-prepare", action="store_true", help="仅自动测试使用")
    parser.add_argument("--auto-start", action="store_true", help="仅自动测试使用")
    parser.add_argument("--number-image", default="number_01.png")
    parser.add_argument("--expression", choices=EXPRESSIONS)
    parser.add_argument("--gesture", choices=GESTURES)
    parser.add_argument("--need", help="仿真语音需求文本")
    parser.add_argument("--hand", choices=("left", "right"), default="right")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = CompetitionNode(args)
    try:
        success = node.run()
    except KeyboardInterrupt:
        success = False
        node.motion.stop(1.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
