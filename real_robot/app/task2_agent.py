#!/usr/bin/env python3
"""RAICOM 2026 real-X2 Task2 basic interaction agent."""

from __future__ import annotations

import queue
import json
import os
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import cv2
import numpy as np
import rclpy
from aimdk_msgs.msg import McControlArea, McPresetMotion, RequestHeader
from aimdk_msgs.srv import PlayEmoji, SetMcAction, SetMcPresetMotion
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from doubao_vision import DoubaoVisionRecognizer
from robot_profile import load_robot_profile


ROBOT_PROFILE = load_robot_profile()
CAMERA_TOPIC = ROBOT_PROFILE["hardware"]["rgb_topic"]
SPEECH_TEXT_TOPIC = ROBOT_PROFILE["topics"]["speech_text"]
SERVICE_EMOJI = ROBOT_PROFILE["services"]["play_emoji"]
SERVICE_MOTION = ROBOT_PROFILE["services"]["preset_motion"]
SERVICE_ACTION = ROBOT_PROFILE["services"]["mc_action"]

EMOJIS = {"悲伤": 110, "睡觉": 80, "愤怒": 180, "快乐": 90, "充电": 220}
MOTIONS = {
    "挥左手": (1, 1002),
    "挥右手": (2, 1002),
    "左手敬礼": (1, 1013),
    "右手敬礼": (2, 1013),
    "双手交叉": (11, 3009),
    "双手打叉": (11, 3009),
}


@dataclass(frozen=True)
class Intent:
    kind: str
    value: str = ""


def parse_command(text: str) -> Intent:
    """Deterministic parser for every command family listed in the rules."""
    compact = "".join(text.lower().split())
    if any(word in compact for word in ("几点", "时间")):
        return Intent("time")
    if any(word in compact for word in ("数字", "颜色", "图片", "图中", "看见", "看到")):
        return Intent("vision")
    for name in EMOJIS:
        if name in compact:
            return Intent("emoji", name)
    for name in MOTIONS:
        if name in compact:
            return Intent("motion", name)
    return Intent("unknown")


def time_answer(now: datetime | None = None) -> str:
    now = now or datetime.now(ZoneInfo("Asia/Shanghai"))
    hour, minute = now.hour, now.minute
    if hour < 6:
        period = "凌晨"
    elif hour < 9:
        period = "早上"
    elif hour < 12:
        period = "上午"
    elif hour < 14:
        period = "中午"
    elif hour < 18:
        period = "下午"
    else:
        period = "晚上"
    display_hour = hour % 12 or 12
    return f"现在是{period}{display_hour}点整" if minute == 0 else f"现在是{period}{display_hour}点{minute}分"


class Task2Agent(Node):
    def __init__(self) -> None:
        super().__init__("task2_agent")
        self.bridge = CvBridge()
        self.image_lock = threading.Lock()
        self.latest_image = None
        self.recognizer = DoubaoVisionRecognizer(ROBOT_PROFILE.get("vision_cloud", {}))
        # Match the vendor's own perception/agent consumers on this firmware.
        camera_qos = qos_profile_sensor_data
        if CAMERA_TOPIC.endswith("/compressed"):
            self.create_subscription(
                CompressedImage, CAMERA_TOPIC, self._on_compressed_image, camera_qos
            )
        else:
            self.create_subscription(Image, CAMERA_TOPIC, self._on_image, camera_qos)
        self.speech_text = self.create_publisher(String, SPEECH_TEXT_TOPIC, 10)
        self.emoji = self.create_client(PlayEmoji, SERVICE_EMOJI)
        self.motion = self.create_client(SetMcPresetMotion, SERVICE_MOTION)
        self.action = self.create_client(SetMcAction, SERVICE_ACTION)
        self.get_logger().info(
            f"Task2 已启动：离线规则解析，相机={CAMERA_TOPIC}，等待指令"
        )

    def _on_image(self, msg: Image) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self.image_lock:
                self.latest_image = image
        except Exception as exc:
            self.get_logger().warning(f"相机帧转换失败: {exc}")

    def _on_compressed_image(self, msg: CompressedImage) -> None:
        try:
            image = cv2.imdecode(np.frombuffer(bytes(msg.data), dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError("JPEG解码返回空图像")
            with self.image_lock:
                self.latest_image = image
        except Exception as exc:
            self.get_logger().warning(f"压缩相机帧转换失败: {exc}")

    def _image(self, timeout: float = 4.0):
        """Wait for the first camera frame after this short-lived agent starts."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.image_lock:
                if self.latest_image is not None:
                    return self.latest_image.copy()
            time.sleep(0.05)
        return None

    def _call(self, client, request, label: str, timeout: float = 3.0):
        # AimDK documents that cross-board ROS services need discovery/retry
        # protection.  Face UI runs on PC3 and often needs more than 0.5 s.
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning(f"{label} 真机服务不可用")
            return None
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            self.get_logger().error(f"{label} 调用超时")
            return None
        try:
            response = future.result()
            self.get_logger().info(f"{label} 服务已响应")
            return response
        except Exception as exc:
            self.get_logger().error(f"{label} 调用异常: {exc}")
            return None

    @staticmethod
    def _common_success(common) -> bool:
        if common is None:
            return False
        status = getattr(common, "status", None)
        return getattr(status, "value", -1) == 1 and getattr(common.header, "code", -1) == 0

    def speak(self, text: str) -> bool:
        print(f"[播报] {text}", flush=True)
        self.speech_text.publish(String(data=text))
        return True

    def play_emoji(self, name: str) -> bool:
        helper = Path(__file__).resolve().with_name("emoji_player.py")
        result = subprocess.run(
            [sys.executable, str(helper), str(EMOJIS[name])],
            capture_output=True, text=True, timeout=8.0, check=False,
        )
        output = (result.stdout + result.stderr).strip()
        self.get_logger().info(f"表情 {name} 官方同步调用: {output}")
        return result.returncode == 0

    def ensure_sd(self) -> bool:
        # SD and the preset motion are issued together by motion_player.py.
        # It follows AimDK's required cross-board timestamp/retry pattern.
        return True

    def run_motion(self, name: str) -> bool:
        area, motion = MOTIONS[name]
        helper = Path(__file__).resolve().with_name("motion_player.py")
        result = subprocess.run(
            [sys.executable, str(helper), str(area), str(motion)],
            capture_output=True, text=True, timeout=12.0, check=False,
        )
        output = (result.stdout + result.stderr).strip()
        self.get_logger().info(f"动作 {name} 官方同步调用: {output}")
        return result.returncode == 0

    def recognize_depth_camera(self):
        """Run recognition beside the RGB-D publisher on X2 PC42."""
        remote_dir = "/home/agi/x2_deploy_workspace/raicom_real_robot/app"
        ssh_target = os.environ.get("RAICOM_PC42_SSH_TARGET", "agi@10.0.1.42")
        command = (
            f"source /opt/ros/humble/setup.bash >/dev/null 2>&1; "
            f"cd {remote_dir} && python3 depth_vision_once.py"
        )
        try:
            result = subprocess.run(
                ["ssh", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes",
                 "-o", "ConnectTimeout=3", ssh_target, command],
                capture_output=True, text=True, timeout=12.0, check=False,
            )
            for line in reversed(result.stdout.splitlines()):
                try:
                    payload = json.loads(line)
                    return payload
                except json.JSONDecodeError:
                    continue
            self.get_logger().error(f"深度相机识别无结果: {result.stderr.strip()}")
        except Exception as exc:
            self.get_logger().error(f"深度相机识别调用失败: {exc}")
        return {"ok": False, "error": "depth_recognizer_failed"}

    def handle_command(self, text: str) -> bool:
        intent = parse_command(text)
        self.get_logger().info(f"指令={text!r}, 意图={intent.kind}:{intent.value}")
        if intent.kind == "time":
            self.speak(time_answer())
        elif intent.kind == "vision":
            image = self._image()
            if image is None:
                self.speak("没有收到深度相机彩色画面，请再展示一次")
                return False
            # RK4's head RGB-D stream is physically mounted upside down.  The
            # scoring card must be made human-upright before cloud inference;
            # otherwise the ambiguous pair 6/9 is systematically reversed.
            image = cv2.rotate(image, cv2.ROTATE_180)
            result = self.recognizer.recognize(image)
            if not result.ok:
                self.get_logger().warning(f"豆包视觉识别失败: {result.error}")
                self.speak("我没有看清，请再展示一次")
                return False
            self.get_logger().info(
                f"豆包视觉结果: {result.color}{result.digit}, 置信度={result.confidence:.2f}"
            )
            self.speak(f"图中的数字是{result.digit}，颜色是{result.color}")
        elif intent.kind == "emoji":
            ok = self.play_emoji(intent.value)
            self.get_logger().info(f"表情 {intent.value}: {'成功' if ok else '失败/服务缺失'}")
            self.speak(f"已切换到{intent.value}表情" if ok else "表情切换失败")
            return ok
        elif intent.kind == "motion":
            if not self.ensure_sd():
                self.get_logger().warning("未确认 SD，取消预设动作以保护机器人")
                self.speak("机器人还没有站稳，暂不执行动作")
                return False
            self.speak(f"我正在执行{intent.value}动作")
            ok = self.run_motion(intent.value)
            self.get_logger().info(f"动作 {intent.value}: {'成功' if ok else '失败'}")
            if not ok:
                self.speak(f"{intent.value}动作没有完成，请检查机器人状态")
            return ok
        else:
            self.speak("请再说一次")
            return False
        return True


def _stdin_reader(commands: queue.Queue) -> None:
    for line in sys.stdin:
        commands.put(line.strip())
    commands.put(None)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Task2Agent()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    commands = queue.Queue()
    threading.Thread(target=_stdin_reader, args=(commands,), daemon=True).start()
    print("可输入：时间、彩色数字、5 种表情、5 种动作；输入 quit 退出。", flush=True)
    all_ok = True
    try:
        while rclpy.ok():
            command = commands.get()
            if command is None or command.lower() in ("q", "quit", "exit"):
                break
            if command:
                all_ok = node.handle_command(command) and all_ok
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()
    if not all_ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
