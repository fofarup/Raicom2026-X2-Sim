#!/usr/bin/env python3
"""Standalone real-X2 VAD -> ASR -> intent -> local task controller."""

from __future__ import annotations

import os
import queue
import signal
import subprocess
import threading
from pathlib import Path

import numpy as np
import rclpy
from aimdk_msgs.msg import ProcessedAudioOutput
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from deepseek_intent import DeepSeekIntentResolver
from offline_asr import AudioData, OfflineChineseASR, TARGET_SAMPLE_RATE
from robot_audio_input import VadPcmCollector
from robot_profile import load_robot_profile
from voice_intents import VoiceIntent, parse_voice_intent


APP_DIR = Path(__file__).resolve().parent


class RealVoiceController(Node):
    def __init__(self) -> None:
        super().__init__("raicom_real_voice_controller")
        profile = load_robot_profile()
        self.collector = VadPcmCollector(max_seconds=20.0)
        self.asr = OfflineChineseASR()
        self.semantic = DeepSeekIntentResolver()
        self.static_only = os.environ.get("RAICOM_STATIC_ONLY", "0") == "1"
        self.utterances: queue.Queue[bytes | None] = queue.Queue(maxsize=3)
        self.process: subprocess.Popen | None = None
        self.process_lock = threading.Lock()
        self.speech = self.create_publisher(String, profile["topics"]["speech_text"], 10)
        self.create_subscription(
            ProcessedAudioOutput, profile["topics"]["vad_audio"],
            self._on_audio, qos_profile_sensor_data,
        )
        self.worker = threading.Thread(target=self._recognition_worker, daemon=True)
        self.worker.start()
        self.say("我已经准备就绪，请说开始执行任务")
        self.get_logger().info(
            f"真机语音总控就绪，SenseVoice加载={self.asr.load_ms:.0f}ms，"
            f"静态模式={'开启' if self.static_only else '关闭'}"
        )

    def say(self, text: str) -> None:
        self.get_logger().info(f"机器人回答: {text}")
        self.speech.publish(String(data=text))

    def _on_audio(self, msg: ProcessedAudioOutput) -> None:
        try:
            completed = self.collector.feed(
                msg.stream_id, msg.audio_vad_state.value, bytes(msg.audio_data)
            )
            if completed:
                self.utterances.put_nowait(completed)
        except (ValueError, queue.Full) as exc:
            self.get_logger().error(f"VAD语句丢弃: {exc}")

    def _recognition_worker(self) -> None:
        while rclpy.ok():
            pcm = self.utterances.get()
            if pcm is None:
                return
            samples = np.frombuffer(pcm, dtype="<i2").astype(np.float32) / 32768.0
            audio = AudioData(samples, TARGET_SAMPLE_RATE, samples.size / TARGET_SAMPLE_RATE)
            try:
                text, elapsed = self.asr.transcribe(audio)
                self.get_logger().info(f"ASR={text!r} decode={elapsed:.0f}ms")
                if text:
                    self.dispatch(text)
            except Exception as exc:
                self.get_logger().error(f"ASR处理失败: {exc}")

    def _command(self, action: str) -> list[str]:
        return [str(APP_DIR / "competition_run.sh"), "voice-action", action]

    def _start(self, label: str, command: list[str], stdin_text: str = "") -> bool:
        with self.process_lock:
            if self.process is not None and self.process.poll() is None:
                self.say("我正在执行任务，如需中断请说停止任务")
                return False
            self.process = subprocess.Popen(
                command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                stdin=subprocess.PIPE if stdin_text else None,
                text=True, start_new_session=True,
            )
            process = self.process
            if stdin_text and process.stdin is not None:
                process.stdin.write(stdin_text)
                process.stdin.close()

        def monitor() -> None:
            assert process.stdout is not None
            for line in process.stdout:
                self.get_logger().info(f"[{label}] {line.rstrip()}")
            code = process.wait()
            with self.process_lock:
                if self.process is process:
                    self.process = None
            if label.startswith("基础交互-"):
                # Task2 already speaks its precise result; do not add a second,
                # misleading generic failure sentence.
                return
            if code != 0:
                self.say(f"{label}没有正常完成，请检查日志")
            else:
                self.say(f"{label}已完成")

        threading.Thread(target=monitor, daemon=True).start()
        return True

    def stop_action(self) -> None:
        with self.process_lock:
            process = self.process
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        subprocess.run(self._command("stop"), timeout=10, check=False)
        self.say("已停止任务并进入稳定站立模式")

    def dispatch(self, text: str) -> VoiceIntent:
        intent = parse_voice_intent(text)
        if intent.kind == "unknown":
            semantic_intent = self.semantic.resolve(text).intent
            # DeepSeek may correct only the closed Task2 command set and the
            # three Task3 care needs.  Navigation/start/stop remain strictly
            # local so cloud latency or venue conversation cannot move legs.
            if semantic_intent.kind in ("task2", "need"):
                intent = semantic_intent
        self.get_logger().info(
            f"意图={intent.kind}:{intent.value} 来源={intent.source} 置信度={intent.confidence:.2f}"
        )
        with self.process_lock:
            task_running = self.process is not None and self.process.poll() is None
        if task_running and intent.kind != "stop":
            self.get_logger().info("任务执行期间忽略非停止语音，避免扬声器回音触发")
            return intent
        if self.static_only and intent.kind in ("start_flow", "navigate", "need"):
            self.say("机器人正在充电，当前只执行基础交互，不执行行走任务")
            return intent
        if intent.kind == "stop":
            self.stop_action()
        elif intent.kind == "start_flow":
            self.say("收到，开始执行任务，我将自主前往交互区")
            self._start("Task1自主导航", self._command("interaction"))
        elif intent.kind == "navigate":
            labels = {"start": "返回出发区", "interaction": "前往交互区", "work": "前往作业区"}
            self.say("收到，" + labels[intent.value])
            self._start(labels[intent.value], self._command(intent.value))
        elif intent.kind == "status":
            self._start("查询位置", self._command("status"))
        elif intent.kind == "task2":
            canonical = {"time": "现在时间", "vision": "识别图片数字和颜色"}.get(intent.value, intent.value)
            self._start(
                "基础交互-" + intent.value,
                [str(APP_DIR / "competition_run.sh"), "2"],
                canonical + "\nquit\n",
            )
        elif intent.kind == "need" and intent.need is not None:
            self.say(intent.need.answer)
            action = {"药盒": "grasp-medicine", "一次性纸杯": "grasp-cup", "小面包": "grasp-bread"}[intent.need.item]
            self._start("Task3取" + intent.need.item, self._command(action))
        else:
            # Open microphones also capture venue conversation and fragments
            # split by VAD.  Unknown text is diagnostic information, not a
            # reason to interrupt the contestant with a spoken response.
            self.get_logger().info("未知语音仅记录日志，不播报")
        return intent

    def close(self) -> None:
        self.utterances.put(None)
        with self.process_lock:
            running = self.process is not None and self.process.poll() is None
        if running:
            self.stop_action()


def main() -> None:
    rclpy.init()
    node = RealVoiceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
