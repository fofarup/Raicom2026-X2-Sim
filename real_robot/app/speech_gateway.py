#!/usr/bin/env python3
"""Convert /raicom/speech/text to PCM using an external TTS provider."""

from __future__ import annotations

import argparse
import json
import os
import queue
import subprocess
import tempfile
from pathlib import Path

from audio_pcm import read_pcm_wav


class CommandWavSynthesizer:
    """Run a configured argv template that writes a 16kHz mono PCM WAV."""

    def __init__(self, command_json: str) -> None:
        try:
            template = json.loads(command_json)
        except json.JSONDecodeError as exc:
            raise ValueError(f"RAICOM_TTS_COMMAND_JSON不是合法JSON: {exc}") from exc
        if not isinstance(template, list) or not template or not all(
            isinstance(item, str) for item in template
        ):
            raise ValueError("RAICOM_TTS_COMMAND_JSON必须是非空字符串数组")
        joined = "\0".join(template)
        if "{text}" not in joined or "{output}" not in joined:
            raise ValueError("TTS命令必须同时包含{text}和{output}占位符")
        self.template = template

    def synthesize(self, text: str) -> bytes:
        text = " ".join(text.split()).strip()
        if not text:
            raise ValueError("不能合成空文本")
        with tempfile.TemporaryDirectory(prefix="raicom_tts_") as directory:
            output = Path(directory) / "speech.wav"
            command = [
                part.replace("{text}", text).replace("{output}", str(output))
                for part in self.template
            ]
            completed = subprocess.run(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=30,
                check=False,
            )
            if completed.returncode != 0:
                raise RuntimeError(
                    f"TTS命令失败 code={completed.returncode}: {completed.stdout[-500:]}"
                )
            if not output.is_file():
                raise RuntimeError("TTS命令没有生成输出WAV")
            return read_pcm_wav(output)


def main() -> None:
    parser = argparse.ArgumentParser(description="RAICOM真机TTS到PCM播放网关")
    parser.add_argument(
        "--command-json",
        default=os.environ.get("RAICOM_TTS_COMMAND_JSON", ""),
        help="TTS argv JSON，必须包含{text}和{output}",
    )
    args = parser.parse_args()
    if not args.command_json:
        raise SystemExit("FAIL 未设置RAICOM_TTS_COMMAND_JSON")
    synthesizer = CommandWavSynthesizer(args.command_json)

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    from robot_audio_playback import RobotPcmPlayer
    from robot_profile import load_robot_profile

    profile = load_robot_profile()
    speech_topic = profile["topics"]["speech_text"]

    rclpy.init()
    node = Node("raicom_speech_gateway")
    pending: queue.Queue[str] = queue.Queue(maxsize=10)

    def on_text(message: String) -> None:
        try:
            pending.put_nowait(message.data)
        except queue.Full:
            node.get_logger().error("TTS队列已满，丢弃播报")

    node.create_subscription(String, speech_topic, on_text, 10)
    player = RobotPcmPlayer(node)
    node.get_logger().info("自研TTS网关就绪：未使用PlayTts")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                text = pending.get_nowait()
            except queue.Empty:
                continue
            try:
                node.get_logger().info(f"合成播报: {text}")
                pcm = synthesizer.synthesize(text)
                if not player.play_pcm(pcm):
                    node.get_logger().error("PCM播放失败")
            except Exception as exc:
                node.get_logger().error(f"TTS播报异常: {exc}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
