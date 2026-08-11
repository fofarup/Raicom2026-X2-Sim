#!/usr/bin/env python3
"""Receive X2 VAD PCM and save complete utterances for ASR."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from audio_pcm import SAMPLE_RATE, SAMPLE_WIDTH_BYTES, write_pcm_wav


class VadPcmCollector:
    IDLE, START, ACTIVE, END = 0, 1, 2, 3

    def __init__(self, max_seconds: float = 20.0) -> None:
        self.max_bytes = int(max_seconds * SAMPLE_RATE * SAMPLE_WIDTH_BYTES)
        self.stream_id: int | None = None
        self.buffer = bytearray()
        self.active = False

    def feed(self, stream_id: int, vad_state: int, audio_data: bytes) -> bytes | None:
        if vad_state == self.START:
            self.stream_id = int(stream_id)
            self.buffer.clear()
            self.active = True
        if not self.active or self.stream_id != int(stream_id):
            return None
        if vad_state in (self.START, self.ACTIVE, self.END):
            self.buffer.extend(audio_data)
        if len(self.buffer) > self.max_bytes:
            self.buffer.clear()
            self.active = False
            raise ValueError("VAD utterance exceeds safety duration")
        if vad_state == self.END:
            result = bytes(self.buffer)
            self.buffer.clear()
            self.active = False
            return result if result else None
        return None


def main() -> None:
    import rclpy
    from aimdk_msgs.msg import ProcessedAudioOutput
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from robot_profile import load_robot_profile

    profile = load_robot_profile()
    vad_topic = profile["topics"]["vad_audio"]

    parser = argparse.ArgumentParser(description="接收一条X2 VAD语音并保存为WAV")
    parser.add_argument("--output", type=Path, default=Path("asr_samples/robot_latest.wav"))
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()

    class Receiver(Node):
        def __init__(self) -> None:
            super().__init__("raicom_robot_audio_input")
            self.collector = VadPcmCollector()
            self.result: bytes | None = None
            self.create_subscription(
                ProcessedAudioOutput,
                vad_topic,
                self.on_audio,
                qos_profile_sensor_data,
            )
            self.get_logger().info(f"真机VAD输入话题: {vad_topic}")

        def on_audio(self, msg: ProcessedAudioOutput) -> None:
            try:
                completed = self.collector.feed(
                    msg.stream_id, msg.audio_vad_state.value, bytes(msg.audio_data)
                )
            except ValueError as exc:
                self.get_logger().error(str(exc))
                return
            if completed is not None:
                self.result = completed
                self.get_logger().info(f"收到完整语音: {len(completed)} bytes")

    rclpy.init()
    node = Receiver()
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and node.result is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.result is None:
            raise SystemExit("FAIL 未在限定时间收到完整VAD语音")
        write_pcm_wav(args.output, node.result)
        print(f"PASS {args.output} {len(node.result)} bytes")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
