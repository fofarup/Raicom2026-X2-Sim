#!/usr/bin/env python3
"""Play validated PCM through the X2 speaker without using PlayTts."""

from __future__ import annotations

import argparse
import time
import uuid
from pathlib import Path

import numpy as np

from audio_pcm import SAMPLE_RATE, pcm_chunks, read_pcm_wav, validate_pcm_s16le


class RobotPcmPlayer:
    def __init__(self, node, pkg_name: str = "raicom_project") -> None:
        from aimdk_msgs.msg import AudioPlayback, FocusResponse
        from aimdk_msgs.srv import AbandonAudioFocus, RequestAudioFocus
        from robot_profile import load_robot_profile

        self.node = node
        self.pkg_name = pkg_name
        profile = load_robot_profile()
        self._gain = float(profile["audio"].get("playback_gain", 1.0))
        playback_topic = profile["topics"]["audio_playback"]
        focus_topic = profile["topics"]["audio_focus_response"]
        request_service = profile["services"]["audio_focus_request"]
        release_service = profile["services"]["audio_focus_release"]
        self._focus = False
        self._AudioPlayback = AudioPlayback
        self._RequestAudioFocus = RequestAudioFocus
        self._AbandonAudioFocus = AbandonAudioFocus
        self._publisher = node.create_publisher(AudioPlayback, playback_topic, 10)
        node.create_subscription(FocusResponse, focus_topic, self._on_focus, 10)
        self._request = node.create_client(RequestAudioFocus, request_service)
        self._release = node.create_client(AbandonAudioFocus, release_service)
        node.get_logger().info(
            f"真机PCM播放={playback_topic} 音频焦点={focus_topic} gain={self._gain:.2f}"
        )

    def _on_focus(self, msg) -> None:
        if msg.pkg_name == self.pkg_name:
            self._focus = bool(msg.focus_gain)

    def _call_focus(self, acquire: bool) -> bool:
        import rclpy
        from aimdk_msgs.msg import FocusRequester

        client = self._request if acquire else self._release
        request_type = self._RequestAudioFocus if acquire else self._AbandonAudioFocus
        if not client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().error(f"音频焦点服务不可用: {client.srv_name}")
            return False
        request = request_type.Request()
        request.focus_requester = FocusRequester(
            pkg_name=self.pkg_name, priority=6, priority_weight=50
        )
        for attempt in range(8):
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=0.25)
            if future.done() and future.result() is not None:
                response = future.result()
                success = response.reponse.status.value == 1
                self._focus = bool(response.focus_response.focus_gain)
                if success and (self._focus if acquire else not self._focus):
                    return True
            self.node.get_logger().warning(f"音频焦点第 {attempt + 1} 次请求未成功")
        return False

    def play_pcm(self, data: bytes) -> bool:
        import rclpy
        from aimdk_msgs.msg import AudioData, AudioInfo

        validate_pcm_s16le(data)
        if not data:
            return False
        if self._gain != 1.0:
            samples = np.frombuffer(data, dtype="<i2").astype(np.float32)
            samples = np.clip(samples * self._gain, -32768, 32767).astype("<i2")
            data = samples.tobytes()
        if not self._call_focus(True):
            self.node.get_logger().error("未取得音频焦点，拒绝发送PCM")
            return False
        token = f"raicom_{uuid.uuid4().hex}"
        try:
            for chunk in pcm_chunks(data, milliseconds=40):
                rclpy.spin_once(self.node, timeout_sec=0.0)
                if not self._focus:
                    self.node.get_logger().error("播放期间丢失音频焦点")
                    return False
                message = self._AudioPlayback()
                message.stamps = self.node.get_clock().now().to_msg()
                message.pkg_name = self.pkg_name
                message.token_id = token
                message.info = AudioInfo(
                    channels=1,
                    sample_rate=SAMPLE_RATE,
                    size=len(chunk),
                    sample_format="S16LE",
                    coding_format="pcm",
                )
                message.data = AudioData(data=list(chunk))
                self._publisher.publish(message)
                time.sleep(len(chunk) / (SAMPLE_RATE * 2.0))
            time.sleep(0.12)
            return True
        finally:
            if not self._call_focus(False):
                self.node.get_logger().warning("音频焦点释放未确认")


def main() -> None:
    parser = argparse.ArgumentParser(description="X2原始PCM扬声器验证（不调用PlayTts）")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--wav", type=Path)
    source.add_argument("--pcm", type=Path)
    parser.add_argument("--dry-run", action="store_true", help="只验证音频，不启动ROS")
    args = parser.parse_args()
    data = read_pcm_wav(args.wav) if args.wav else args.pcm.read_bytes()
    validate_pcm_s16le(data)
    print(f"PCM PASS bytes={len(data)} duration={len(data) / 32000.0:.3f}s")
    if args.dry_run:
        return

    import rclpy
    from rclpy.node import Node

    rclpy.init()
    node = Node("raicom_robot_audio_playback")
    try:
        success = RobotPcmPlayer(node).play_pcm(data)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
