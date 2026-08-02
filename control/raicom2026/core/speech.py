"""语音抽象层。

仿真模式(--sim)：键盘输入 + print 输出
真机模式：ASR + TTS API（待接入）
"""

from rclpy.node import Node


class SpeechController:
    def __init__(self, node: Node, sim: bool = True):
        self._node = node
        self._sim = sim

    def say(self, text: str):
        """机器人语音输出。"""
        if self._sim:
            self._node.get_logger().info(f"[TTS] {text}")
        else:
            # TODO: 真机 TTS API
            self._node.get_logger().info(f"TTS: {text}")

    def listen(self, prompt: str = "") -> str:
        """获取语音输入。"""
        if self._sim:
            if prompt:
                print(f"\n🤖 {prompt}")
            return input("🎤 请输入（模拟语音）: ").strip()
        else:
            # TODO: 真机 ASR API
            return input(f"{prompt}: ").strip()
