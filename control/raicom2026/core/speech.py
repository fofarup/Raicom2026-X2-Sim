"""语音抽象层。

仿真模式(--sim)：键盘输入 + print 输出
非仿真模式：sherpa-onnx SenseVoice 离线 ASR + 麦克风
"""

from rclpy.node import Node
from .offline_asr import get_asr


class SpeechController:
    def __init__(self, node: Node, sim: bool = True):
        self._node = node
        self._sim = sim
        if not sim:
            self._asr = get_asr()  # 延迟加载模型
        else:
            self._asr = None

    def say(self, text: str):
        """机器人语音输出。"""
        if self._sim:
            self._node.get_logger().info(f"[TTS] {text}")
        else:
            self._node.get_logger().info(f"TTS: {text}")

    def listen(self, prompt: str = "", duration: float = 5.0) -> str:
        """获取语音输入。仿真键盘，真机麦克风+ASR。"""
        if self._sim or self._asr is None:
            if prompt:
                print(f"\n🤖 {prompt}")
            return input("🎤 请输入: ").strip()
        return self._asr.listen(prompt, duration=duration)
