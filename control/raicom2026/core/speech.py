"""语音抽象层。

- 宿主机(--sim or --asr-bridge)：麦克风 + SenseVoice 离线 ASR
- 容器(Docker)：自动检测 ASR_BRIDGE_DIR 桥接
- 回退：键盘输入
- TTS：自研实现（离线 piper 优先，云端 edge-tts 可选），比赛禁用官方 TTS
"""

import os
import time

from rclpy.node import Node
from .offline_asr import get_asr
from .tts import TTSController


class SpeechController:
    def __init__(self, node: Node, sim: bool = True):
        self._node = node
        self._sim = sim
        self._bridge = os.environ.get("ASR_BRIDGE_DIR", "")
        if not sim:
            self._asr = get_asr()
        else:
            self._asr = None
        self._tts = TTSController()

    def say(self, text: str):
        self._node.get_logger().info(f"[TTS] {text}")
        # 自研 TTS：合成并播放；无模型/无音频设备时降级为仅日志
        if not self._tts.speak(text):
            self._node.get_logger().warn(f"TTS 无声输出（无模型或无音频设备），仅记录: {text}")

    def listen(self, prompt: str = "", duration: float = 5.0) -> str:
        # 桥接模式（容器↔宿主机 ASR）
        if self._bridge and os.path.isdir(self._bridge):
            req = os.path.join(self._bridge, "request.txt")
            resp = os.path.join(self._bridge, "response.txt")
            with open(req, "w") as f:
                f.write(prompt)
            if prompt:
                print(f"\n🤖 {prompt}")
            print("🎤 请说话...")
            for _ in range(int(duration * 10 + 15)):
                if os.path.exists(resp):
                    with open(resp) as f:
                        result = f.read().strip()
                    os.remove(resp)
                    print(f"🎤 识别: {result}")
                    return result
                time.sleep(0.1)
            return input("🎤 超时，请手动输入: ").strip()

        # 仿真模式
        if self._sim or self._asr is None or self._asr._recognizer is None:
            if prompt:
                print(f"\n🤖 {prompt}")
            return input("🎤 请输入: ").strip()

        # 宿主机 ASR
        return self._asr.listen(prompt, duration=duration)
