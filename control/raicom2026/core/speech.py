"""语音抽象层。

- 宿主机(--sim or --asr-bridge)：麦克风 + SenseVoice 离线 ASR
- 容器(Docker)：自动检测 ASR_BRIDGE_DIR 桥接
- 回退：键盘输入
"""

import os
import time

from rclpy.node import Node
from .offline_asr import get_asr


class SpeechController:
    def __init__(self, node: Node, sim: bool = True):
        self._node = node
        self._sim = sim
        self._bridge = os.environ.get("ASR_BRIDGE_DIR", "")
        if not sim:
            self._asr = get_asr()
        else:
            self._asr = None

    def say(self, text: str):
        self._node.get_logger().info(f"[TTS] {text}")

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
