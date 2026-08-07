#!/usr/bin/env python3
"""真机语音交互测试 —— ASR 识别 + TTS 合成播放。

用法（在 PC2 真机上）：
  python3 test_voice.py                    # 交互模式
  python3 test_voice.py --asr              # 只测语音识别
  python3 test_voice.py --tts "你好"       # 只测语音合成播放
  python3 test_voice.py --loop             # 循环对话测试

依赖：
  pip3 install sherpa-onnx piper-tts
  bash download_model.sh   # 下载 ASR + TTS 模型到 ../models/

比赛规则三：
  ASR 豁免云端限制，可接云端（讯飞已集成）
  TTS 必须自研（piper 离线 + edge-tts 云端，禁用官方 TTS）
"""

import sys
import os
import time

# 确保能找到 core 模块
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))
sys.path.insert(0, PROJECT_DIR)

from core.offline_asr import OfflineASR
from core.tts import TTSController


def test_asr():
    """单独测试语音识别。"""
    print("=== 语音识别测试 (ASR) ===")
    print("讯飞云端优先(需网络)，本地 SenseVoice 保底\n")

    asr = OfflineASR()
    while True:
        try:
            text = asr.listen("请说话（q=退出）", duration=5.0)
            if text.lower() == "q":
                break
            if text:
                print(f"  → 最终结果: {text!r}\n")
        except (EOFError, KeyboardInterrupt):
            break


def test_tts(text: str | None = None):
    """单独测试语音合成。"""
    print("=== 语音合成测试 (TTS) ===")
    print("离线 piper 优先（默认），云端 edge-tts 可选\n")

    tts = TTSController()
    if text:
        tts.speak(text)
        print(f"✅ 合成: {text}")
        return

    samples = [
        "你好，我是X2机器人。",
        "正在自主导航至交互区一。",
        "已进入交互区一并面向交互区二。",
        "检测到红色盒子，共两个，已安全放置。",
    ]
    for s in samples:
        print(f"\n>>> {s}")
        ok = tts.speak(s)
        print(f"  {'✅ 有声' if ok else '⚠️ 无声(降级日志)'}")
        time.sleep(1.5)


def test_loop():
    """循环对话测试：ASR + TTS。"""
    print("=== 循环语音对话测试 ===")
    print("说话后机器人会复述你的话\n")

    asr = OfflineASR()
    tts = TTSController()

    while True:
        try:
            text = asr.listen("请说话（q=退出）", duration=5.0)
            if not text or text.lower() == "q":
                break
            print(f"识别: {text}")
            # 复述
            reply = f"你说了：{text}"
            tts.speak(reply)
            print()
        except (EOFError, KeyboardInterrupt):
            break


def main():
    if "--tts" in sys.argv:
        idx = sys.argv.index("--tts")
        text = sys.argv[idx + 1] if idx + 1 < len(sys.argv) else None
        test_tts(text)
    elif "--asr" in sys.argv:
        test_asr()
    elif "--loop" in sys.argv:
        test_loop()
    else:
        print("=== 真机语音交互测试 ===")
        print("1 = ASR 识别测试")
        print("2 = TTS 合成测试")
        print("3 = 循环对话测试")
        print()

        try:
            choice = input("选择 (1/2/3): ").strip()
        except (EOFError, KeyboardInterrupt):
            return

        if choice == "1":
            test_asr()
        elif choice == "2":
            test_tts()
        elif choice == "3":
            test_loop()
        else:
            print("无效选择")


if __name__ == "__main__":
    main()
