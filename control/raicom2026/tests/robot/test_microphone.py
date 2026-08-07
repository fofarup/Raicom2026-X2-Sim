#!/usr/bin/env python3
"""真机麦克风测试 —— 录音 + 回放，验证麦克风硬件。

用法（在 PC2 真机上）：
  python3 test_microphone.py              # 录5秒并回放
  python3 test_microphone.py --duration 3  # 录3秒
  python3 test_microphone.py --list        # 列出音频设备

依赖：arecord + paplay（系统自带）、numpy（可选，显示波形）

在 X2 真机 PC2 (Jetson Orin NX) 上，麦克风设备可能与笔记本不同。
先运行 --list 查看可用设备。
"""

import os
import sys
import subprocess
import tempfile

# 回退设备列表（按优先级尝试）
RECORD_DEVICES = [
    "plughw:CARD=PCH,DEV=0",     # Intel HDA
    "plughw:0,0",                # 通用声卡0
    "default",                    # ALSA 默认
    "plughw:1,0",                # USB 麦克风
    "plughw:2,0",                # 第二USB设备
]


def find_mic() -> str | None:
    """找到第一个可用的录音设备。"""
    # 先列设备
    r = subprocess.run(["arecord", "-l"], capture_output=True, text=True)
    print(r.stdout)

    for dev in RECORD_DEVICES:
        print(f"  尝试 {dev} ...", end=" ")
        rc = subprocess.run(
            ["timeout", "1", "arecord", "-D", dev,
             "-f", "S16_LE", "-r", "16000", "-c", "1",
             "-t", "wav", "/dev/null"],
            stderr=subprocess.DEVNULL).returncode
        if rc in (0, 124):  # 0=正常退出, 124=timeout(正常)
            print("OK")
            return dev
        print("失败")
    return None


def record(device: str, duration: int, output: str) -> bool:
    """录音到 WAV 文件。"""
    print(f"🎤 录音 {duration} 秒（请说话）...")
    rc = subprocess.run(
        ["timeout", str(duration), "arecord", "-D", device,
         "-f", "S16_LE", "-r", "16000", "-c", "1",
         "-t", "wav", output],
        stderr=subprocess.DEVNULL).returncode
    size = os.path.getsize(output)
    if size < 2000:
        print(f"  ❌ 文件过小({size}B)")
        return False
    print(f"  ✅ {size} 字节 ({size/32000:.1f}s)")
    return True


def play(path: str):
    """回放 WAV。"""
    print("🔊 回放...")
    for cmd in (["paplay", path], ["aplay", path]):
        if subprocess.run(cmd, stderr=subprocess.DEVNULL).returncode == 0:
            print("  ✅ 播放完毕")
            return
    print("  ❌ 播放失败")


def show_waveform(path: str):
    """显示波形统计（需要 numpy）。"""
    try:
        import numpy as np
        import wave
        with wave.open(path, "rb") as wf:
            d = np.frombuffer(wf.readframes(wf.getnframes()), dtype=np.int16)
        rms = np.sqrt(np.mean(d.astype(float) ** 2))
        print(f"  波形: peak={abs(d).max()} rms={rms:.0f} "
              f"({'✅正常' if rms > 100 else '⚠️信号弱,检查麦克风'})")
    except ImportError:
        pass


def main():
    if "--list" in sys.argv:
        subprocess.run(["arecord", "-l"])
        print("\nPulseAudio 源:")
        subprocess.run(["pactl", "list", "sources", "short"])
        return

    duration = 5
    if "--duration" in sys.argv:
        try:
            idx = sys.argv.index("--duration")
            duration = int(sys.argv[idx + 1])
        except (ValueError, IndexError):
            pass

    print("=== 真机麦克风测试 ===")

    device = find_mic()
    if device is None:
        print("\n❌ 未找到可用录音设备！")
        print("  检查: 1) 麦克风是否连接  2) arecord -l 是否有 capture 设备")
        sys.exit(1)

    print(f"\n使用设备: {device}")

    wav = tempfile.mktemp(suffix=".wav")
    try:
        if record(device, duration, wav):
            show_waveform(wav)
            play(wav)
            print("\n✅ 麦克风测试通过")
        else:
            print("\n❌ 录音失败")
    finally:
        if os.path.exists(wav):
            os.unlink(wav)


if __name__ == "__main__":
    main()
