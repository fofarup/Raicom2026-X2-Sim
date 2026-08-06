#!/usr/bin/env python3
"""ASR 桥接服务——跑在宿主机，容器通过文件拿识别结果。

用法（宿主机）：
  python3 asr_bridge.py

容器里的 speech.py 自动检测 ASR_BRIDGE_DIR 环境变量，
通过共享文件与桥接通信。
"""

import os
import sys
import time
import wave
import tempfile
import subprocess
from pathlib import Path

sys.path.insert(0, os.path.dirname(__file__))
from core.offline_asr import get_asr

BRIDGE_DIR = os.environ.get("ASR_BRIDGE_DIR",
    os.path.expanduser("~/x2_ws/x2_biao/control/raicom2026/.asr_bridge"))


def main():
    os.makedirs(BRIDGE_DIR, exist_ok=True)
    asr = get_asr()
    if asr._recognizer is None:
        print("ASR 模型未加载，桥接不可用")
        return

    req_file = os.path.join(BRIDGE_DIR, "request.txt")
    resp_file = os.path.join(BRIDGE_DIR, "response.txt")
    lock_file = os.path.join(BRIDGE_DIR, "lock.txt")

    print(f"ASR 桥接就绪: {BRIDGE_DIR}")
    print("等待容器请求...")

    while True:
        # 等请求
        if not os.path.exists(req_file):
            time.sleep(0.2)
            continue

        # 加锁
        try:
            with open(lock_file, "x") as f:
                f.write("locked")
        except FileExistsError:
            time.sleep(0.1)
            continue

        try:
            # 读提示
            with open(req_file) as f:
                prompt = f.read().strip()
            os.remove(req_file)

            # 录音识别
            result = asr.listen(prompt, duration=5.0)

            # 写响应
            with open(resp_file, "w") as f:
                f.write(result)
        finally:
            if os.path.exists(lock_file):
                os.unlink(lock_file)


if __name__ == "__main__":
    main()
