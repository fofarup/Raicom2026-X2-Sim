"""离线语音识别模块 — sherpa-onnx SenseVoice。

首次使用需下载模型（~200MB），放在 models/asr/ 下。
如果模型不存在，自动回退到键盘输入。

安装: pip3 install sherpa-onnx
"""

import os
import sys
import time
import wave
import subprocess
import tempfile
from pathlib import Path

MODEL_DIR = Path(__file__).resolve().parents[1] / "models" / "asr"
# SenseVoice 模型路径
MODEL_PATH = MODEL_DIR / "sherpa-onnx-sense-voice-zh-en-ja-ko-yue-2024-07-17"
MODEL_FILES = [
    "model.onnx",          # ONNX 模型
    "tokens.txt",          # 词表
]


def _find_model() -> str | None:
    """找模型目录。"""
    if MODEL_PATH.is_dir():
        return str(MODEL_PATH)
    # 也搜其他可能的模型
    if MODEL_DIR.is_dir():
        for d in MODEL_DIR.iterdir():
            if d.is_dir() and (d / "model.onnx").exists():
                return str(d)
    return None


def download_model():
    """下载 SenseVoice 模型（约 200MB）。"""
    import urllib.request
    import tarfile

    os.makedirs(MODEL_DIR, exist_ok=True)
    url = ("https://github.com/k2-fsa/sherpa-onnx/releases/download/"
           "asr-models/sherpa-onnx-sense-voice-zh-en-ja-ko-yue-2024-07-17.tar.bz2")
    fname = MODEL_DIR / "sensevoice.tar.bz2"
    print(f"下载 SenseVoice 模型 ({url})...")
    urllib.request.urlretrieve(url, str(fname))
    print("解压中...")
    with tarfile.open(str(fname), "r:bz2") as tar:
        tar.extractall(str(MODEL_DIR))
    os.remove(str(fname))
    print(f"模型已就绪: {MODEL_PATH}")


class OfflineASR:
    """离线语音识别器。"""

    def __init__(self):
        self._recognizer = None
        model_path = _find_model()
        if model_path is None:
            print("[ASR] 模型未找到，使用键盘输入回退。")
            print("[ASR] 运行 download_model.sh 下载模型。")
            return
        try:
            import sherpa_onnx
            sv = sherpa_onnx.OfflineSenseVoiceModelConfig()
            sv.model = str(Path(model_path) / "model.onnx")
            sv.language = "zh"
            mc = sherpa_onnx.OfflineModelConfig(
                sense_voice=sv,
                tokens=str(Path(model_path) / "tokens.txt"),
            )
            cfg = sherpa_onnx.OfflineRecognizerConfig(model_config=mc)
            self._recognizer = sherpa_onnx.offline_recognizer._Recognizer(cfg)
            print("[ASR] SenseVoice 模型已加载")
        except Exception as e:
            print(f"[ASR] 模型加载失败: {e}，回退键盘")

    def recognize_file(self, wav_path: str) -> str:
        """识别 WAV 文件（16kHz, mono, 16-bit）。"""
        if self._recognizer is None:
            return ""
        import wave
        import numpy as np
        with wave.open(wav_path, "rb") as wf:
            frames = wf.readframes(wf.getnframes())
            audio = np.frombuffer(frames, dtype=np.int16).astype(np.float32) / 32768.0
            sr = wf.getframerate()
        stream = self._recognizer.create_stream()
        stream.accept_waveform(sr, audio)
        self._recognizer.decode_stream(stream)
        return stream.result.text.strip()

    def listen(self, prompt: str = "", duration: float = 5.0) -> str:
        """从麦克风录音 duration 秒，识别后返回文本。

        如果 ASR 不可用，回退键盘输入。
        """
        if self._recognizer is None:
            if prompt:
                print(f"\n🤖 {prompt}")
            return input("🎤 请输入: ").strip()

        print(f"\n🤖 {prompt}")
        print(f"🎤 录音 {duration} 秒...")

        # 用 arecord 录音
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
            wav_path = tmp.name

        try:
            subprocess.run([
                "arecord", "-D", "pulse", "-f", "S16_LE",
                "-r", "16000", "-c", "1",
                "-d", str(int(duration)),
                wav_path,
            ], capture_output=True, check=True, timeout=int(duration) + 5)

            result = self.recognize_file(wav_path)
            if result:
                print(f"🎤 识别: {result}")
                return result
        except Exception as e:
            print(f"[ASR] 录音失败: {e}")
        finally:
            os.unlink(wav_path)

        # 回退
        return input("🎤 识别失败，请输入: ").strip()


# 全局单例
_asr_instance = None


def get_asr() -> OfflineASR:
    global _asr_instance
    if _asr_instance is None:
        _asr_instance = OfflineASR()
    return _asr_instance
