"""语音识别 — 云端讯飞流式优先，本地 SenseVoice 保底。

链路（比赛规则豁免语音交互接云端）：
  麦克风 → VAD 短静默切段(0.5s) → 讯飞流式识别(~0.5s) → 结果
  讯飞失败/断网 → 回退本地 SenseVoice(CPU 2-3s)
VAD (Voice Activity Detection): 自动检测说话开始/结束，无需固定时长。
如果 VAD 模型不可用，回退固定 4 秒录音。
如果 ASR 模型不可用，回退键盘输入。
"""

import os
import sys
import time
import wave
import tempfile
import numpy as np
from pathlib import Path

from .cloud_asr import CloudASR

MODEL_DIR = Path(__file__).resolve().parents[1] / "models" / "asr"


def _find_model() -> str | None:
    """搜模型目录（含 model.onnx + tokens.txt）。"""
    if not MODEL_DIR.is_dir():
        return None
    for d in MODEL_DIR.iterdir():
        if d.is_dir() and (d / "model.onnx").exists() and (d / "tokens.txt").exists():
            return str(d)
    return None


def _find_vad_model() -> str | None:
    """搜 VAD 模型。"""
    p = MODEL_DIR / "silero_vad.onnx"
    return str(p) if p.exists() else None


class OfflineASR:
    """离线语音识别器，支持 VAD。"""

    def __init__(self):
        self._recognizer = None
        self._vad = None
        self._cloud = CloudASR()
        if self._cloud.available():
            print("[ASR] 讯飞云端识别已加载")
        model_path = _find_model()
        if model_path is None:
            print("[ASR] 本地模型未找到，仅云端模式。")
        else:
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
                print("[ASR] SenseVoice 已加载")

                # VAD（0.5s 静默即切段，压响应延迟）
                vad_path = _find_vad_model()
                if vad_path:
                    import sherpa_onnx as so
                    svc = so.SileroVadModelConfig(model=vad_path, threshold=0.5,
                        min_silence_duration=0.5, min_speech_duration=0.3)
                    vc = so.VadModelConfig(silero_vad=svc, sample_rate=16000)
                    self._vad = so.VoiceActivityDetector(vc, buffer_size_in_seconds=30)
                    print("[ASR] Silero VAD 已加载")
            except Exception as e:
                print(f"[ASR] 模型加载失败: {e}，仅云端模式")

    def recognize_file(self, wav_path: str) -> str:
        """识别 WAV 文件（16kHz, mono, 16-bit）。"""
        if self._recognizer is None:
            return ""
        with wave.open(wav_path, "rb") as wf:
            frames = wf.readframes(wf.getnframes())
            audio = np.frombuffer(frames, dtype=np.int16).astype(np.float32) / 32768.0
            sr = wf.getframerate()
        stream = self._recognizer.create_stream()
        stream.accept_waveform(sr, audio)
        self._recognizer.decode_stream(stream)
        return stream.result.text.strip()

    def _record_vad(self, prompt: str = "") -> str | None:
        """先录长段音频，再用 VAD 切割出说话段。"""
        if self._vad is None:
            return None

        if prompt:
            print(f"\n🤖 {prompt}")

        # 录音（parec → raw PCM）
        max_dur = 4
        raw_full = tempfile.mktemp(suffix=".raw")
        print(f"🎤 录音 {max_dur}s（说话自动检测）...")
        os.system(
            f"timeout {max_dur} parec --format=s16le --rate=16000 --channels=1"
            f" > {raw_full} 2>/dev/null"
        )

        if os.path.getsize(raw_full) < 2000:
            os.unlink(raw_full); return None

        # 读 raw PCM
        audio = np.fromfile(raw_full, dtype=np.int16).astype(np.float32) / 32768.0
        os.unlink(raw_full)
        sr = 16000
        if len(audio) < 1600:
            return None

        # 用 VAD 找语音段
        import sherpa_onnx as so
        vad_path = _find_vad_model()
        svc = so.SileroVadModelConfig(model=vad_path, threshold=0.5,
            min_silence_duration=0.5, min_speech_duration=0.3)
        vc = so.VadModelConfig(silero_vad=svc, sample_rate=16000)
        vad = so.VoiceActivityDetector(vc, buffer_size_in_seconds=30)

        segments = []
        in_speech, seg_start = False, 0
        chunk = 512
        for i in range(0, len(audio) - chunk, chunk):
            vad.accept_waveform(audio[i:i+chunk])
            if vad.is_speech_detected():
                if not in_speech:
                    seg_start, in_speech = i, True
            elif in_speech:
                segments.append((seg_start, i))
                in_speech = False
        vad.flush()
        if in_speech:
            segments.append((seg_start, len(audio)))

        if not segments:
            return None

        best = max(segments, key=lambda s: s[1] - s[0])
        s, e = best
        dur = (e - s) / sr
        print(f"  🔊 {dur:.1f}s 语音段")

        wav_out = tempfile.mktemp(suffix=".wav")
        seg = audio[s:e]
        with wave.open(wav_out, "wb") as wf:
            wf.setnchannels(1); wf.setsampwidth(2); wf.setframerate(sr)
            wf.writeframes((seg * 32768).astype(np.int16).tobytes())
        return wav_out

    def _record_fixed(self, duration: float = 5.0) -> str | None:
        """固定时长录音（parec 输出 raw PCM → 转 WAV）。"""
        raw = tempfile.mktemp(suffix=".raw")
        d = int(duration)
        print(f"🎤 录音 {d} 秒...")
        os.system(
            f"timeout {d} parec --format=s16le --rate=16000 --channels=1"
            f" > {raw} 2>/dev/null"
        )
        if os.path.getsize(raw) < 2000:
            os.unlink(raw); return None
        # 转 WAV
        wav = tempfile.mktemp(suffix=".wav")
        data = np.fromfile(raw, dtype=np.int16)
        os.unlink(raw)
        with wave.open(wav, "wb") as wf:
            wf.setnchannels(1); wf.setsampwidth(2); wf.setframerate(16000)
            wf.writeframes(data.tobytes())
        return wav

    def listen(self, prompt: str = "", duration: float = 5.0) -> str:
        """语音输入。讯飞云端 > 本地 SenseVoice > 键盘。"""
        cloud_ok = self._cloud is not None and self._cloud.available()
        if self._recognizer is None and not cloud_ok:
            if prompt:
                print(f"\n🤖 {prompt}")
            return input("🎤 请输入: ").strip()

        wav_path = None
        # 优先 VAD
        if self._vad is not None:
            wav_path = self._record_vad(prompt)
        # 回退固定时长（VAD 失败后等 PulseAudio 释放设备）
        if wav_path is None and self._recognizer is not None:
            time.sleep(0.3)
            wav_path = self._record_fixed(duration)

        if wav_path is not None:
            try:
                # 1) 讯飞云端流式识别（快+准，~0.5s）
                if cloud_ok:
                    with wave.open(wav_path, "rb") as wf:
                        frames = wf.readframes(wf.getnframes())
                    text = self._cloud.recognize(frames)
                    if text:
                        print(f"🎤 识别: {text}")
                        return text
                    print("[ASR] 云端失败，回退本地...")
                # 2) 本地 SenseVoice 保底
                result = self.recognize_file(wav_path)
                if result:
                    print(f"🎤 识别: {result}")
                    return result
            finally:
                os.unlink(wav_path)

        try:
            return input("🎤 识别失败，请输入: ").strip()
        except (EOFError, OSError):
            return ""


_asr_instance = None


def get_asr() -> OfflineASR:
    global _asr_instance
    if _asr_instance is None:
        _asr_instance = OfflineASR()
    return _asr_instance
