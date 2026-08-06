"""离线语音识别 — sherpa-onnx SenseVoice + Silero VAD。

VAD (Voice Activity Detection): 自动检测说话开始/结束，无需固定时长。
如果 VAD 模型不可用，回退固定 5 秒录音。
如果 ASR 模型不可用，回退键盘输入。
"""

import os
import sys
import time
import wave
import tempfile
import numpy as np
from pathlib import Path

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
        model_path = _find_model()
        if model_path is None:
            print("[ASR] 模型未找到，使用键盘输入回退。")
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
            print("[ASR] SenseVoice 已加载")

            # VAD
            vad_path = _find_vad_model()
            if vad_path:
                import sherpa_onnx as so
                svc = so.SileroVadModelConfig(model=vad_path, threshold=0.5,
                    min_silence_duration=1.2, min_speech_duration=0.3)
                vc = so.VadModelConfig(silero_vad=svc, sample_rate=16000)
                self._vad = so.VoiceActivityDetector(vc, buffer_size_in_seconds=30)
                print("[ASR] Silero VAD 已加载")
        except Exception as e:
            print(f"[ASR] 模型加载失败: {e}，回退键盘")

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
        """先录长段音频，再用 VAD 切割出说话段。

        避免了 pyaudio 依赖，直接用 arecord + 离线 VAD 分割。
        """
        if self._vad is None:
            return None

        if prompt:
            print(f"\n🤖 {prompt}")
        print("🎤 请说话（自动检测结束）...")

        # 录最长 8 秒（VAD 比固定更快，不需要太长）
        max_dur = 8
        wav_full = tempfile.mktemp(suffix=".wav")
        try:
            import subprocess
            subprocess.run(
                ["arecord", "-D", "pulse", "-f", "S16_LE",
                 "-r", "16000", "-c", "1", "-d", str(max_dur), wav_full],
                capture_output=True, timeout=max_dur + 10)
        except subprocess.TimeoutExpired:
            pass
        except Exception:
            pass

        if os.path.getsize(wav_full) < 1000:
            os.unlink(wav_full)
            return None

        # 用 VAD 切成语音段
        with wave.open(wav_full, "rb") as wf:
            nframes = wf.getnframes()
            audio = np.frombuffer(wf.readframes(nframes),
                                  dtype=np.int16).astype(np.float32) / 32768.0
            sr = wf.getframerate()
        os.unlink(wav_full)

        # 初始化新 VAD session
        import sherpa_onnx as so
        vad_path = _find_vad_model()
        svc = so.SileroVadModelConfig(model=vad_path, threshold=0.4,
            min_silence_duration=1.0, min_speech_duration=0.3)
        vc = so.VadModelConfig(silero_vad=svc, sample_rate=16000)
        vad = so.VoiceActivityDetector(vc, buffer_size_in_seconds=30)

        segments = []
        in_speech = False
        seg_start = 0
        chunk = 512
        for i in range(0, len(audio) - chunk, chunk):
            vad.accept_waveform(audio[i:i+chunk])
            if vad.is_speech_detected():
                if not in_speech:
                    seg_start = i
                    in_speech = True
            else:
                if in_speech:
                    segments.append((seg_start, i))
                    in_speech = False
        vad.flush()
        if in_speech:
            segments.append((seg_start, len(audio)))

        if not segments:
            return None

        # 取最长语音段
        best = max(segments, key=lambda s: s[1] - s[0])
        start_samp, end_samp = best
        dur = (end_samp - start_samp) / sr
        print(f"  🔊 {dur:.1f}s 语音段")

        wav_out = tempfile.mktemp(suffix=".wav")
        seg = audio[start_samp:end_samp]
        with wave.open(wav_out, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(sr)
            wf.writeframes((seg * 32768).astype(np.int16).tobytes())
        return wav_out

    def _record_fixed(self, duration: float = 5.0) -> str | None:
        """固定时长录音（VAD 不可用时回退）。"""
        import subprocess
        wav = tempfile.mktemp(suffix=".wav")
        print(f"🎤 录音 {duration} 秒...")
        try:
            subprocess.run(
                ["arecord", "-D", "pulse", "-f", "S16_LE",
                 "-r", "16000", "-c", "1", "-d", str(int(duration)), wav],
                capture_output=True, timeout=int(duration) + 8)
            if os.path.getsize(wav) > 1000:
                return wav
        except Exception as e:
            print(f"[ASR] 录音失败: {e}")
        return None

    def listen(self, prompt: str = "", duration: float = 5.0) -> str:
        """语音输入。VAD > 固定时长 > 键盘。"""
        if self._recognizer is None and self._vad is None:
            if prompt:
                print(f"\n🤖 {prompt}")
            return input("🎤 请输入: ").strip()

        wav_path = None
        # 优先 VAD
        if self._vad is not None:
            wav_path = self._record_vad(prompt)
        # 回退固定时长（VAD 失败后等 PulseAudio 释放设备）
        if wav_path is None and self._recognizer is not None:
            time.sleep(0.5)
            wav_path = self._record_fixed(duration)

        if wav_path is not None:
            try:
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
