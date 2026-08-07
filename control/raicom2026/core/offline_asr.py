"""语音识别 — SenseVoice 本地优先 + 词表纠错，讯飞云端保底。

链路：
  麦克风 → VAD(0.5s静默) → SenseVoice 本地识别(~0.5s) → 词表纠错 → 结果
  SenseVoice 失败/低置信 → 讯飞云端(需网络) → 键盘回退

录音方式：ALSA plughw 直录 WAV（PulseAudio 在某些声卡上有 bug，
parec 只出 0.2s 数据，因此绕过 PulseAudio）。

比赛规则三：语音交互豁免云端算力限制，讯飞云端合规。
"""

import os
import sys
import time
import wave
import tempfile
import numpy as np
from pathlib import Path
from difflib import SequenceMatcher

from .cloud_asr import CloudASR

MODEL_DIR = Path(__file__).resolve().parents[1] / "models" / "asr"

# ---- 赛事词表（比赛所有可能的语音输入）----
COMPETITION_KEYWORDS = {
    "时间": ["时间", "几点了", "几点", "现在几点", "时间是多少"],
    "数字": [str(i) for i in range(10)] + ["零", "一", "二", "三", "四", "五", "六", "七", "八", "九"],
    "颜色": ["粉色", "青色", "绿色", "黄色", "紫色", "深橙色", "蓝绿色",
             "蓝色", "浅蓝色", "红色", "深紫色", "靛蓝色", "黄绿色", "橙色", "浅绿色"],
    "表情": ["快乐", "悲伤", "睡觉", "愤怒", "充电"],
    "动作": ["挥左手", "挥右手", "左手敬礼", "右手敬礼", "双手打叉",
             "挥手", "敬礼", "打叉"],
    "需求": ["拿杯子", "拿水杯", "取杯子", "取水杯",
             "拿瓶子", "取瓶子", "拿水瓶", "取水瓶",
             "拿盒子", "取盒子"],
    "确认": ["前往", "开始", "确认", "是的", "对", "好的", "可以", "行"],
}


def _keyword_correct(text: str, context: str | None = None) -> str:
    """用赛事词表纠正识别结果。context 用于限定词表范围（如 '表情' 只匹配表情词）。"""
    if not text or not text.strip():
        return text

    # 如果指定了 context，只用该词表
    if context and context in COMPETITION_KEYWORDS:
        candidates = COMPETITION_KEYWORDS[context]
        best = max(candidates, key=lambda w: SequenceMatcher(None, text, w).ratio())
        score = SequenceMatcher(None, text, best).ratio()
        if score > 0.4:
            return best
        return text

    # 否则用全词表找最佳匹配
    best_word, best_score = text, 0.0
    for words in COMPETITION_KEYWORDS.values():
        for w in words:
            s = SequenceMatcher(None, text, w).ratio()
            if s > best_score:
                best_score, best_word = s, w
    if best_score > 0.5:
        return best_word
    return text


def _find_model() -> str | None:
    """搜模型目录（含 model.onnx + tokens.txt）。"""
    if not MODEL_DIR.is_dir():
        return None
    for d in MODEL_DIR.iterdir():
        if d.is_dir() and (d / "model.onnx").exists() and (d / "tokens.txt").exists():
            return str(d)
    return None


def _find_vad_model() -> str | None:
    p = MODEL_DIR / "silero_vad.onnx"
    return str(p) if p.exists() else None


class OfflineASR:
    """语音识别器：SenseVoice 本地优先 → 讯飞云端 → 键盘。"""

    def __init__(self):
        self._recognizer = None
        self._vad = None
        self._cloud = CloudASR()
        if self._cloud.available():
            print("[ASR] 讯飞云端识别已加载（保底）")
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
                print("[ASR] SenseVoice 已加载（优先）")

                vad_path = _find_vad_model()
                if vad_path:
                    import sherpa_onnx as so
                    svc = so.SileroVadModelConfig(
                        model=vad_path, threshold=0.5,
                        min_silence_duration=0.5, min_speech_duration=0.3)
                    vc = so.VadModelConfig(silero_vad=svc, sample_rate=16000)
                    self._vad = so.VoiceActivityDetector(vc, buffer_size_in_seconds=30)
                    print("[ASR] Silero VAD 已加载")
            except Exception as e:
                print(f"[ASR] 模型加载失败: {e}，仅云端模式")

    def recognize_file(self, wav_path: str) -> str:
        """识别 WAV 文件。本地优先，失败回退云端。"""
        # 1. 本地 SenseVoice
        if self._recognizer is not None:
            try:
                with wave.open(wav_path, "rb") as wf:
                    frames = wf.readframes(wf.getnframes())
                    audio = np.frombuffer(frames, dtype=np.int16).astype(np.float32) / 32768.0
                    sr = wf.getframerate()
                stream = self._recognizer.create_stream()
                stream.accept_waveform(sr, audio)
                self._recognizer.decode_stream(stream)
                result = stream.result.text.strip()
                if result:
                    return result
            except Exception:
                pass

        # 2. 讯飞云端
        if self._cloud.available():
            try:
                with wave.open(wav_path, "rb") as wf:
                    pcm = wf.readframes(wf.getnframes())
                text = self._cloud.recognize(pcm)
                if text:
                    return text
            except Exception:
                pass

        return ""

    # ---------- 录音 ----------

    @staticmethod
    def _record_raw(duration: int, wav_path: str) -> bool:
        """ALSA plughw 直录 WAV（绕过 PulseAudio bug）。返回是否成功。"""
        os.system(
            f"timeout {duration} arecord -D plughw:CARD=PCH,DEV=0"
            f" -f S16_LE -r 16000 -c 1 -t wav {wav_path} 2>/dev/null"
        )
        return os.path.getsize(wav_path) > 2000

    def _record_vad(self, prompt: str = "") -> str | None:
        """录长段音频，VAD 切出说话段，返回 WAV 路径。"""
        if self._vad is None:
            return None

        if prompt:
            print(f"\n🤖 {prompt}")

        max_dur = 4
        wav_full = tempfile.mktemp(suffix=".wav")
        print(f"🎤 录音 {max_dur}s（说话自动检测）...")
        if not self._record_raw(max_dur, wav_full):
            return None

        # 读 WAV
        try:
            with wave.open(wav_full, "rb") as wf:
                sr = wf.getframerate()
                audio = np.frombuffer(wf.readframes(wf.getnframes()),
                                      dtype=np.int16).astype(np.float32) / 32768.0
        except Exception:
            os.unlink(wav_full)
            return None
        os.unlink(wav_full)

        if len(audio) < 1600:
            return None

        # VAD 找语音段
        import sherpa_onnx as so
        vad_path = _find_vad_model()
        svc = so.SileroVadModelConfig(
            model=vad_path, threshold=0.5,
            min_silence_duration=0.5, min_speech_duration=0.3)
        vc = so.VadModelConfig(silero_vad=svc, sample_rate=16000)
        vad = so.VoiceActivityDetector(vc, buffer_size_in_seconds=30)

        segments = []
        in_speech, seg_start = False, 0
        chunk = 512
        for i in range(0, len(audio) - chunk, chunk):
            vad.accept_waveform(audio[i:i + chunk])
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
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(sr)
            wf.writeframes((seg * 32768).astype(np.int16).tobytes())
        return wav_out

    def _record_fixed(self, duration: float = 4.0) -> str | None:
        """固定时长录音，返回 WAV 路径。"""
        wav = tempfile.mktemp(suffix=".wav")
        d = int(duration)
        print(f"🎤 录音 {d} 秒...")
        if self._record_raw(d, wav):
            return wav
        return None

    # ---------- 对外接口 ----------

    def listen(self, prompt: str = "", duration: float = 5.0,
               context: str | None = None) -> str:
        """语音输入。SenseVoice(+词表纠错) → 讯飞云端 → 键盘。

        context: 限定词表范围（'表情'/'动作'/'需求'/'颜色'/'数字'/'时间'）
        """
        # 无任何识别能力 → 键盘
        if self._recognizer is None and not self._cloud.available():
            if prompt:
                print(f"\n🤖 {prompt}")
            return input("🎤 请输入: ").strip()

        wav_path = None
        # 优先 VAD
        if self._vad is not None:
            wav_path = self._record_vad(prompt)
        # 回退固定时长
        if wav_path is None:
            time.sleep(0.3)
            wav_path = self._record_fixed(duration)

        if wav_path is not None:
            try:
                result = self.recognize_file(wav_path)
                if result:
                    # 词表纠错
                    corrected = _keyword_correct(result, context)
                    if corrected != result:
                        print(f"🎤 识别: {result} → 纠错: {corrected}")
                    else:
                        print(f"🎤 识别: {result}")
                    return corrected
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
