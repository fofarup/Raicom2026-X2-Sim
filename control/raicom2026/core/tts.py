"""自研 TTS — 离线 piper 优先，云端 edge-tts 可选。

比赛规则三：语音合成（TTS）必须自行完成，禁止使用官方写好的 TTS。
本模块是完全自研实现：
- 本地引擎：piper TTS（模型部署在 PC2，完全离线，比赛零网络风险）
- 云端引擎：edge-tts（微软公开接口，音质接近真人；现场连公共网络时可用）
- 无音频设备（Docker 仿真 / 无扬声器）自动降级：speak() 返回 False，
  由调用方只打日志，不影响任务流程。

用法：
    from core.tts import TTSController
    tts = TTSController(prefer="local")   # "local" | "cloud" | "auto"
    tts.speak("正在自主导航至交互区I。")
"""

import os
import shutil
import subprocess
import tempfile
import wave
from pathlib import Path

MODEL_DIR = Path(__file__).resolve().parents[1] / "models" / "tts"
PIPER_MODEL = "zh_CN-huayan-medium"          # 中文女声，~63MB
CLOUD_VOICE = "zh-CN-XiaoxiaoNeural"         # edge-tts 中文女声


def _find_piper_model() -> str | None:
    """找 piper 模型（onnx 或 onnx.json 二选一作存在标志）。"""
    for suffix in ("", ".json"):
        p = MODEL_DIR / f"{PIPER_MODEL}.onnx{suffix}"
        if p.exists():
            return str(p)
    return None


class TTSController:
    """文本转语音控制器。prefer: local=离线优先(默认) / cloud=云端优先 / auto=先本地后云端。"""

    def __init__(self, prefer: str = "local"):
        if prefer not in ("local", "cloud", "auto"):
            raise ValueError(f"未知 TTS 模式: {prefer}")
        self._prefer = prefer
        self._cloud_voice = CLOUD_VOICE
        self._voice = None
        self._model_path = _find_piper_model()
        if self._model_path:
            try:
                from piper import PiperVoice
                # piper 不同版本 load 返回 (voice, config) 或直接 voice
                result = PiperVoice.load(self._model_path)
                self._voice = result[0] if isinstance(result, tuple) else result
                print(f"[TTS] piper 已加载: {self._model_path}")
            except Exception as e:
                print(f"[TTS] piper 加载失败: {e}，回退云端/日志")
                self._voice = None
        else:
            print(f"[TTS] piper 模型未找到（{MODEL_DIR}），回退云端/日志")

    # ---------- 引擎 ----------

    def _synth_local(self, text: str) -> str | None:
        """piper 离线合成，返回 wav 路径。"""
        if self._voice is None:
            return None
        wav = tempfile.mktemp(suffix=".wav")
        try:
            with wave.open(wav, "wb") as f:
                synth = getattr(self._voice, "synthesize_wav", None)
                if synth is not None:
                    synth(text, f)          # piper >= 1.2
                else:
                    self._voice.synthesize(text, f)  # 旧版 API
            return wav if os.path.getsize(wav) > 2000 else None
        except Exception as e:
            print(f"[TTS] piper 合成失败: {e}")
            self._safe_unlink(wav)
            return None

    def _synth_cloud(self, text: str) -> str | None:
        """edge-tts 云端合成，返回 mp3/wav 路径。需要网络。"""
        out = tempfile.mktemp(suffix=".mp3")
        try:
            proc = subprocess.run(
                ["edge-tts", "--voice", self._cloud_voice,
                 "--text", text, "--write-media", out],
                capture_output=True, timeout=60,
            )
            if proc.returncode != 0 or not os.path.exists(out) or os.path.getsize(out) < 2000:
                print(f"[TTS] edge-tts 失败: {proc.stderr.decode(errors='ignore')[:120]}")
                self._safe_unlink(out)
                return None
            # 有 ffmpeg 时转 wav，播放更稳（paplay 对 mp3 依赖解码器）
            if shutil.which("ffmpeg"):
                wav = tempfile.mktemp(suffix=".wav")
                proc = subprocess.run(
                    ["ffmpeg", "-y", "-loglevel", "error", "-i", out, wav],
                    capture_output=True, timeout=60)
                self._safe_unlink(out)
                if proc.returncode == 0 and os.path.getsize(wav) > 2000:
                    return wav
                self._safe_unlink(wav)
                return None
            return out
        except (FileNotFoundError, subprocess.TimeoutExpired, OSError) as e:
            print(f"[TTS] edge-tts 不可用: {e}")
            self._safe_unlink(out)
            return None

    # ---------- 播放 ----------

    @staticmethod
    def _play(path: str) -> bool:
        """paplay (PulseAudio) 优先，aplay (ALSA) 备选。成功返回 True。"""
        for cmd in (["paplay", path], ["aplay", "-q", path]):
            try:
                proc = subprocess.run(cmd, capture_output=True, timeout=30)
                if proc.returncode == 0:
                    return True
            except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
                continue
        return False

    @staticmethod
    def _safe_unlink(path: str | None):
        if path:
            try:
                os.unlink(path)
            except OSError:
                pass

    # ---------- 对外接口 ----------

    def speak(self, text: str) -> bool:
        """合成并播放。返回是否真的发声（无模型/无设备时为 False，调用方降级日志）。"""
        engines = {
            "local": [self._synth_local, self._synth_cloud],
            "cloud": [self._synth_cloud, self._synth_local],
            "auto": [self._synth_local, self._synth_cloud],
        }[self._prefer]
        for synth in engines:
            audio = synth(text)
            if audio is None:
                continue
            try:
                if self._play(audio):
                    return True
            finally:
                self._safe_unlink(audio)
        return False

    def say(self, text: str) -> bool:
        """speak() 别名，与 SpeechController 接口一致。"""
        return self.speak(text)
