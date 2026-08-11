#!/usr/bin/env python3
"""Offline Chinese ASR for RAICOM 2026.

The scoring runtime must not depend on a cloud service.  This module uses the
local SenseVoice INT8 model through sherpa-onnx, then feeds the recognized text
to Task3's deterministic need classifier.  It intentionally does not command
the base or arms; motion is enabled only by a separate, explicit task runner.
"""

from __future__ import annotations

import argparse
import re
import sys
import time
import wave
from dataclasses import dataclass
from pathlib import Path

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_MODEL_DIR = SCRIPT_DIR / "models" / "asr" / "sensevoice"
DEFAULT_MODEL = DEFAULT_MODEL_DIR / "model.int8.onnx"
DEFAULT_TOKENS = DEFAULT_MODEL_DIR / "tokens.txt"
TARGET_SAMPLE_RATE = 16_000


@dataclass(frozen=True)
class AudioData:
    samples: np.ndarray
    sample_rate: int
    duration_s: float


def _decode_pcm(raw: bytes, sample_width: int) -> np.ndarray:
    if sample_width == 1:
        return (np.frombuffer(raw, dtype=np.uint8).astype(np.float32) - 128.0) / 128.0
    if sample_width == 2:
        return np.frombuffer(raw, dtype="<i2").astype(np.float32) / 32768.0
    if sample_width == 3:
        data = np.frombuffer(raw, dtype=np.uint8).reshape(-1, 3)
        value = (
            data[:, 0].astype(np.int32)
            | (data[:, 1].astype(np.int32) << 8)
            | (data[:, 2].astype(np.int32) << 16)
        )
        value = np.where(value & 0x800000, value - 0x1000000, value)
        return value.astype(np.float32) / 8388608.0
    if sample_width == 4:
        return np.frombuffer(raw, dtype="<i4").astype(np.float32) / 2147483648.0
    raise ValueError(f"不支持 {sample_width * 8}-bit PCM WAV")


def _resample_linear(samples: np.ndarray, source_rate: int, target_rate: int) -> np.ndarray:
    if source_rate == target_rate or samples.size == 0:
        return samples.astype(np.float32, copy=False)
    target_size = max(1, round(samples.size * target_rate / source_rate))
    source_x = np.arange(samples.size, dtype=np.float64)
    target_x = np.arange(target_size, dtype=np.float64) * source_rate / target_rate
    return np.interp(target_x, source_x, samples).astype(np.float32)


def load_wav(path: Path, target_rate: int = TARGET_SAMPLE_RATE) -> AudioData:
    with wave.open(str(path), "rb") as wav:
        if wav.getcomptype() != "NONE":
            raise ValueError(f"只支持未压缩 PCM WAV，实际为 {wav.getcomptype()}")
        channels = wav.getnchannels()
        sample_width = wav.getsampwidth()
        source_rate = wav.getframerate()
        frames = wav.getnframes()
        samples = _decode_pcm(wav.readframes(frames), sample_width)
    if channels < 1:
        raise ValueError("WAV 声道数无效")
    if samples.size % channels:
        raise ValueError("WAV PCM 数据长度与声道数不匹配")
    if channels > 1:
        samples = samples.reshape(-1, channels).mean(axis=1)
    samples = _resample_linear(samples, source_rate, target_rate)
    return AudioData(samples, target_rate, samples.size / target_rate)


def clean_sensevoice_text(text: str) -> str:
    # SenseVoice can prefix metadata such as <|zh|><|NEUTRAL|><|Speech|>.
    text = re.sub(r"<\|[^|>]+\|>", "", text)
    return " ".join(text.strip().split())


class OfflineChineseASR:
    def __init__(self, model: Path = DEFAULT_MODEL, tokens: Path = DEFAULT_TOKENS):
        missing = [str(path) for path in (model, tokens) if not path.is_file()]
        if missing:
            raise FileNotFoundError("缺少离线 ASR 文件：" + ", ".join(missing))
        try:
            import sherpa_onnx
        except ImportError as exc:
            raise RuntimeError("未安装 sherpa-onnx，请先运行离线依赖安装脚本") from exc
        started = time.perf_counter()
        self.recognizer = sherpa_onnx.OfflineRecognizer.from_sense_voice(
            model=str(model),
            tokens=str(tokens),
            num_threads=2,
            sample_rate=TARGET_SAMPLE_RATE,
            language="zh",
            use_itn=True,
            debug=False,
        )
        self.load_ms = (time.perf_counter() - started) * 1000

    def transcribe(self, audio: AudioData) -> tuple[str, float]:
        stream = self.recognizer.create_stream()
        stream.accept_waveform(audio.sample_rate, audio.samples)
        started = time.perf_counter()
        self.recognizer.decode_stream(stream)
        elapsed_ms = (time.perf_counter() - started) * 1000
        return clean_sensevoice_text(stream.result.text), elapsed_ms


def print_task3_decision(text: str) -> bool:
    from task3_needs import classify_need

    decision = classify_need(text)
    if decision is None:
        print("[语义] 未识别为 Task3 的头痛/口渴/饥饿需求；安全策略：不执行抓取")
        return False
    print(f"[语义] 需求={decision.need} 目标物品={decision.item} 置信度={decision.confidence:.2f}")
    print(f"[回答] {decision.answer}")
    return True


def main() -> None:
    parser = argparse.ArgumentParser(description="RAICOM 离线中文 ASR（默认不控制机器人）")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--wav", type=Path, help="识别一个 PCM WAV 文件")
    source.add_argument("--text", help="跳过声学识别，仅验证 Task3 语义判断")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--tokens", type=Path, default=DEFAULT_TOKENS)
    parser.add_argument("--no-task3", action="store_true", help="只输出识别文本")
    args = parser.parse_args()

    if args.text is not None:
        print(f"[文本] {args.text}")
        if not args.no_task3:
            raise SystemExit(0 if print_task3_decision(args.text) else 2)
        return

    audio = load_wav(args.wav)
    print(
        f"[音频] {args.wav} duration={audio.duration_s:.2f}s "
        f"sample_rate={audio.sample_rate} samples={audio.samples.size}"
    )
    asr = OfflineChineseASR(args.model, args.tokens)
    print(f"[模型] SenseVoice INT8 load={asr.load_ms:.0f}ms")
    text, decode_ms = asr.transcribe(audio)
    realtime_factor = decode_ms / max(1.0, audio.duration_s * 1000)
    print(f"[识别] {text}")
    print(f"[性能] decode={decode_ms:.0f}ms RTF={realtime_factor:.3f}")
    if not text:
        print("[错误] 未识别到语音", file=sys.stderr)
        raise SystemExit(3)
    if not args.no_task3:
        raise SystemExit(0 if print_task3_decision(text) else 2)


if __name__ == "__main__":
    main()
