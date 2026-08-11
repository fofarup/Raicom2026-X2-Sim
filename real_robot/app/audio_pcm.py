#!/usr/bin/env python3
"""ROS-independent PCM helpers shared by simulation and the real robot."""

from __future__ import annotations

import wave
from pathlib import Path


SAMPLE_RATE = 16_000
SAMPLE_WIDTH_BYTES = 2
CHANNELS = 1


def validate_pcm_s16le(data: bytes, max_seconds: float = 30.0) -> None:
    if len(data) % (SAMPLE_WIDTH_BYTES * CHANNELS):
        raise ValueError("S16LE PCM byte length must be even")
    maximum = int(max_seconds * SAMPLE_RATE * SAMPLE_WIDTH_BYTES * CHANNELS)
    if len(data) > maximum:
        raise ValueError(f"PCM exceeds {max_seconds:.1f}s safety limit")


def write_pcm_wav(path: Path, data: bytes) -> None:
    validate_pcm_s16le(data)
    path.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(path), "wb") as wav:
        wav.setnchannels(CHANNELS)
        wav.setsampwidth(SAMPLE_WIDTH_BYTES)
        wav.setframerate(SAMPLE_RATE)
        wav.writeframes(data)


def read_pcm_wav(path: Path) -> bytes:
    with wave.open(str(path), "rb") as wav:
        actual = (wav.getframerate(), wav.getsampwidth(), wav.getnchannels())
        expected = (SAMPLE_RATE, SAMPLE_WIDTH_BYTES, CHANNELS)
        if actual != expected or wav.getcomptype() != "NONE":
            raise ValueError(
                f"WAV must be PCM 16kHz/16bit/mono, got rate={actual[0]} "
                f"width={actual[1] * 8}bit channels={actual[2]} codec={wav.getcomptype()}"
            )
        data = wav.readframes(wav.getnframes())
    validate_pcm_s16le(data)
    return data


def pcm_chunks(data: bytes, milliseconds: int = 40):
    if milliseconds <= 0:
        raise ValueError("chunk duration must be positive")
    validate_pcm_s16le(data)
    size = SAMPLE_RATE * SAMPLE_WIDTH_BYTES * CHANNELS * milliseconds // 1000
    size -= size % (SAMPLE_WIDTH_BYTES * CHANNELS)
    for offset in range(0, len(data), size):
        yield data[offset : offset + size]
