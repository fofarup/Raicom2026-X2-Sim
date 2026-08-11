#!/usr/bin/env python3
"""Generate a 16 kHz mono PCM WAV with edge-tts for the raw X2 speaker path."""

import argparse
import os
import shutil
import subprocess
import tempfile
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--text", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--voice", default="zh-CN-XiaoxiaoNeural")
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="raicom_edge_tts_") as directory:
        media = Path(directory) / "speech.mp3"
        subprocess.run(
            ["/home/agi/.local/bin/edge-tts", "--voice", args.voice,
             "--text", args.text, "--write-media", str(media)],
            check=True,
        )
        ffmpeg = shutil.which("ffmpeg") or "/home/agi/.local/bin/ffmpeg"
        if not Path(ffmpeg).is_file():
            raise FileNotFoundError("PATH中没有ffmpeg")
        ffmpeg_env = os.environ.copy()
        private_lib = "/home/agi/.local/lib/raicom_ffmpeg"
        ffmpeg_env["LD_LIBRARY_PATH"] = private_lib + ":" + ffmpeg_env.get("LD_LIBRARY_PATH", "")
        subprocess.run(
            [ffmpeg, "-loglevel", "error", "-y", "-i", str(media),
             "-ac", "1", "-ar", "16000", "-c:a", "pcm_s16le", str(args.output)],
            env=ffmpeg_env,
            check=True,
        )


if __name__ == "__main__":
    main()
