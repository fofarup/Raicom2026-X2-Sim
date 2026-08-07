#!/bin/bash
# 下载离线语音模型:
#   - SenseVoice ASR 语音识别 (~166MB)  models/asr/sensevoice/
#   - piper TTS 语音合成 (~63MB)        models/tts/
# 用法: bash download_model.sh
# 另需 pip 依赖: pip3 install piper-tts edge-tts

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# 国内可访问 hf-mirror.com，否则走 github 发布
if curl -sI https://hf-mirror.com >/dev/null 2>&1; then
    HF_BASE="https://hf-mirror.com"
else
    HF_BASE="https://huggingface.co"
fi

download() {  # $1=URL $2=本地路径
    echo "  -> $(basename "$2")"
    wget -c -q --show-progress "$1" -O "$2" || \
        curl -C - -L -o "$2" "$1"
}

# ---------- 1. SenseVoice ASR ----------
ASR_DIR="$SCRIPT_DIR/models/asr/sensevoice"
mkdir -p "$ASR_DIR"
echo "下载 SenseVoice 模型到 $ASR_DIR"
if curl -sI "$HF_BASE" >/dev/null 2>&1; then
    ASR_BASE="$HF_BASE/csukuangfj/sherpa-onnx-sense-voice-zh-en-ja-ko-yue-2024-07-17/resolve/main"
else
    ASR_BASE="https://github.com/k2-fsa/sherpa-onnx/releases/download/asr-models/sherpa-onnx-sense-voice-zh-en-ja-ko-yue-2024-07-17"
fi
for f in model.onnx tokens.txt; do
    download "$ASR_BASE/$f" "$ASR_DIR/$f"
done

# ---------- 2. piper TTS ----------
TTS_DIR="$SCRIPT_DIR/models/tts"
mkdir -p "$TTS_DIR"
echo "下载 piper TTS 模型 (zh_CN-huayan-medium) 到 $TTS_DIR"
PIPER_REPO="rhasspy/piper-voices/resolve/v1.0.0/zh/zh_CN/huayan/medium"
for f in zh_CN-huayan-medium.onnx zh_CN-huayan-medium.onnx.json; do
    download "$HF_BASE/$PIPER_REPO/$f" "$TTS_DIR/$f"
done

echo ""
echo "完成。模型文件:"
ls -lh "$SCRIPT_DIR/models/asr/sensevoice/" "$SCRIPT_DIR/models/tts/"
echo ""
echo "安装依赖:"
echo "  pip3 install sherpa-onnx piper-tts edge-tts"
echo ""
echo "现在可以用: python3 competition_node.py --sim"
