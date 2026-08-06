#!/bin/bash
# 下载 SenseVoice 离线语音识别模型 (~166MB)
# 用法: bash download_model.sh

set -e
MODEL_DIR="$(cd "$(dirname "$0")" && pwd)/models/asr/sensevoice"
mkdir -p "$MODEL_DIR"

echo "下载 SenseVoice 模型到 $MODEL_DIR"

# 优先用代理下载
if curl -sI https://hf-mirror.com >/dev/null 2>&1; then
    BASE="https://hf-mirror.com/csukuangfj/sherpa-onnx-sense-voice-zh-en-ja-ko-yue-2024-07-17/resolve/main"
else
    BASE="https://github.com/k2-fsa/sherpa-onnx/releases/download/asr-models/sherpa-onnx-sense-voice-zh-en-ja-ko-yue-2024-07-17"
fi

echo "源: $BASE"

for f in model.onnx tokens.txt; do
    echo "  -> $f"
    wget -c -q --show-progress "$BASE/$f" -O "$MODEL_DIR/$f" || \
        curl -C - -L -o "$MODEL_DIR/$f" "$BASE/$f"
done

echo ""
echo "完成。模型文件:"
ls -lh "$MODEL_DIR/"
echo ""
echo "现在可以用: python3 competition_node.py --sim"
