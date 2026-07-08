#!/bin/bash

set -e

# 定義目標根目錄
TARGET_ROOT="./src/smartnav_audio"

if [ ! -d "$TARGET_ROOT" ]; then
    echo "錯誤: 找不到目錄 $TARGET_ROOT"
    echo "請確保你在專案根目錄下執行此腳本，或者檢查目錄結構是否正確"
    exit 1
fi

# 定義模型存放目錄
MODEL_DIR="$TARGET_ROOT/models"
mkdir -p "$MODEL_DIR"

echo "開始下載模型至 $MODEL_DIR..."

# --- ASR (Automatic Speech Recognition) ---
echo "正在處理 ASR..."
mkdir -p "$MODEL_DIR/asr"
wget -c -P "$MODEL_DIR/asr" https://github.com/k2-fsa/sherpa-onnx/releases/download/asr-models/sherpa-onnx-streaming-zipformer-bilingual-zh-en-2023-02-20-mobile.tar.bz2
tar xvf "$MODEL_DIR/asr/sherpa-onnx-streaming-zipformer-bilingual-zh-en-2023-02-20-mobile.tar.bz2" -C "$MODEL_DIR/asr" --strip-components=1
rm "$MODEL_DIR/asr/sherpa-onnx-streaming-zipformer-bilingual-zh-en-2023-02-20-mobile.tar.bz2"

# --- VAD (Voice Activity Detection) ---
echo "正在處理 VAD..."
mkdir -p "$MODEL_DIR/vad"
wget -c -P "$MODEL_DIR/vad/" https://github.com/k2-fsa/sherpa-onnx/releases/download/asr-models/silero_vad.onnx

# --- TTS (Text to Speech) ---
echo "正在處理 TTS..."
mkdir -p "$MODEL_DIR/tts"
wget -c -P "$MODEL_DIR/tts" https://github.com/k2-fsa/sherpa-onnx/releases/download/tts-models/sherpa-onnx-zipvoice-distill-int8-zh-en-emilia.tar.bz2
tar xvf "$MODEL_DIR/tts/sherpa-onnx-zipvoice-distill-int8-zh-en-emilia.tar.bz2" -C "$MODEL_DIR/tts" --strip-components=1
rm "$MODEL_DIR/tts/sherpa-onnx-zipvoice-distill-int8-zh-en-emilia.tar.bz2"

# --- Vocoder ---
echo "正在處理 Vocoder..."
wget -c -P "$MODEL_DIR/tts/" https://github.com/k2-fsa/sherpa-onnx/releases/download/vocoder-models/vocos_24khz.onnx

echo "所有模型已成功安裝於 $MODEL_DIR"
ls -R "$MODEL_DIR"
