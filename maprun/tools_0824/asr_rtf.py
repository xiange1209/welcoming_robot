#!/usr/bin/env python3
"""離線量 ASR 的即時率（RTF）—— 不需要麥克風、不需要人、不需要 ROS。

★ 為什麼要重量（2026-08-24）
專案裡唯一的 RTF 數字是 **0.76**，但那是 **2026-08-17 換成 int8 之前**、
encoder 還是 fp32 (315 MB) 時量的。8/17 把 encoder/decoder/joiner 全換成 int8
（encoder 182 MB）之後**從來沒有重量過**，而「ASR 餘裕只有 24%」這個結論
（以及所有以它為基礎的 CPU 排程推算）都還建立在那個過期數字上。

這支直接用**節點實際會載入的同一組模型檔**與同樣的 num_threads，
把 wav 一塊一塊餵進 OnlineRecognizer（跟節點的串流用法一致），
量「處理時間 ÷ 音訊長度」。

RTF < 1 才跟得上即時；RTF 0.76 表示每秒音訊要花 0.76 秒處理，餘裕 24%。

用法：
    python3 asr_rtf.py                 # 用模型附的 test_wavs，num_threads=2
    python3 asr_rtf.py 4               # num_threads=4
"""
import glob
import sys
import time
import wave
from pathlib import Path

import numpy as np
import sherpa_onnx

MODEL = Path("/home/user/models/sherpa-onnx-streaming-zipformer-bilingual-zh-en-2023-02-20")
THREADS = int(sys.argv[1]) if len(sys.argv) > 1 else 2
CHUNK = 0.1          # 每次餵 0.1 秒，跟節點的串流節奏一致


def pick(stem):
    """跟節點同一套規則：有 int8 就用 int8。"""
    c = MODEL / f"{stem}.int8.onnx"
    return str(c if c.exists() else MODEL / f"{stem}.onnx")


def main():
    enc, dec, joi = (pick("encoder-epoch-99-avg-1"),
                     pick("decoder-epoch-99-avg-1"),
                     pick("joiner-epoch-99-avg-1"))
    print("=" * 70)
    for p in (enc, dec, joi):
        n = Path(p)
        prec = "int8" if ".int8." in n.name else "fp32"
        print(f"  {n.name:42s} {prec}  {n.stat().st_size/1e6:6.1f} MB")
    print(f"  num_threads = {THREADS}")
    print("-" * 70)

    t0 = time.time()
    rec = sherpa_onnx.OnlineRecognizer.from_transducer(
        tokens=str(MODEL / "tokens.txt"),
        encoder=enc, decoder=dec, joiner=joi,
        num_threads=THREADS,
        provider="cpu",
        decoding_method="modified_beam_search",
    )
    print(f"  模型載入 {time.time()-t0:.1f} 秒")
    print("-" * 70)

    wavs = sorted(glob.glob(str(MODEL / "test_wavs" / "*.wav")))
    tot_audio = tot_proc = 0.0
    print(f"  {'檔案':10s} {'長度':>7s} {'處理':>7s} {'RTF':>7s}   辨識結果")
    for w in wavs:
        with wave.open(w) as f:
            sr = f.getframerate()
            if sr != 16000:          # 8k.wav 取樣率不合，模型是 16 kHz 訓練的
                print(f"  {Path(w).name:10s} {'—':>7s} {'—':>7s} {'跳過':>7s}   取樣率 {sr} Hz")
                continue
            n = f.getnframes()
            pcm = np.frombuffer(f.readframes(n), dtype=np.int16).astype(np.float32) / 32768.0
        dur = n / sr
        step = int(CHUNK * sr)
        s = rec.create_stream()
        t = time.time()
        for i in range(0, len(pcm), step):
            s.accept_waveform(sr, pcm[i:i + step])
            while rec.is_ready(s):
                rec.decode_stream(s)
        s.input_finished()
        while rec.is_ready(s):
            rec.decode_stream(s)
        proc = time.time() - t
        txt = rec.get_result(s)
        tot_audio += dur
        tot_proc += proc
        print(f"  {Path(w).name:10s} {dur:6.2f}s {proc:6.2f}s {proc/dur:7.3f}   {txt[:34]}")

    print("-" * 70)
    rtf = tot_proc / tot_audio if tot_audio else 0.0
    print(f"  合計 音訊 {tot_audio:.2f} 秒 / 處理 {tot_proc:.2f} 秒")
    print(f"  ★ RTF = {rtf:.3f}    餘裕 {(1-rtf)*100:.1f}%")
    if rtf >= 1.0:
        print("  ✗ RTF >= 1，跟不上即時 —— 音訊佇列會累積，最後靜默截斷語音")
    elif rtf > 0.8:
        print("  ⚠ 餘裕不到 20%，導航或人臉一起跑時很可能就不夠了")
    else:
        print("  ✓ 餘裕充足")
    print(f"  （對照：專案舊紀錄 0.76，但那是 fp32 encoder 時代量的）")
    print("=" * 70)


if __name__ == "__main__":
    main()
