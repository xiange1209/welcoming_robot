#!/usr/bin/env python3
"""拿實錄的 WAV 餵給 sherpa-onnx，看 Astra S 的麥克風到底認不認得出文字。"""
import sys, time, wave, struct
import numpy as np
import sherpa_onnx

# 2026-08-10：模型原本放在 /tmp 的 scratchpad（重開機可能被清），已搬到家目錄
MD = "/home/user/models/sherpa-onnx-streaming-zipformer-bilingual-zh-en-2023-02-20"


def load_mono(path):
    w = wave.open(path, "rb")
    n, ch, sr = w.getnframes(), w.getnchannels(), w.getframerate()
    d = struct.unpack("<%dh" % (n * ch), w.readframes(n))
    a = np.array(d, dtype=np.float32) / 32768.0
    if ch == 2:
        a = a.reshape(-1, 2).mean(axis=1)      # 兩聲道取平均當單聲道
    return a, sr


def main():
    t0 = time.time()
    rec = sherpa_onnx.OnlineRecognizer.from_transducer(
        tokens=f"{MD}/tokens.txt",
        encoder=f"{MD}/encoder-epoch-99-avg-1.onnx",
        decoder=f"{MD}/decoder-epoch-99-avg-1.onnx",
        joiner=f"{MD}/joiner-epoch-99-avg-1.int8.onnx",
        num_threads=2,
        sample_rate=16000,
        feature_dim=80,
        decoding_method="greedy_search",
    )
    print(f"  模型載入 {time.time() - t0:.1f} 秒")

    for path, label in ((sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else ""),):
        audio, sr = load_mono(path)
        dur = len(audio) / sr
        t1 = time.time()
        s = rec.create_stream()
        s.accept_waveform(sr, audio)
        # 尾端補靜音，讓最後一個字有機會吐出來
        s.accept_waveform(sr, np.zeros(int(sr * 0.5), dtype=np.float32))
        s.input_finished()
        while rec.is_ready(s):
            rec.decode_stream(s)
        el = time.time() - t1
        txt = rec.get_result(s)
        print(f"\n  ── {label} ──")
        print(f"  音檔 {dur:.1f} 秒，辨識耗時 {el:.1f} 秒（即時率 {el/dur:.2f}x）")
        print(f"  辨識結果：「{txt}」" if txt.strip() else "  辨識結果：（空的，沒認出任何字）")


if __name__ == "__main__":
    main()
