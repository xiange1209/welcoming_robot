#!/usr/bin/env python3
"""語音辨識離線基準（不用麥克風）：TTS 合成測試句 → 混入實驗室真實背景音 → 用車上同一套 ASR 設定辨識。

  - 測試句：asr_cer.py 的 10 句（5 句含熱詞）；TTS：sherpa-onnx vits-icefall-zh-aishell3（多語者，取 6 位）
  - 背景音：~/maprun/tools_0817/rec 的 Astra S 真實錄音（左聲道），依 SNR 縮放後疊上去
  - ASR：與 speech_recognizer_node 相同——streaming zipformer（精度組合見 --configs）、2 執行緒、modified_beam_search、
    beam 4、熱詞 1.8（熱詞檔濾掉 # 行）；餵法照 voice_trigger：前面 0.48 s（pre-roll 15×512）、
    句尾 0.5 s（silence_timeout）後 input_finished()，**不補 tail padding**（節點也沒補）
  - 量：CER（嚴格／寬鬆，同 asr_cer.py）、RTF（解碼 CPU 時間 ÷ 音訊長度）、收尾延遲（最後一塊送進去到結果出來）
  ★ 合成語音是「乾淨、標準口音」的上限情境，實車的 42~51% 才是現場值；這裡拿來比較設定與雜訊的影響。

用法：python3 asr_bench.py --tts-dir <vits-icefall-zh-aishell3> [--snr inf 20 10 5] [--speakers 3 21 45 66 99 150]
"""
import argparse
import csv
import glob
import os
import statistics
import sys
import tempfile
import time
import wave

import numpy as np
import sherpa_onnx
from scipy.signal import resample_poly

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from asr_cer import SENTENCES, ed_ops, norm_lenient, norm_strict  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--tts-dir", required=True)
ap.add_argument("--asr-dir", default=os.path.expanduser("~/models/asr"))
ap.add_argument("--hotwords", default=os.path.expanduser("~/models/asr/hotwords.txt"))
ap.add_argument("--noise", default=os.path.expanduser("~/maprun/tools_0817/rec/實驗室講話_171655.wav"))
ap.add_argument("--snr", nargs="+", default=["inf", "20", "10", "5"])
ap.add_argument("--speakers", nargs="+", type=int, default=[3, 21, 45, 66, 99, 150])
ap.add_argument("--threads", type=int, default=2)
ap.add_argument("--cut-ms", nargs="+", type=int, default=[0], help="把句首砍掉幾 ms（模擬 VAD 太晚觸發）")
ap.add_argument("--configs", nargs="+", default=["現況全int8=encoder,decoder,joiner", "decoder改fp32=encoder,joiner"],
                help="要比較的精度組合：名稱=用 int8 的元件（逗號分隔；空白＝全 fp32）")
ap.add_argument("--out", default=os.path.expanduser(f"~/maprun/verify_out/asr_bench_{time.strftime('%H%M')}"))
a = ap.parse_args()
os.makedirs(a.out, exist_ok=True)
SR = 16000
rng = np.random.default_rng(20261022)

try:
    from opencc import OpenCC

    T2S = OpenCC("t2s")
except Exception:  # noqa: BLE001
    sys.exit("需要 opencc（TTS 詞典是簡體）")

# ── TTS ─────────────────────────────────────────────
d = a.tts_dir
tts = sherpa_onnx.OfflineTts(
    sherpa_onnx.OfflineTtsConfig(
        model=sherpa_onnx.OfflineTtsModelConfig(
            vits=sherpa_onnx.OfflineTtsVitsModelConfig(
                model=f"{d}/model.onnx", lexicon=f"{d}/lexicon.txt", tokens=f"{d}/tokens.txt"
            ),
            num_threads=2,
            provider="cpu",
        ),
        rule_fsts=",".join(f"{d}/{x}" for x in ("phone.fst", "date.fst", "number.fst", "new_heteronym.fst")),
        rule_fars=f"{d}/rule.far",
        max_num_sentences=1,
    )
)
print(f"TTS 取樣率 {tts.sample_rate} Hz、語者數 {tts.num_speakers}")


def synth(text, sid):
    au = tts.generate(T2S.convert(text), sid=sid, speed=1.0)
    x = np.asarray(au.samples, dtype=np.float32)
    if au.sample_rate != SR:
        g = np.gcd(SR, au.sample_rate)
        x = resample_poly(x, SR // g, au.sample_rate // g).astype(np.float32)
    # 去掉前後靜音（TTS 會留一點），讓「句首」對齊真正開始發音的位置
    env = np.abs(x)
    thr = 0.02 * env.max()
    idx = np.where(env > thr)[0]
    x = x[max(0, idx[0] - 160): idx[-1] + 160]
    return x / (np.abs(x).max() + 1e-9) * 0.5


# ── 背景音（Astra S 真實錄音，左聲道）────────────────────
with wave.open(a.noise) as w:
    raw = np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16).reshape(-1, w.getnchannels())
noise_all = raw[:, 0].astype(np.float32) / 32768.0
print(f"背景音 {os.path.basename(a.noise)}：{len(noise_all) / SR:.1f} s，RMS {np.sqrt(np.mean(noise_all ** 2)):.4f}")


def mix(speech, snr_db, lead=0.48, tail=0.5):
    n = int((lead + tail) * SR) + len(speech)
    st = rng.integers(0, len(noise_all) - n)
    nz = noise_all[st: st + n].copy()
    y = np.zeros(n, np.float32)
    y[int(lead * SR): int(lead * SR) + len(speech)] = speech
    if snr_db == float("inf"):
        return y + nz * 0.0
    ps = np.mean(speech ** 2)
    pn = np.mean(nz ** 2) + 1e-12
    return y + nz * np.sqrt(ps / (pn * 10 ** (snr_db / 10)))


# ── ASR（同節點設定）──────────────────────────────────
def model_file(stem, int8_set):
    p8 = os.path.join(a.asr_dir, f"{stem}-epoch-99-avg-1.int8.onnx")
    return p8 if (stem in int8_set and os.path.exists(p8)) else os.path.join(a.asr_dir, f"{stem}-epoch-99-avg-1.onnx")


def kw_for(int8_set):
    return dict(
        tokens=os.path.join(a.asr_dir, "tokens.txt"), encoder=model_file("encoder", int8_set),
        decoder=model_file("decoder", int8_set), joiner=model_file("joiner", int8_set), num_threads=a.threads,
        sample_rate=SR, feature_dim=80, decoding_method="modified_beam_search", max_active_paths=4,
        enable_endpoint_detection=True, rule1_min_trailing_silence=2.4, rule2_min_trailing_silence=1.2,
        rule3_min_utterance_length=20.0,
    )
clean = [ln.strip() for ln in open(a.hotwords, encoding="utf-8") if ln.strip() and not ln.strip().startswith("#")]
hw = tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False, encoding="utf-8")
hw.write("\n".join(clean) + "\n")
hw.close()
t0 = time.time()
recs = {}
for spec in a.configs:
    label, _, comps = spec.partition("=")
    int8_set = {c.strip() for c in comps.split(",") if c.strip()}
    kw = kw_for(int8_set)
    recs[(label, "無熱詞")] = sherpa_onnx.OnlineRecognizer.from_transducer(**kw)
    recs[(label, "熱詞")] = sherpa_onnx.OnlineRecognizer.from_transducer(hotwords_file=hw.name, hotwords_score=1.8, **kw)
print(f"ASR 載入 {time.time() - t0:.1f} s（{len(recs)} 個辨識器、{a.threads} 執行緒、熱詞 {len(clean)} 個）")


def recognize(rec, x):
    s = rec.create_stream()
    cpu, step = 0.0, 512
    for i in range(0, len(x), step):
        s.accept_waveform(SR, x[i: i + step])
        t = time.perf_counter()
        while rec.is_ready(s):
            rec.decode_stream(s)
        cpu += time.perf_counter() - t
    t = time.perf_counter()
    s.input_finished()
    while rec.is_ready(s):
        rec.decode_stream(s)
    fin = time.perf_counter() - t
    return rec.get_result(s), cpu + fin, fin


rows = []
speech_cache = {}
for sid in a.speakers:
    for k, (ref, _kind) in enumerate(SENTENCES):
        speech_cache[(sid, k)] = synth(ref, sid)
print(f"合成 {len(speech_cache)} 句完成")

for cut in a.cut_ms:
    for snr_s in a.snr:
        snr = float(snr_s)
        for sid in a.speakers:
            for k, (ref, _kind) in enumerate(SENTENCES):
                sp = speech_cache[(sid, k)][int(cut * SR / 1000):]
                x = mix(sp, snr)
                for (cfg, mode), rec in recs.items():
                    hyp, cpu, fin = recognize(rec, x)
                    r_s, h_s = norm_strict(ref), norm_strict(hyp)
                    dist, sub, dele, ins = ed_ops(norm_lenient(r_s), norm_lenient(h_s))
                    rows.append({
                        "精度": cfg, "句首砍ms": cut, "SNR": snr_s, "語者": sid, "句號": k + 1, "熱詞": mode, "參考": ref,
                        "辨識": hyp, "嚴格錯": ed_ops(r_s, h_s)[0], "寬鬆錯": dist, "取代": sub, "刪除": dele,
                        "插入": ins, "字數": len(r_s), "首字錯": int(not h_s or norm_lenient(h_s)[:1] != norm_lenient(r_s)[:1]),
                        "音長s": round(len(x) / SR, 2), "RTF": round(cpu / (len(x) / SR), 3), "收尾ms": round(fin * 1000, 1),
                    })
        print(f"  砍 {cut} ms、SNR {snr_s} 完成")

with open(os.path.join(a.out, "per_utt.csv"), "w", newline="", encoding="utf-8-sig") as fh:
    w = csv.DictWriter(fh, fieldnames=list(rows[0]))
    w.writeheader()
    w.writerows(rows)

temp = int(open("/sys/class/thermal/thermal_zone0/temp").read()) / 1000
lines = [f"ASR 離線基準 {time.strftime('%Y-%m-%d %H:%M')}：{len(a.speakers)} 位 TTS 語者 × {len(SENTENCES)} 句，"
         f"背景音 {os.path.basename(a.noise)}，{a.threads} 執行緒，結束時 CPU {temp:.0f}°C"]
for cut in a.cut_ms:
    for snr_s in a.snr:
        for cfg, mode in recs:
            sel = [r for r in rows if r["句首砍ms"] == cut and r["SNR"] == snr_s and r["熱詞"] == mode and r["精度"] == cfg]
            n = sum(r["字數"] for r in sel)
            hot = [r for r in sel if r["句號"] <= 5]
            nh = sum(r["字數"] for r in hot)
            lines.append(
                f"{cfg:<10} 砍{cut:>3}ms SNR {snr_s:>3} {mode:<3}：CER 寬鬆 {100 * sum(r['寬鬆錯'] for r in sel) / n:5.1f}%"
                f"（嚴格 {100 * sum(r['嚴格錯'] for r in sel) / n:5.1f}%；熱詞句 {100 * sum(r['寬鬆錯'] for r in hot) / nh:5.1f}%）"
                f" 首字錯 {sum(r['首字錯'] for r in sel)}/{len(sel)}"
                f" RTF 中位 {statistics.median(r['RTF'] for r in sel):.3f} 收尾中位 {statistics.median(r['收尾ms'] for r in sel):.0f} ms"
            )
open(os.path.join(a.out, "summary.txt"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
print("\n".join(lines))
print("輸出：", a.out)
