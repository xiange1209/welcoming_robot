#!/usr/bin/env python3
"""雙麥克風：把語音以不同到達時間差注入真實噪音，量 SNR 改善（2026-08-17）

★ 這支要回答的問題
    mic_snr_cc.py 只量得到「底噪被壓低多少」。但底噪壓低 15 dB
    **不代表 SNR 改善 15 dB** —— 如果 FIR 把人聲也一起壓低 15 dB，
    SNR 一點都沒變，只是整段變小聲，VAD 反而更難觸發。

    要分開量「人聲增益」與「噪音增益」，必須有一段乾淨人聲。
    我們沒有雙聲道人聲錄音（要有人站在車前講話才錄得到），
    所以改成：拿真實單聲道語音，用分數延遲合成出「從某個方向來」的
    雙聲道版本，疊到**真實**噪音上。噪音是真的，語音的到達方向是假的，
    而到達方向正是那個未知量 —— 所以就把它掃過一遍。

★ 為什麼可以分開量
    DualMicMixer 的五種模式全都是線性非時變的，所以
        y(noise + speech) = y(noise) + y(speech)
    人聲增益與噪音增益可以各自單獨餵、單獨量，
        ΔSNR = 20log10(|y_speech|/|L_speech|) - 20log10(|y_noise|/|L_noise|)
    這是恆等式，不是近似，也不受人聲音量大小影響。

★ 幾何（4.3 cm @ 16 kHz）
    lag 0     = broadside = 車頭正前方，人站的位置
    lag ±2.01 = endfire   = 車子正左／正右
    中間的角度對應中間的 lag：lag = 2.01 * sin(θ)

用法：
    mic_lag_sweep_cc.py --noise 安靜.wav [--speech 語音.wav]
"""
import argparse
import os
import sys
import wave

import numpy as np

_SRC = "/home/user/welcoming_robot_ws/src/smartnav_ws/src/smartnav_audio"
_INSTALL = "/home/user/welcoming_robot_ws/install/smartnav_audio/lib/python3.12/site-packages"
for _p in (_SRC, _INSTALL):
    if os.path.isdir(_p):
        sys.path.insert(0, _p)
from smartnav_audio.mic_array import DualMicMixer, fit_canceller  # noqa: E402

SR = 16000
MAX_LAG = 0.043 / 343.0 * SR  # 2.005 樣本


def load_wav(path):
    w = wave.open(path)
    sr, ch, n, sw = w.getframerate(), w.getnchannels(), w.getnframes(), w.getsampwidth()
    raw = w.readframes(n)
    w.close()
    if sw != 2:
        raise SystemExit(f"{path}: 只支援 S16_LE")
    d = np.frombuffer(raw, dtype=np.int16).astype(np.float64) / 32768.0
    if ch == 2:
        d = d.reshape(-1, 2)
    return d, sr


def dbfs(x):
    return 20 * np.log10(np.sqrt(np.mean(x ** 2)) + 1e-12)


def frac_delay(x, d):
    """把 x 延後 d 個樣本（d 可以是小數、可以是負的），用 FFT 相位平移。

    ★ 一定要用分數延遲。整數延遲只能表示 0/±1/±2 三個角度，
      而 4.3 cm 陣列的整個角度範圍就只有 ±2 個樣本 —— 用整數等於
      只看三個角度，會漏掉最危險的中間地帶。
    """
    n = len(x)
    nf = 1 << int(np.ceil(np.log2(n * 2)))
    xf = np.fft.rfft(x, nf)
    f = np.fft.rfftfreq(nf)
    xf = xf * np.exp(-2j * np.pi * f * d)
    return np.fft.irfft(xf, nf)[:n]


def run(mode, L, R, ctx):
    mixer = DualMicMixer(
        mode=mode, coeffs=ctx.get("fir"), lag=ctx.get("lag", 2), pre=ctx.get("pre", 4)
    )
    return mixer(np.stack([L, R], axis=1).astype(np.float32))


MODES = ["left", "average", "beamform", "cancel"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--noise", required=True, help="安靜（沒有人聲）的雙聲道錄音")
    ap.add_argument(
        "--speech",
        default="/home/user/models/sherpa-onnx-streaming-zipformer-bilingual-zh-en-2023-02-20/test_wavs/0.wav",
        help="乾淨的單聲道語音，用來合成不同到達方向",
    )
    ap.add_argument("--ntap", type=int, default=16)
    ap.add_argument("--pre", type=int, default=4)
    a = ap.parse_args()

    nz, sr = load_wav(a.noise)
    if nz.ndim != 2:
        raise SystemExit("--noise 必須是雙聲道")
    n = len(nz)
    half = n // 2
    # ★ 前半訓練、後半驗證。同一個檔切兩半只能證明「時間上穩定」，
    #   不能證明換場地也成立 —— 那要另外錄一段才算。
    trL, trR = nz[:half, 0], nz[:half, 1]
    hoL, hoR = nz[half:, 0], nz[half:, 1]

    w = fit_canceller(trL, trR, ntap=a.ntap, pre=a.pre)
    ctx = {"fir": w, "pre": a.pre, "lag": 2}

    print(f"噪音檔 {a.noise}  {n/sr:.1f}s  -> 訓練 {half/sr:.1f}s / 驗證 {(n-half)/sr:.1f}s")
    print(f"FIR {a.ntap} 抽頭 pre={a.pre}  係數最大絕對值 {np.abs(w).max():.2f}  總和 {w.sum():+.3f}")
    print(f"左聲道底噪 {dbfs(nz[:,0]):.1f} dBFS   右聲道底噪 {dbfs(nz[:,1]):.1f} dBFS "
          f"（差 {dbfs(nz[:,1])-dbfs(nz[:,0]):+.1f} dB）")

    # ── 1. 噪音增益（訓練段 vs 驗證段）──
    print(f"\n{'='*70}\n1. 底噪抑制（只有噪音，沒有人聲）\n{'='*70}")
    print(f"{'模式':<12} {'訓練段 dB':>12} {'驗證段 dB':>12}")
    print("-" * 40)
    noise_gain = {}
    for m in MODES:
        gtr = dbfs(run(m, trL, trR, ctx)) - dbfs(trL)
        gho = dbfs(run(m, hoL, hoR, ctx)) - dbfs(hoL)
        noise_gain[m] = gho
        print(f"{m:<12} {gtr:>+12.2f} {gho:>+12.2f}")
    print("※ 負值 = 噪音被壓低。這一欄單獨看沒有意義，要配下面的人聲增益。")

    # ── 2. 人聲增益 vs 到達時間差 ──
    sp, ssr = load_wav(a.speech)
    if sp.ndim == 2:
        sp = sp[:, 0]
    if ssr != sr:
        raise SystemExit(f"語音檔採樣率 {ssr} != {sr}")
    ln = min(len(sp), len(hoL))
    sp = sp[:ln]
    # 右聲道的靈敏度實測比左聲道低，語音也會吃到同一個差異
    g_r = np.sqrt(np.mean(nz[:, 1] ** 2)) / (np.sqrt(np.mean(nz[:, 0] ** 2)) + 1e-20)

    print(f"\n{'='*70}\n2. 人聲增益 vs 到達時間差（語音 {os.path.basename(a.speech)}，右聲道靈敏度比 {g_r:.3f}）\n{'='*70}")
    lags = [0.0, 0.25, 0.5, 1.0, 1.5, 2.0, -0.5, -1.0, -2.0]
    print(f"{'lag(樣本)':>10} {'角度':>8} " + " ".join(f"{m:>10}" for m in MODES))
    print("-" * 60)
    speech_gain = {m: {} for m in MODES}
    for d in lags:
        s_l = sp
        s_r = g_r * frac_delay(sp, d)
        ang = np.degrees(np.arcsin(np.clip(d / MAX_LAG, -1, 1)))
        row = []
        for m in MODES:
            g = dbfs(run(m, s_l, s_r, ctx)) - dbfs(s_l)
            speech_gain[m][d] = g
            row.append(f"{g:>+10.2f}")
        print(f"{d:>10.2f} {ang:>7.0f}° " + " ".join(row))
    print("※ 0 dB = 人聲原封不動。負很多 = 人聲也被消掉了。")

    # ── 3. ΔSNR ──
    print(f"\n{'='*70}\n3. ★ SNR 改善 = 人聲增益 - 噪音增益（相對 left）\n{'='*70}")
    print(f"{'lag(樣本)':>10} {'角度':>8} " + " ".join(f"{m:>10}" for m in MODES))
    print("-" * 60)
    for d in lags:
        ang = np.degrees(np.arcsin(np.clip(d / MAX_LAG, -1, 1)))
        base = speech_gain["left"][d] - noise_gain["left"]
        row = []
        for m in MODES:
            snr = speech_gain[m][d] - noise_gain[m]
            row.append(f"{snr - base:>+10.2f}")
        print(f"{d:>10.2f} {ang:>7.0f}° " + " ".join(row))
    print("※ 正值 = 比現況(left)好。人站在車頭正前方時 lag=0，看那一列。")


if __name__ == "__main__":
    main()
