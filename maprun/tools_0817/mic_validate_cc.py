#!/usr/bin/env python3
"""雙麥克風：跨場次驗證 + 決定要不要補回增益（2026-08-17）

★ 這支要回答三個問題
  1. 兩個聲道真的是兩顆實體麥克風嗎？ -> 互相關峰值要落在 lag≈-2，不是 0。
     （★ 不可以用相關係數 r 判斷。縮放複本的 r 也會很高，但峰值必在 lag 0。）
  2. **重開機前**訓練出來的係數，套到**重開機後**新錄的音上還有效嗎？
     這比「同一個檔切兩半」強：中間隔了一次完整重開機與 USB 重新列舉。
  3. cancel 把人聲也壓低了約 5 dB。要不要在輸出補一個固定增益把人聲拉回來，
     讓 VAD 看到的人聲位準跟現況一樣、但噪音低 9 dB？

用法：
    mic_validate_cc.py --train 舊安靜.wav --test 新安靜.wav [--coeffs 係數.txt]
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
SPEECH = "/home/user/models/sherpa-onnx-streaming-zipformer-bilingual-zh-en-2023-02-20/test_wavs/0.wav"


def load(path):
    w = wave.open(path)
    sr, ch, n = w.getframerate(), w.getnchannels(), w.getnframes()
    raw = w.readframes(n)
    w.close()
    d = np.frombuffer(raw, dtype=np.int16).astype(np.float64) / 32768.0
    if ch == 2:
        d = d.reshape(-1, 2)
    return d, sr


def dbfs(x):
    return 20 * np.log10(np.sqrt(np.mean(x ** 2)) + 1e-12)


def frac_delay(x, d):
    n = len(x)
    nf = 1 << int(np.ceil(np.log2(n * 2)))
    xf = np.fft.rfft(x, nf)
    f = np.fft.rfftfreq(nf)
    return np.fft.irfft(xf * np.exp(-2j * np.pi * f * d), nf)[:n]


def run(mode, L, R, w=None, lag=2, pre=4):
    m = DualMicMixer(mode=mode, coeffs=w, lag=lag, pre=pre)
    return m(np.stack([L, R], axis=1).astype(np.float32))


def xcorr_peak(L, R, span=8):
    """互相關峰值落在哪個 lag。兩顆實體麥克風 -> 非零；同一顆的縮放複本 -> 0。"""
    a = (L - L.mean()) / (L.std() + 1e-20)
    b = (R - R.mean()) / (R.std() + 1e-20)
    n = min(len(a), len(b))
    a, b = a[:n], b[:n]
    lags = np.arange(-span, span + 1)
    vals = []
    for k in lags:
        if k >= 0:
            v = np.dot(a[k:], b[:n - k]) / (n - k)
        else:
            v = np.dot(a[:n + k], b[-k:]) / (n + k)
        vals.append(v)
    vals = np.array(vals)
    return lags, vals


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--train", required=True)
    ap.add_argument("--test", required=True)
    ap.add_argument("--coeffs")
    ap.add_argument("--ntap", type=int, default=16)
    ap.add_argument("--pre", type=int, default=4)
    a = ap.parse_args()

    tr, _ = load(a.train)
    te, _ = load(a.test)
    print(f"訓練檔 {os.path.basename(a.train)} {len(tr)/SR:.1f}s   "
          f"測試檔 {os.path.basename(a.test)} {len(te)/SR:.1f}s")

    # ── 1. 是不是兩顆實體麥克風 ──
    print(f"\n{'='*70}\n1. 兩個聲道是不是兩顆實體麥克風\n{'='*70}")
    for name, d in (("重開機前", tr), ("重開機後", te)):
        lags, vals = xcorr_peak(d[:, 0], d[:, 1])
        pk = lags[np.argmax(vals)]
        r0 = vals[list(lags).index(0)]
        print(f"{name}: 互相關峰值在 lag={pk:+d}（{pk/SR*1000:+.3f} ms）  "
              f"峰值 r={vals.max():.4f}   lag0 的 r={r0:.4f}")
        print("      " + "  ".join(f"{l:+d}:{v:+.3f}" for l, v in zip(lags, vals) if abs(l) <= 4))
    print("※ 峰值不在 lag 0 -> 兩顆實體麥克風。若峰值在 0 才是同一顆的複本。")

    # ── 2. 跨場次泛化 ──
    print(f"\n{'='*70}\n2. 跨場次泛化（重開機前訓練 -> 重開機後測試）\n{'='*70}")
    if a.coeffs:
        w_old = np.loadtxt(a.coeffs, delimiter=",").astype(np.float32)
        print(f"讀入既有係數 {a.coeffs}（{len(w_old)} 抽頭，最大絕對值 {np.abs(w_old).max():.2f}）")
    else:
        w_old = fit_canceller(tr[:, 0], tr[:, 1], ntap=a.ntap, pre=a.pre)
    w_new = fit_canceller(te[:, 0], te[:, 1], ntap=a.ntap, pre=a.pre)

    print(f"\n{'':<34}{'重開機前的檔':>14}{'重開機後的檔':>14}")
    print("-" * 62)
    for label, w in (("舊係數（實際要上機的那組）", w_old), ("新係數（拿測試檔重訓，上界）", w_new)):
        g1 = dbfs(run("cancel", tr[:, 0], tr[:, 1], w)) - dbfs(tr[:, 0])
        g2 = dbfs(run("cancel", te[:, 0], te[:, 1], w)) - dbfs(te[:, 0])
        print(f"{label:<30}{g1:>+14.2f}{g2:>+14.2f}")
    print(f"\n左/右底噪：重開機前 {dbfs(tr[:,0]):.1f}/{dbfs(tr[:,1]):.1f} dBFS   "
          f"重開機後 {dbfs(te[:,0]):.1f}/{dbfs(te[:,1]):.1f} dBFS")

    # ── 3. 人聲增益 / 要補多少回來 ──
    sp, ssr = load(SPEECH)
    if sp.ndim == 2:
        sp = sp[:, 0]
    if ssr != SR:
        raise SystemExit(f"語音檔 {ssr} Hz")
    ln = min(len(sp), len(te))
    sp = sp[:ln]
    g_r = np.sqrt(np.mean(te[:, 1] ** 2)) / (np.sqrt(np.mean(te[:, 0] ** 2)) + 1e-20)

    print(f"\n{'='*70}\n3. 用舊係數：人聲/噪音/SNR（測試檔的噪音，合成的人聲方向）\n{'='*70}")
    ng = dbfs(run("cancel", te[:, 0], te[:, 1], w_old)) - dbfs(te[:, 0])
    print(f"噪音增益 {ng:+.2f} dB")
    print(f"\n{'lag':>6} {'角度':>7} {'人聲增益':>10} {'ΔSNR':>9} {'補回人聲需要':>13}")
    print("-" * 52)
    best = None
    for d in (0.0, 0.5, 1.0, 2.0, -0.5, -1.0, -2.0):
        s_l = sp
        s_r = g_r * frac_delay(sp, d)
        sg = dbfs(run("cancel", s_l, s_r, w_old)) - dbfs(s_l)
        ang = np.degrees(np.arcsin(np.clip(d / MAX_LAG, -1, 1)))
        print(f"{d:>6.2f} {ang:>6.0f}° {sg:>+10.2f} {sg-ng:>+9.2f} {-sg:>12.2f} dB")
        if d == 0.0:
            best = sg
    print(f"\n※ 人站正前方(lag 0)：人聲被壓 {-best:.2f} dB、噪音被壓 {-ng:.2f} dB "
          f"-> SNR 淨賺 {best-ng:+.2f} dB")
    print(f"※ 若在輸出補 {10**(-best/20):.2f} 倍（{-best:+.2f} dB）增益，"
          f"VAD 看到的人聲位準與現況相同，但噪音低 {-(ng-best):.2f} dB。")

    # 補回增益後會不會削波
    peak_te = np.abs(te[:, 0]).max()
    print(f"※ 削波檢查：測試檔左聲道峰值 {20*np.log10(peak_te+1e-12):.1f} dBFS，"
          f"補 {-best:.2f} dB 後 {20*np.log10(peak_te+1e-12)-best:.1f} dBFS")


if __name__ == "__main__":
    main()
