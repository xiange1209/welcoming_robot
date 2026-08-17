#!/usr/bin/env python3
"""雙麥克風演算法比較器（Astra S / 2026-08-17）

用途：對同一段雙聲道錄音，比較各種「兩聲道 -> 單聲道」的處理方式，
      量化每一種的 SNR（人聲區間 RMS - 安靜區間 RMS，dB）。

用法：
    # 一個檔案，自動切出人聲/安靜區間
    mic_snr_cc.py 錄音.wav

    # 兩個檔案：安靜基準 + 有人講話
    mic_snr_cc.py --quiet 安靜.wav --speech 講話.wav

    # 只看底噪抑制（沒有人聲時）
    mic_snr_cc.py --noise-only 安靜.wav

★ 錄音方式（卡號每次開機都會變，一定要現場抓）：
    CARD=$(arecord -l | grep -i 'ASTRA S' | sed -E 's/^card ([0-9]+):.*/\\1/')
    arecord -D hw:$CARD,0 -f S16_LE -r 16000 -c 2 -d 10 out.wav
  錄之前要先 ~/maprun/run_asr_cc.sh stop，否則裝置被 voice_trigger 佔住。
"""
import argparse
import os
import sys
import wave

import numpy as np

# ★ 直接用 smartnav_audio 裡的實作，不要在這裡另外寫一份。
#   這支工具量到的東西必須跟節點實際跑的是同一段程式碼，
#   否則「離線量到有改善、上機沒改善」會查很久。
_SRC = "/home/user/welcoming_robot_ws/src/smartnav_ws/src/smartnav_audio"
_INSTALL = "/home/user/welcoming_robot_ws/install/smartnav_audio/lib/python3.12/site-packages"
for _p in (_SRC, _INSTALL):
    if os.path.isdir(_p):
        sys.path.insert(0, _p)
from smartnav_audio.mic_array import DualMicMixer, fit_canceller  # noqa: E402

SR_EXPECT = 16000
# 4.3 cm 間距 @16 kHz -> 最大聲程差 2.01 樣本。
# lag 0  = broadside = 車頭正前方（人站的位置）
# lag ±2 = endfire   = 車子左右側（實測底噪的來向）
NOISE_LAG = 2


# ────────────────────── 基本工具 ──────────────────────
def load_stereo(path):
    w = wave.open(path)
    sr, ch, n, sw = w.getframerate(), w.getnchannels(), w.getnframes(), w.getsampwidth()
    raw = w.readframes(n)
    w.close()
    if sw != 2:
        raise SystemExit(f"{path}: 只支援 S16_LE")
    d = np.frombuffer(raw, dtype=np.int16).astype(np.float64) / 32768.0
    if ch != 2:
        raise SystemExit(f"{path}: 需要雙聲道，這個檔是 {ch} 聲道")
    if sr != SR_EXPECT:
        print(f"⚠ {path} 採樣率 {sr} != {SR_EXPECT}", file=sys.stderr)
    return d.reshape(-1, 2), sr


def dbfs(x):
    return 20 * np.log10(np.sqrt(np.mean(x ** 2)) + 1e-12)


def delay(x, k):
    """把 x 延後 k 個樣本（k>0 = 往後移，前面補零）"""
    if k == 0:
        return x.copy()
    out = np.zeros_like(x)
    if k > 0:
        out[k:] = x[:-k]
    else:
        out[:k] = x[-k:]
    return out


# ────────────────────── 各種演算法 ──────────────────────
# 每個都吃 (L, R) 回傳單聲道。ls_gain 由安靜段訓練出來，供 null 用。
def algo_left(L, R, ctx):
    return L.copy()


def algo_right(L, R, ctx):
    return R.copy()


def algo_average(L, R, ctx):
    """單純兩聲道平均（= 指向 broadside/車頭正前方的延遲相加，因為正前方 lag=0）"""
    return (L + R) / 2.0


def algo_avg_gainmatched(L, R, ctx):
    """先把右聲道補到和左聲道同音量再平均（右聲道實測低 6 dB）"""
    g = ctx["gain_match"]
    return (L + g * R) / 2.0


def algo_ds_endfire(L, R, ctx):
    """延遲相加、指向 endfire（車子側面）：把右聲道延後 2 樣本再相加。
    ★ 這是任務書原本設想的做法，但幾何上它指的是側面而非車頭。"""
    return (L + delay(R, NOISE_LAG)) / 2.0


def algo_ds_endfire_neg(L, R, ctx):
    """延遲相加、指向另一側 endfire：把左聲道延後 2 樣本"""
    return (delay(L, NOISE_LAG) + R) / 2.0


def algo_null_1tap(L, R, ctx):
    """單抽頭零點：y = L - g*delay(R,2)。拿來跟多抽頭 FIR 對照。"""
    g = ctx["null_gain"]
    return L - g * delay(R, NOISE_LAG)


def algo_null_broadside_ref(L, R, ctx):
    """對照組：在 lag 0 打零點。正前方的人聲也會被一起消掉，
    拿來確認零點真的是「打在噪音方向」而不是隨便減都好。"""
    g = ctx["null0_gain"]
    return L - g * R


def _run_mixer(mode, L, R, ctx):
    """走節點實際會用的 DualMicMixer，一次餵完整段"""
    mixer = DualMicMixer(mode=mode, coeffs=ctx.get("fir"), lag=NOISE_LAG, pre=ctx.get("pre", 4))
    block = np.stack([L, R], axis=1).astype(np.float32)
    return mixer(block)


def algo_cancel_fir(L, R, ctx):
    """★ 多抽頭 FIR 噪音消除（節點的 mic_mode=cancel）"""
    return _run_mixer("cancel", L, R, ctx)


ALGOS = [
    ("left（現況）", algo_left),
    ("right", algo_right),
    ("average 平均", algo_average),
    ("average 增益匹配", algo_avg_gainmatched),
    ("delay-sum 指向側面(+2)", algo_ds_endfire),
    ("delay-sum 指向另側(-2)", algo_ds_endfire_neg),
    ("null 單抽頭(+2)", algo_null_1tap),
    ("null 消 lag0(對照)", algo_null_broadside_ref),
    ("★ cancel FIR 16抽頭", algo_cancel_fir),
]


def build_ctx(qL, qR, ntap=16, pre=4):
    """用**安靜段**算出各演算法需要的係數

    ★ 一定要用沒有人聲的片段。用有人聲的片段訓練，FIR 會學會把人聲也消掉，
      然後你會量到一個漂亮但假的 SNR。
    """
    ctx = {"pre": pre}
    rms_l = np.sqrt(np.mean(qL ** 2))
    rms_r = np.sqrt(np.mean(qR ** 2))
    ctx["gain_match"] = rms_l / (rms_r + 1e-12)
    d2 = delay(qR, NOISE_LAG)
    ctx["null_gain"] = float(np.dot(qL, d2) / (np.dot(d2, d2) + 1e-20))
    ctx["null0_gain"] = float(np.dot(qL, qR) / (np.dot(qR, qR) + 1e-20))
    ctx["fir"] = fit_canceller(qL, qR, ntap=ntap, pre=pre)
    return ctx


# ────────────────────── 區間切分 ──────────────────────
def split_speech_quiet(L, sr, frame=0.05):
    """用能量分位數自動切出人聲段與安靜段。
    回傳 (人聲遮罩, 安靜遮罩)；若起伏太小視為沒有人聲。"""
    n = int(sr * frame)
    nf = len(L) // n
    e = np.array([np.mean(L[i * n:(i + 1) * n] ** 2) for i in range(nf)])
    edb = 10 * np.log10(e + 1e-20)
    lo, hi = np.percentile(edb, 20), np.percentile(edb, 95)
    spread = hi - lo
    quiet_th = np.percentile(edb, 30)
    speech_th = max(quiet_th + 6.0, np.percentile(edb, 80))
    sm = np.zeros(len(L), dtype=bool)
    qm = np.zeros(len(L), dtype=bool)
    for i in range(nf):
        sl = slice(i * n, (i + 1) * n)
        if edb[i] >= speech_th:
            sm[sl] = True
        elif edb[i] <= quiet_th:
            qm[sl] = True
    return sm, qm, spread


# ────────────────────── 主流程 ──────────────────────
def report(ctx, seg_speech, seg_quiet, title):
    print(f"\n{'='*74}\n{title}\n{'='*74}")
    print(f"{'演算法':<26} {'人聲 dBFS':>10} {'安靜 dBFS':>10} {'SNR dB':>8} {'相對 left':>10}")
    print("-" * 74)
    base = None
    rows = []
    for name, fn in ALGOS:
        sp = fn(seg_speech[0], seg_speech[1], ctx) if seg_speech else None
        qt = fn(seg_quiet[0], seg_quiet[1], ctx)
        q_db = dbfs(qt)
        if sp is not None:
            s_db = dbfs(sp)
            snr = s_db - q_db
        else:
            s_db, snr = float("nan"), float("nan")
        if base is None:
            base = snr
        delta = snr - base
        rows.append((name, s_db, q_db, snr, delta))
        print(f"{name:<26} {s_db:>10.1f} {q_db:>10.1f} {snr:>8.2f} {delta:>+9.2f}")
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("wav", nargs="?")
    ap.add_argument("--quiet")
    ap.add_argument("--speech")
    ap.add_argument("--noise-only")
    ap.add_argument("--fit", metavar="安靜.wav", help="訓練 FIR 係數並印出 ros2 參數字串")
    ap.add_argument("--ntap", type=int, default=16)
    a = ap.parse_args()

    if a.fit:
        d, sr = load_stereo(a.fit)
        L, R = d[:, 0], d[:, 1]
        sm, qm, spread = split_speech_quiet(L, sr)
        if spread >= 6:
            print(f"⚠ 這個檔的能量起伏有 {spread:.1f} dB，可能含人聲。"
                  f"FIR 一定要用**沒有人聲**的錄音訓練，否則會把人聲一起消掉。", file=sys.stderr)
        w = fit_canceller(L, R, ntap=a.ntap)
        mixer = DualMicMixer(mode="cancel", coeffs=w)
        out = mixer(np.stack([L, R], axis=1).astype(np.float32))
        red = dbfs(out) - dbfs(L)
        print(f"# 訓練於 {a.fit}（{len(L)/sr:.1f}s），{a.ntap} 抽頭")
        print(f"# 這段錄音上的底噪抑制：{red:+.2f} dB   係數最大絕對值 {np.abs(w).max():.2f}")
        print(f"# 係數最大絕對值若超過 ~10，代表擬合病態，換一段更長的安靜錄音重訓")
        print("\n# 直接貼給 ros2 run：")
        arr = ",".join(f"{v:.6f}" for v in w)
        print(f"  -p mic_mode:=cancel -p 'mic_cancel_coeffs:=[{arr}]'")
        return

    if a.noise_only:
        d, sr = load_stereo(a.noise_only)
        L, R = d[:, 0], d[:, 1]
        ctx = build_ctx(L, R)
        print(f"檔案 {a.noise_only}  {len(L)/sr:.1f}s")
        print(f"增益匹配係數 g={ctx['gain_match']:.3f}  null(+2) g={ctx['null_gain']:.3f}  null(0) g={ctx['null0_gain']:.3f}")
        print(f"\n{'演算法':<26} {'底噪 dBFS':>10} {'相對 left':>10}")
        print("-" * 50)
        base = dbfs(L)
        for name, fn in ALGOS:
            v = dbfs(fn(L, R, ctx))
            print(f"{name:<26} {v:>10.1f} {v-base:>+9.2f}")
        print("\n※ 這裡只是底噪變化，不是 SNR。噪音降多少要跟人聲降多少一起看才有意義。")
        return

    if a.quiet and a.speech:
        dq, sr = load_stereo(a.quiet)
        ds, _ = load_stereo(a.speech)
        ctx = build_ctx(dq[:, 0], dq[:, 1])
        # 講話檔裡也要挑出真正有人聲的片段
        sm, qm, spread = split_speech_quiet(ds[:, 0], sr)
        if spread < 4:
            print(f"⚠ 講話檔的能量起伏只有 {spread:.1f} dB，可能整段都沒有人聲", file=sys.stderr)
        sp = (ds[sm, 0], ds[sm, 1])
        qt = (dq[:, 0], dq[:, 1])
        print(f"安靜檔 {a.quiet} {len(dq)/sr:.1f}s ／ 講話檔 {a.speech} {len(ds)/sr:.1f}s "
              f"(人聲區間 {sm.sum()/sr:.1f}s)")
        print(f"增益匹配 g={ctx['gain_match']:.3f}  null(+2) g={ctx['null_gain']:.3f}  null(0) g={ctx['null0_gain']:.3f}")
        report(ctx, sp, qt, "SNR 比較（人聲區間 vs 獨立安靜檔）")
        return

    if a.wav:
        d, sr = load_stereo(a.wav)
        L, R = d[:, 0], d[:, 1]
        sm, qm, spread = split_speech_quiet(L, sr)
        print(f"檔案 {a.wav} {len(L)/sr:.1f}s  能量起伏 {spread:.1f} dB  "
              f"人聲 {sm.sum()/sr:.1f}s / 安靜 {qm.sum()/sr:.1f}s")
        if spread < 4:
            print("⚠ 能量起伏太小，這段錄音裡大概沒有人聲；改用 --noise-only 看底噪。")
            return
        ctx = build_ctx(L[qm], R[qm])
        report(ctx, (L[sm], R[sm]), (L[qm], R[qm]), "SNR 比較（同一檔內人聲 vs 安靜區間）")
        return

    ap.print_help()


if __name__ == "__main__":
    main()
