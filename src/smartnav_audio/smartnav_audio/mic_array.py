#!/usr/bin/env python3
"""Astra S 雙麥克風處理

Astra S 上有兩顆實體麥克風（間距約 4.3 cm，左右各一個開孔）。
2026-08-17 之前的程式只開 channels=1，PortAudio 給的是左聲道
（實測與 ch0 相差 0.18 dB，與 ch1 相差 5.93 dB，確定是左聲道而不是混音）。

這個模組把 (frames, 2) 的雙聲道區塊轉成 (frames,) 單聲道，
給 VAD 與下游 ASR 用。下游完全不用改。

★ 幾何（4.3 cm 間距 @ 16 kHz）
    最大聲程差 = 0.043 / 343 = 125 us = 2.01 個樣本
    lag 0  = broadside = 車頭正前方（人站的位置）
    lag ±2 = endfire   = 車子左右側

    所以「把右聲道位移 2 個樣本再相加」指向的是**側面**，不是正前方；
    正前方對應的是 lag 0，也就是單純平均。這一點很容易搞反。

★ 模式
    left      只取左聲道。2026-08-17 之前的行為，預設值。
    right     只取右聲道（實測比左聲道低約 6 dB，只拿來對照）。
    average   兩聲道平均 = 指向正前方的延遲相加。
    beamform  延遲相加，指向 lag=mic_lag 的方向（預設 2 = 側面）。
    cancel    ★ FIR 噪音消除：用右聲道去預測左聲道裡的噪音再減掉。
              係數要先用 ~/maprun/tools_0817/mic_snr_cc.py --fit 在
              「安靜、沒有人講話」的雙聲道錄音上訓練出來。

★ 為什麼 cancel 會比 beamform 好
    4.3 cm 對 300-1000 Hz（實測底噪 99% 的能量都在這裡）來說是極小孔徑，
    延遲相加做不出指向性增益。但兩聲道的噪音相關係數高達 0.96，
    高相關正是「可預測、可相減」的條件 —— 小陣列做不出波束，做得出零點。
    實測 16 抽頭 FIR 把底噪壓低 13.6~14.3 dB，單抽頭只有 10.7 dB，
    average / beamform 只有 2.7 dB。

★ 底噪降 13.6 dB 不等於 SNR 改善 13.6 dB（2026-08-17 實測）
    cancel 會把人聲也一起壓低。因為所有模式都是線性非時變的，
    可以把人聲與噪音分開餵、分開量：
        ΔSNR = 人聲增益 - 噪音增益      （恆等式，不是近似）
    實測人站正前方（lag 0）時：人聲 -5.4 dB、噪音 -13.6 dB -> **淨賺 +8.2 dB**。
    整個 ±86 度範圍都是正的（+7.2 ~ +10.1 dB），沒有會變差的角度。
    對照組 average 只有 +0.14 dB、beamform -0.07 dB，等於沒用。

★ 為什麼要有 gain（補回增益）
    人聲被壓低 5.4 dB 這件事本身會傷到 VAD —— threshold 0.25 是在
    現況的人聲位準下校出來的，整體變小聲會讓它更難觸發，那就白做了。
    輸出補 x1.86 (+5.41 dB) 之後，VAD 看到的人聲位準與現況相同，
    噪音則低 8.2 dB，嚴格優於現況。

    gain 會套在**所有經過 mixer 的模式**上（right/average/beamform/cancel），
    只有兩個地方不套：mic_mode=left（節點根本不建 mixer，直接開單聲道），
    以及「裝置沒給到雙聲道」的退化輸入 —— 那兩種情況沒做任何消除，
    硬放大只會把底噪一起放大。
    ★ 所以要 A/B 比較 right 跟 left 的原始位準時，記得把 gain 設回 1.0。
"""

from typing import Optional, Sequence

import numpy as np

VALID_MODES = ("left", "right", "average", "beamform", "cancel")

# 4.3 cm @ 16 kHz 的 endfire 延遲
DEFAULT_LAG = 2
# FIR 允許往回看幾個樣本（見 fit_canceller 的說明）
DEFAULT_PRE = 4

# ★ 驗證過的 16 抽頭係數（2026-08-17）
#   訓練於 ~/maprun/tools_0817/rec/安靜基準_165544.wav（12 s，重開機前錄的）。
#   泛化驗證：套到**重開機後**新錄的 15 s 錄音上仍有 -13.62 dB，
#   而「拿那個新錄音重新訓練」的上界是 -13.68 dB —— 只差 0.06 dB。
#   中間隔了一次完整重開機與 USB 重新列舉，所以這是穩定的物理關係，不是過擬合。
#
#   ★ 什麼時候要重訓：換麥克風、改 amixer 增益（現在是 66 = 33 dB）、
#     或把相機換到別台車。重訓指令見 mic_snr_cc.py --fit。
DEFAULT_COEFFS = (
    3.907790, -3.590395, -0.085804, 0.889969,
    0.368317, 0.270470, 0.080703, 0.131420,
    0.200823, -0.087714, -0.075641, -0.047511,
    -0.424008, 0.551386, 2.593465, -2.615437,
)

# 補回被 cancel 壓掉的人聲位準：+5.41 dB。推導見上面的說明。
DEFAULT_GAIN = 1.86


class DualMicMixer:
    """把雙聲道區塊縮成單聲道，跨區塊保留必要的歷史樣本。

    可以直接當函式呼叫： mono = mixer(block)，block 形狀 (frames, 2)。
    """

    def __init__(
        self,
        mode: str = "left",
        coeffs: Optional[Sequence[float]] = None,
        lag: int = DEFAULT_LAG,
        pre: int = DEFAULT_PRE,
        gain: float = 1.0,
    ) -> None:
        if mode not in VALID_MODES:
            raise ValueError(f"未知的 mic_mode: {mode}，可用: {VALID_MODES}")
        self.mode = mode
        self.lag = int(lag)
        self.pre = int(pre)
        self.gain = float(gain)
        if not np.isfinite(self.gain) or self.gain <= 0:
            raise ValueError(f"mic_output_gain 必須是正的有限值，收到 {gain}")
        self.w: Optional[np.ndarray] = None

        if mode == "cancel":
            if coeffs is None or len(coeffs) == 0:
                raise ValueError("mic_mode=cancel 需要 FIR 係數（mic_cancel_coeffs）")
            self.w = np.asarray(coeffs, dtype=np.float32)
            if self.pre >= len(self.w):
                raise ValueError(f"pre({self.pre}) 必須小於抽頭數({len(self.w)})")

        # 需要保留多少歷史樣本才能跨區塊連續處理
        if mode == "cancel":
            self._hist = len(self.w) - 1
        elif mode == "beamform":
            self._hist = max(self.lag, 0)
        else:
            self._hist = 0

        self._prev_l: Optional[np.ndarray] = None
        self._prev_r: Optional[np.ndarray] = None

    # ── 內部：把上一塊的尾巴接到這一塊前面 ──
    def _with_history(self, left: np.ndarray, right: np.ndarray):
        if self._hist == 0:
            return left, right
        if self._prev_l is None:
            self._prev_l = np.zeros(self._hist, dtype=np.float32)
            self._prev_r = np.zeros(self._hist, dtype=np.float32)
        lc = np.concatenate((self._prev_l, left))
        rc = np.concatenate((self._prev_r, right))
        self._prev_l = lc[-self._hist:].copy()
        self._prev_r = rc[-self._hist:].copy()
        return lc, rc

    def reset(self) -> None:
        """清掉跨區塊歷史（重新開始錄音時呼叫）"""
        self._prev_l = None
        self._prev_r = None

    def __call__(self, block: np.ndarray) -> np.ndarray:
        """block: (frames, 2) -> (frames,)"""
        # ★ 這兩條是「裝置沒給到雙聲道」的退路。這種情況下沒有做任何消除，
        #   所以**不可以**套 gain —— 補回增益只是為了補償 cancel 壓掉的量，
        #   對原始訊號硬放大 5.4 dB 只會讓底噪跟著變大。
        if block.ndim == 1:
            return block.astype(np.float32, copy=False)
        if block.shape[1] < 2:
            return block[:, 0].astype(np.float32, copy=False)

        out = self._mix(block)
        if self.gain != 1.0:
            out = out * np.float32(self.gain)
        return out.astype(np.float32, copy=False)

    def _mix(self, block: np.ndarray) -> np.ndarray:
        left = block[:, 0].astype(np.float32, copy=False)
        right = block[:, 1].astype(np.float32, copy=False)
        n = len(left)

        if self.mode == "left":
            return left.copy()
        if self.mode == "right":
            return right.copy()
        if self.mode == "average":
            return ((left + right) * 0.5).astype(np.float32)

        lc, rc = self._with_history(left, right)

        if self.mode == "beamform":
            # y[m] = (L[m] + R[m-lag]) / 2
            cur_l = lc[self._hist:]
            start = self._hist - self.lag
            cur_r = rc[start:start + n]
            return ((cur_l + cur_r) * 0.5).astype(np.float32)

        # cancel: y[j] = L[pre+j] - sum_k w[k] * R[j+k]
        # 這個對齊方式的推導見 fit_canceller。輸出會比輸入延遲
        # (ntap-1-pre) 個樣本，16 抽頭時是 11 個樣本 = 0.7 ms，可以忽略。
        pred = np.correlate(rc, self.w, mode="valid")
        out = lc[self.pre:self.pre + n] - pred[:n]
        return out.astype(np.float32)


def fit_canceller(
    left: np.ndarray,
    right: np.ndarray,
    ntap: int = 16,
    pre: int = DEFAULT_PRE,
    ridge: float = 1e-4,
) -> np.ndarray:
    """在「安靜、沒有人講話」的雙聲道錄音上訓練 FIR 噪音消除係數。

    解最小平方： L[n] ≈ sum_k w[k] * R[n - pre + k]
    也就是拿右聲道在 n-pre .. n-pre+ntap-1 這段鄰域去預測左聲道。
    pre > 0 是因為右聲道可能比左聲道**早**收到聲音，濾波器要看得到「未來」。

    ★ 一定要用沒有人聲的片段訓練。拿有人聲的片段訓練，
      濾波器會學會把人聲也一起消掉。

    ★ ridge（脊迴歸）不是裝飾。底噪能量 99% 集中在 300-1000 Hz，
      自相關矩陣接近奇異，純最小平方解出來的係數會爆到 ±35，
      這種濾波器換個房間、增益漂一點就會失控放大。
      實測 λ=1e-4 把係數壓到 ±4，held-out 抑制量只從 15.07 掉到 14.83 dB，
      非常划算。
    """
    left = np.asarray(left, dtype=np.float64)
    right = np.asarray(right, dtype=np.float64)
    n = len(left) - ntap - pre
    if n <= ntap:
        raise ValueError("訓練資料太短")
    x = np.empty((n, ntap), dtype=np.float64)
    for k in range(ntap):
        x[:, k] = right[k:k + n]
    y = left[pre:pre + n]
    a = x.T @ x
    if ridge > 0:
        a = a + ridge * np.trace(a) / ntap * np.eye(ntap)
    w = np.linalg.solve(a, x.T @ y)
    return w.astype(np.float32)
