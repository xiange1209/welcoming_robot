"""收音距離閘門：車頭前方一定距離內有人，語音才會觸發（★ 2026-11-24）。

使用者：「語音輸入太敏感了 我希望前面1公尺才收音」。11/24 錄影時旁人說話每分鐘誤觸發 3.6～4.8 次，
LLM 忙著回旁人的話，客人真正的問題就被「上一題還在處理」略過（17:00 後 41 句）。

做法：用雷射（/scan，雷射高度約 11 cm＝小腿）看「車頭外緣前方 distance_m、左右 half_width_m」
的長方形裡有沒有東西。用雷射而不是人臉：人臉在帶位中會暫停、客人低頭看平板也認不到；
用長方形而不是單純距離：車身自己的結構（鋁擠條）在車頭外緣以內，不會被算成人。

開門的情況（任一成立）：
  - 前方長方形裡有 ≥ min_points 個點（hold_sec 內有看到就算，講話時退半步不會被切掉）；
  - 正在帶位（客人跟在車後，要能喊「停下來」）；
  - 雷射資料 stale_sec 沒更新（感測器沒開）→ 跟以前一樣不限距離，不能讓語音整個失靈。

★ 已知限制：車頭正對牆或櫃檯（1 公尺內）時門會一直開著，等於沒有限制 —— 待命位置要朝向空曠處。
"""

import math
import time
from typing import Callable, Optional, Sequence, Tuple

import numpy as np


class PresenceGate:
    def __init__(self, distance_m: float = 1.0, front_offset_m: float = 0.31, half_width_m: float = 0.6,
                 min_points: int = 3, hold_sec: float = 3.0, stale_sec: float = 2.0,
                 clock: Callable[[], float] = time.monotonic):
        self.distance_m = float(distance_m)
        self.front_offset_m = float(front_offset_m)     # 雷射到車頭外緣（沿車頭方向）
        self.half_width_m = float(half_width_m)
        self.min_points = int(min_points)
        self.hold_sec = float(hold_sec)
        self.stale_sec = float(stale_sec)
        self._clock = clock
        self._present_at = -math.inf
        self._scan_at = -math.inf
        self._navigating = False
        self._geom_key: Optional[Tuple[float, float, int]] = None
        self._cos = self._sin = None
        self.nearest_m: Optional[float] = None          # 最近一次掃描，長方形裡最近的點離車頭多遠（除錯用）

    def set_navigating(self, flag: bool) -> None:
        self._navigating = bool(flag)

    def on_scan(self, angle_min: float, angle_increment: float, ranges: Sequence[float],
                range_min: float = 0.0, range_max: float = math.inf) -> bool:
        """吃一次雷射掃描；回傳這一次前方長方形裡有沒有人"""
        r = np.asarray(ranges, dtype=np.float32)
        key = (float(angle_min), float(angle_increment), int(r.size))
        if key != self._geom_key:                       # 角度表只在掃描格式變的時候重算
            a = angle_min + angle_increment * np.arange(r.size, dtype=np.float64)
            self._cos, self._sin = np.cos(a).astype(np.float32), np.sin(a).astype(np.float32)
            self._geom_key = key
        ok = np.isfinite(r) & (r > max(range_min, 0.01)) & (r < range_max)
        r = np.where(ok, r, 0.0)                         # inf／nan 先換掉，免得 numpy 每圈印警告洗 log
        x, y = r * self._cos, r * self._sin
        front = x - self.front_offset_m                 # 離車頭外緣多遠（負的＝在車身範圍內）
        inside = ok & (front > 0.02) & (front <= self.distance_m) & (np.abs(y) <= self.half_width_m)
        now = self._clock()
        self._scan_at = now
        count = int(np.count_nonzero(inside))
        self.nearest_m = float(front[inside].min()) if count else None
        if count >= self.min_points:
            self._present_at = now
            return True
        return False

    def is_open(self) -> Tuple[bool, str]:
        now = self._clock()
        if self._navigating:
            return True, "帶位中（不限距離）"
        if now - self._scan_at > self.stale_sec:
            return True, "沒有雷射資料（不限距離）"
        if now - self._present_at <= self.hold_sec:
            return True, "前方有人"
        return False, f"前方 {self.distance_m:.1f} m 內沒人"
