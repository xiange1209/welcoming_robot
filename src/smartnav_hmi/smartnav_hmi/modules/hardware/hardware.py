"""周邊硬體偵測。

專案鐵則：**硬體問題用列舉指令回答，不要用文件回答**。
「驅動套件建置過」不等於「硬體在車上」。這支把那些列舉搬到平板上。

★ 兩層都要看，只看一層會得到錯的結論：
    核心層  裝置節點在不在（插了沒、驅動認得嗎）
    資料層  話題有沒有在發（認得到 ≠ 有資料，相機常態是關的）
  只看核心層 -> 相機插著但沒啟動，會誤報「正常」
  只看資料層 -> 節點沒開時全部紅字，分不出「沒開」還是「沒插」

★ 全部走 /proc 與 /sys，不開子行程：這支會被平板輪詢，
  每次 fork 一個 lsusb 在 Pi 4 上是不必要的負擔。
"""

import glob
import threading
import time
from typing import Callable, Dict


class HardwareProbe:
    """硬體狀態偵測 + 3 秒快取。

    sysinfo_provider 回傳 state 快照裡的 system 區塊——之所以用注入而不是
    直接拿節點，是為了讓這個類別完全不認識 ROS，可以單獨測試。
    """

    def __init__(self, sysinfo_provider: Callable[[], Dict]):
        self._sysinfo = sysinfo_provider
        self._lock = threading.Lock()
        self._cache = None
        self._cache_at = 0.0

    @staticmethod
    def _read(path: str) -> str:
        try:
            with open(path, "r", encoding="utf-8", errors="ignore") as fh:
                return fh.read()
        except OSError:
            return ""

    def _usb_products(self) -> list:
        """USB 裝置的產品名清單（相當於 lsusb，但不開子行程）"""
        names = []
        for f in glob.glob("/sys/bus/usb/devices/*/product"):
            v = self._read(f).strip()
            if v:
                names.append(v)
        return names

    def status(self) -> list:
        now = time.time()
        with self._lock:
            if self._cache and now - self._cache_at < 3.0:
                return self._cache

        pcm = self._read("/proc/asound/pcm")
        usb = self._usb_products()
        videos = sorted(glob.glob("/dev/video*"))
        serials = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
                         + glob.glob("/dev/wheeltec_*"))
        sysinfo = self._sysinfo()

        def item(key, label, present, detail, hint=""):
            return {"key": key, "label": label, "present": bool(present),
                    "detail": detail, "hint": hint}

        out = [
            item("camera", "深度相機 Astra S",
                 any("astra" in u.lower() or "orbbec" in u.lower() for u in usb) or bool(videos),
                 (f"{len(videos)} 個 /dev/video*" if videos else "找不到 /dev/video*")
                 + (f"；USB: {[u for u in usb if 'astra' in u.lower() or 'orbbec' in u.lower()]}"
                    if any('astra' in u.lower() or 'orbbec' in u.lower() for u in usb) else ""),
                 "沒抓到就把 Astra S 手動插拔一次 —— 它有開機列舉失敗的老問題，"
                 "而且**麥克風長在同一顆裝置上，會一起消失**"),
            item("mic", "麥克風（在相機裡）",
                 "capture" in pcm,
                 "有錄音裝置" if "capture" in pcm else "/proc/asound/pcm 沒有 capture",
                 "跟相機同一顆 USB 裝置。ASR 要用它"),
            item("speaker", "喇叭",
                 "playback" in pcm,
                 "有播放裝置" if "playback" in pcm else "沒有播放裝置（正常）",
                 "★ 車上刻意不裝喇叭（2026-08-07 決定）。語音輸出走平板瀏覽器，"
                 "所以這一項紅色是**預期的**，不用處理"),
            item("serial", "序列埠（底盤／光達）",
                 bool(serials),
                 "、".join(serials) if serials else "找不到任何 ttyUSB/ttyACM",
                 "底盤 STM32 與 N10 光達都走 USB 序列埠。少一個就會有一個節點起不來"),
        ]

        # ── 資料層：話題有沒有在發 ──
        # 這幾個量本來就在 state 裡（由既有訂閱更新），直接借用，不另外訂閱。
        out.append(item(
            "chassis_data", "底盤回報（電壓）",
            sysinfo.get("voltage") is not None,
            f"{sysinfo.get('voltage')} V" if sysinfo.get("voltage") is not None else "沒收到 /PowerVoltage",
            "★ 有序列埠但沒有電壓 = 線接上了但節點沒起來（或起來了但通訊失敗）"))

        with self._lock:
            self._cache = out
            self._cache_at = now
        return out
