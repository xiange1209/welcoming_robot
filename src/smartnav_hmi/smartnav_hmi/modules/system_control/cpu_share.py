"""處理器分時（2026-11-24 新增）：依機器人當下在做什麼，自動調整各單元的處理器優先順序（nice）。

為什麼：單板把所有功能一起開時，四個核心沒有空檔（11/24 實測負載平均 50、閒置 0%）。
  11/24 12:06 那幾趟帶位：path_teach 的控制迴圈只跑到 0.1～6.3 Hz（設定 20 Hz），定位也跟著飄
  （AMCL 拿到的 TF 過期、/scan_slam 太久沒更新），車子一直被判定卡住、反覆脫困。
  最吃處理器的是人臉（face_embedding 約一顆核心）、相機＋JPEG 轉檔、語音辨識——
  它們在帶位途中都不急；反過來，待機迎賓時導航只要維持定位就好。

怎麼做：訂 /navigation_active（navigation_action_cc 發、latched；nav_camera.py 也訂同一個）。
  模式            底盤雷達 sensors  導航 nav  語音 asr_chain  相機 camera  人臉 face  對話／身分／迎賓  HMI 自己
  迎賓（平常）         -10             -5           -8            0           0             0           -5
  帶位中               -10            -10           +5          +19         +19            +5           -5
  - 數字是 nice：越小越優先（Linux -20～19；這裡最多用到 -10）。+19 的單元只吃別人用剩的處理器：
    平板的相機小窗照樣有畫面、只是可能比較慢（人臉推論在導航中本來就停，見 face_embedding 的 skip_while_navigating）。
  - 相機若是「避障模式」（深度點雲給導航用），帶位中改成 -5，不往後排。
  - 帶位結束後等 cpu_share_back_delay_sec 秒（車停穩、Nav2 收尾）才切回迎賓。
  - 單元的行程用 launch session 找：HMI（與 system_control CLI）用 start_new_session 開每個單元，
    同一單元底下所有節點的 session id 都是那個 launch 的 PID；session id 從狀態檔讀，不另外掃描整個 /proc 找 launch。
  - nice 是「每條執行緒」各自的屬性：/proc/<pid>/task 底下每個 TID 都要設；之後新開的執行緒會繼承，
    單元重開後的新行程每 cpu_share_rescan_sec 秒補一次。
  - launch 刻意設成正值的（map_service_cc、waypoint_service 是 +10）維持原值，不提高它們的優先權。

權限：讓 nice 變小（提高優先權）要「目標行程」的 RLIMIT_NICE 允許。HMI 服務的 drop-in
  （/etc/systemd/system/smartnav-hmi.service.d/20-asr-priority.conf：LimitNICE=-10）讓 HMI 與它開的單元都繼承這個上限。
  每個行程各自讀 /proc/<pid>/limits：能調到多低就調到多低；**連 0 都回不去的行程（上限 < 20）完全不碰**——
  否則把相機排到 +15 之後就回不來了。

關掉：ros2 param set /hmi_server_node cpu_time_sharing false（下一拍生效，改過的行程全部還原成原本的 nice）。
手動指定：ros2 param set /hmi_server_node cpu_share_force_mode nav（或 greet；設回空字串 "" ＝自動）。
"""

from __future__ import annotations

import os
import threading
import time
from typing import Dict, Iterable, List, Optional, Tuple

GREET, NAV = "greet", "nav"
# 各模式下各單元的目標 nice；沒列到的單元＝維持原值
PROFILES: Dict[str, Dict[str, int]] = {
    # ★ 11/24 16:43 實測：迎賓時只把語音調到 -10，錄影時大家在講話，語音辨識吃到 1.36 顆核心，
    #   雷射資料延遲 1～3 秒（/scan_slam 過期、全域定位「超過 1 秒沒有雷射」中止）→ 定位飄移、帶位失敗。
    #   底盤雷達本身很輕（合計不到半顆核心）但不能晚，平常也排最前面；定位（nav）次之；語音其次。
    GREET: {"sensors": -10, "nav": -5, "asr_chain": -8},
    # ★ 11/24 下午：上午用 -8／-5／+15 時，平板帶位控制迴圈 10～16 Hz 成功，但語音觸發的那幾趟仍不穩 →
    #   帶位中導航與底盤一起 -10，其他單元全部往後排（語音仍會聽，只是排在導航後面；平板急停走 HMI，不受影響）
    NAV: {"sensors": -10, "nav": -10, "asr_chain": 5, "llm": 5, "user_auth": 5, "bank_reception": 5,
          "camera": 19, "face": 19},
}
CAMERA_OBSTACLE_NAV = -5       # 避障模式的相機是導航的一部分
SELF_NICE = -5                 # HMI 自己：平板畫面、遙控與急停都經過它
NICE_FLOOR = -10               # 不比 drop-in 給的上限更激進
# ★ 2026-11-25：launch 檔自己會帶 nice 前綴的單元（導航的 map_service_cc、waypoint_service 是 +10）。
#   其他單元由 supervisor 保證從 0 開始，正的原值一律當 0：帶位中才啟動的單元，launch 先被排到 +19、之後生出來的
#   節點繼承 +19，把它記成「原值」就永遠停在 +19（相機、人臉、LLM 變慢，而且沒有任何錯誤訊息）。
#   負的是節點自己設的（語音辨識 -10），要留著，關掉分時才還原得回去。
OWN_NICE_UNITS = ("nav",)
NAV_ACTIVE_TOPIC = "/navigation_active"
MODE_TEXT = {GREET: "迎賓優先", NAV: "帶位優先", "off": "關閉"}


# ---------------------------------------------------------------------------
# 純函式（不依賴 ROS，可以單獨測試）
# ---------------------------------------------------------------------------
def read_stat(pid) -> Tuple[int, int, int]:
    """/proc/<pid>/stat → (session id, nice, starttime)。行程不存在丟 OSError。"""
    with open(f"/proc/{pid}/stat", "rb") as fh:
        data = fh.read().decode("ascii", "replace")
    rest = data[data.rfind(")") + 2:].split()      # rest[0] 是第 3 欄 state
    return int(rest[3]), int(rest[16]), int(rest[19])


def nice_floor_of(pid) -> Optional[int]:
    """這個行程最低能被調到多少 nice（由它自己的 RLIMIT_NICE 軟上限決定：nice ≥ 20 − 上限）。讀不到回 None。"""
    try:
        with open(f"/proc/{pid}/limits", encoding="ascii", errors="replace") as fh:
            for line in fh:
                if line.startswith("Max nice priority"):
                    soft = line.split()[3]
                    if soft == "unlimited":
                        return -20
                    return max(-20, 20 - int(soft))
    except (OSError, ValueError, IndexError):
        return None
    return None


def thread_ids(pid) -> List[int]:
    try:
        return [int(t) for t in os.listdir(f"/proc/{pid}/task") if t.isdigit()]
    except OSError:
        return []


def processes_by_session(sid_to_unit: Dict[int, str]) -> Dict[str, List[Tuple[int, int, int]]]:
    """掃一次 /proc：{單元: [(pid, nice, starttime), ...]}，只收 session id 屬於受管單元的行程。"""
    out: Dict[str, List[Tuple[int, int, int]]] = {}
    if not sid_to_unit:
        return out
    for entry in os.scandir("/proc"):
        if not entry.name.isdigit():
            continue
        try:
            sid, nice, start = read_stat(entry.name)
        except (OSError, ValueError, IndexError):
            continue
        unit = sid_to_unit.get(sid)
        if unit is not None:
            out.setdefault(unit, []).append((int(entry.name), nice, start))
    return out


def target_nice(unit: str, mode: str, baseline: int, floor: int, obstacle_camera: bool = False) -> int:
    """某單元的某個行程在某模式下應該是多少 nice。

    baseline：第一次看到它時的 nice（launch 刻意設成正值的維持原值）；floor：它的 RLIMIT_NICE 允許的最低值。
    """
    want = PROFILES.get(mode, {}).get(unit)
    if unit == "camera" and obstacle_camera and mode == NAV:
        want = CAMERA_OBSTACLE_NAV
    if want is None:
        return baseline
    if baseline > 0:
        want = max(want, baseline)
    return max(want, floor, NICE_FLOOR)


def apply_nice(pid: int, target: int) -> Tuple[int, int]:
    """把一個行程的每條執行緒設成 target；回 (改了幾條, 被拒絕幾條)。行程或執行緒中途消失就略過。"""
    changed = denied = 0
    for tid in thread_ids(pid):
        try:
            if os.getpriority(os.PRIO_PROCESS, tid) == target:
                continue
            os.setpriority(os.PRIO_PROCESS, tid, target)
            changed += 1
        except PermissionError:
            denied += 1
        except OSError:
            continue
    return changed, denied


class CpuShareCore:
    """不依賴 ROS 的核心：給它「各單元的 session 資訊」和模式，它負責把 nice 調對、記得原值、可以全部還原。"""

    def __init__(self, self_pid: Optional[int] = None):
        self._self_pid = self_pid if self_pid is not None else os.getpid()
        # (pid, starttime) → {"unit", "baseline", "floor"}；starttime 防 PID 重複使用
        self._seen: Dict[Tuple[int, int], dict] = {}
        self._self_baseline: Optional[int] = None

    def apply(self, mode: str, sessions: Dict[str, dict]) -> dict:
        """sessions：{單元: {"sid": int, "obstacle": bool}}。回統計（行程數、改了幾條執行緒、被拒絕幾條、跳過幾個）。"""
        sid_to_unit = {int(info["sid"]): unit for unit, info in sessions.items()}
        found = processes_by_session(sid_to_unit)
        stats = {"processes": 0, "changed": 0, "denied": 0, "skipped": 0}
        alive = set()
        for unit, procs in found.items():
            obstacle = bool(sessions.get(unit, {}).get("obstacle"))
            for pid, nice, start in procs:
                key = (pid, start)
                alive.add(key)
                rec = self._seen.get(key)
                if rec is None:
                    floor = nice_floor_of(pid)
                    rec = {"unit": unit, "baseline": nice if unit in OWN_NICE_UNITS else min(nice, 0), "floor": floor}
                    self._seen[key] = rec
                if rec["floor"] is None or rec["floor"] > 0:
                    stats["skipped"] += 1          # 讀不到上限、或連 0 都回不去：不碰
                    continue
                tgt = target_nice(unit, mode, rec["baseline"], rec["floor"], obstacle)
                c, d = apply_nice(pid, tgt)
                stats["processes"] += 1
                stats["changed"] += c
                stats["denied"] += d
        # 已經結束的行程不用再記
        for key in [k for k in self._seen if k not in alive]:
            del self._seen[key]
        c, d = self._apply_self(SELF_NICE)
        stats["changed"] += c
        stats["denied"] += d
        return stats

    def restore(self) -> dict:
        """全部還原成第一次看到時的 nice（關掉分時時用）。"""
        stats = {"processes": 0, "changed": 0, "denied": 0, "skipped": 0}
        for (pid, start), rec in list(self._seen.items()):
            try:
                if read_stat(pid)[2] != start:
                    continue                       # PID 已經換人
            except (OSError, ValueError, IndexError):
                continue
            if rec["floor"] is None or rec["floor"] > 0:
                continue
            c, d = apply_nice(pid, max(rec["baseline"], rec["floor"]))
            stats["processes"] += 1
            stats["changed"] += c
            stats["denied"] += d
        self._seen.clear()
        if self._self_baseline is not None:
            c, d = apply_nice(self._self_pid, self._self_baseline)
            stats["changed"] += c
            stats["denied"] += d
        return stats

    def _apply_self(self, nice: int) -> Tuple[int, int]:
        if self._self_baseline is None:
            try:
                self._self_baseline = read_stat(self._self_pid)[1]
            except (OSError, ValueError, IndexError):
                return 0, 0
        floor = nice_floor_of(self._self_pid)
        if floor is None or floor > 0:
            return 0, 0
        return apply_nice(self._self_pid, max(nice, floor, NICE_FLOOR))


# ---------------------------------------------------------------------------
# 接到 HMI（ROS）的部分
# ---------------------------------------------------------------------------
class CpuTimeSharing:
    def __init__(self, node, supervisor, units: Iterable[str]):
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Bool

        self._node = node
        self._sup = supervisor
        self._units = [u for u in units]
        self._core = CpuShareCore()
        node.declare_parameter("cpu_time_sharing", True)
        node.declare_parameter("cpu_share_back_delay_sec", 3.0)
        node.declare_parameter("cpu_share_rescan_sec", 5.0)
        # 手動指定模式（測試或展示用）：""＝自動（看 /navigation_active）、"greet"、"nav"
        node.declare_parameter("cpu_share_force_mode", "")
        self._lock = threading.Lock()
        self._nav = False
        self._nav_off_at = 0.0
        self._mode: Optional[str] = None          # 目前套用中的模式；None＝還沒套用或已關閉
        self._last_scan = 0.0
        self._warned = ""
        node.create_subscription(
            Bool, NAV_ACTIVE_TOPIC, self._on_nav,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        node.create_timer(0.5, self._tick)

    # ---- ROS 回呼 ----------------------------------------------------------
    def _on_nav(self, msg) -> None:
        with self._lock:
            nav = bool(msg.data)
            if nav != self._nav:
                self._nav = nav
                if not nav:
                    self._nav_off_at = time.monotonic()

    def _drop_dead_nav(self) -> None:
        """導航在「帶位中」掛掉時沒有人會再發 False，會一直停在帶位優先 → 發布端不見就當作沒在帶位

        做法比照 smartnav_vision 的 face_embedding_node._nav_watchdog。
        """
        with self._lock:
            if not self._nav:
                return
        try:
            if self._node.count_publishers(NAV_ACTIVE_TOPIC) > 0:
                return
        except Exception:  # noqa: BLE001 — 查不到就當作正常
            return
        with self._lock:
            self._nav = False
            self._nav_off_at = time.monotonic()
        self._node.get_logger().warning("處理器分時：/navigation_active 的發布端消失（導航節點掛了？）→ 回到迎賓分配")

    def _param(self, name: str, default):
        # ★ 每拍用 .value 重讀（「設了沒用的參數」與 .integer_value 讀成 0 的教訓）
        try:
            return self._node.get_parameter(name).value
        except Exception:  # noqa: BLE001
            return default

    def _tick(self) -> None:
        try:
            self._tick_inner()
        except Exception as exc:  # noqa: BLE001 — 分時只是加分項，任何錯誤都不能拖垮 HMI
            self._warn_once(f"處理器分時發生錯誤（不影響其他功能，下一拍重試）：{type(exc).__name__}: {exc}")

    def _tick_inner(self) -> None:
        enabled = bool(self._param("cpu_time_sharing", True))
        if not enabled:
            if self._mode is not None:
                st = self._core.restore()
                self._mode = None
                self._set_state("off")
                self._node.get_logger().info(f"處理器分時已關閉：還原 {st['processes']} 個行程、{st['changed']} 條執行緒")
            return
        self._drop_dead_nav()
        with self._lock:
            nav, off_at = self._nav, self._nav_off_at
        delay = max(0.0, float(self._param("cpu_share_back_delay_sec", 3.0)))
        want = NAV if nav or (self._mode == NAV and time.monotonic() - off_at < delay) else GREET
        forced = str(self._param("cpu_share_force_mode", "") or "").strip().lower()
        if forced in (GREET, NAV):
            want = forced
        rescan = max(1.0, float(self._param("cpu_share_rescan_sec", 5.0)))
        now = time.monotonic()
        if want == self._mode and now - self._last_scan < rescan:
            return
        st = self._core.apply(want, self._sessions())
        self._last_scan = now
        if want != self._mode:
            self._node.get_logger().info(
                f"處理器分時 → {MODE_TEXT[want]}：{self._describe(want)}"
                f"（{st['processes']} 個行程、調整 {st['changed']} 條執行緒）")
            self._mode = want
            self._set_state(want)
        if st["skipped"] and not st["processes"]:
            # 沒有 20-asr-priority.conf 時每個行程都「連 0 都回不去」，全部被略過、也沒有 denied → 以前完全沒有訊息
            self._warn_once(f"處理器分時沒有作用：{st['skipped']} 個行程都沒有權限調整優先權"
                            "（HMI 服務缺 LimitNICE=-10；把 smartnav_bringup/systemd/smartnav-hmi.service.d/"
                            "20-asr-priority.conf 裝到 /etc/systemd/system/smartnav-hmi.service.d/ 後重啟服務）")
        elif st["denied"]:
            self._warn_once(f"處理器分時：有 {st['denied']} 條執行緒沒有權限調高優先權"
                            "（HMI 服務需要 LimitNICE=-10，見 20-asr-priority.conf）；降低優先權的部分照樣生效")

    # ---- 小工具 ------------------------------------------------------------
    def _sessions(self) -> Dict[str, dict]:
        out = {}
        for unit, data in self._sup.managed_sessions(self._units).items():
            argv = data.get("argv") or []
            out[unit] = {"sid": int(data["sid"]), "obstacle": unit == "camera" and "variant:=obstacle" in argv}
        return out

    @staticmethod
    def _describe(mode: str) -> str:
        names = {"sensors": "底盤雷達", "nav": "導航", "asr_chain": "語音", "camera": "相機", "face": "人臉",
                 "llm": "對話", "user_auth": "身分", "bank_reception": "迎賓"}
        parts = [f"{names.get(u, u)} {v:+d}" for u, v in PROFILES[mode].items()]
        return "、".join(parts + [f"HMI {SELF_NICE:+d}"])

    def _set_state(self, mode: str) -> None:
        st = getattr(self._node, "state", None)
        if st is not None:
            try:
                st.set_system(cpu_share=mode)
            except Exception:  # noqa: BLE001
                pass

    def _warn_once(self, text: str) -> None:
        if text != self._warned:
            self._warned = text
            self._node.get_logger().warning(text)
