"""節點健康檢查與資源監看。

從 hmi_server_node.py 搬出來的兩塊：
  ・NodeHealthChecker.node_status()  ── /api/health 用的節點/lifecycle 狀態表
  ・NodeHealthChecker.resource_tick() ── 獨立計時器，CPU/記憶體/溫度告警

搬遷時邏輯逐行保留，只是把原本讀寫 self.xxx 的部分改成建構子注入。
這裡仍然吃 `node`（rclpy Node 本體）——node_status() 要動態建立/銷毀
lifecycle 的 get_state service client、讀節點圖、讀服務圖，這些本來
就是 rclpy Node 的方法，硬拆成窄介面只是把 node 的方法簽名重抄一遍。
"""

import os
import threading
import time
from typing import Any, Dict, List, Optional

from lifecycle_msgs.srv import GetState

from ...core.constants import INTERNAL_NODE_RE, NODE_GROUPS, OTHER_GROUP
from .procinfo import local_ros_processes


class NodeHealthChecker:
    # 資源警戒線。四核心 Pi 4：
    #   load 4 = 剛好滿載；8/14 那次量到 51，SSH 已經進不去
    #   記憶體 8 GB，留 600 MB 是「還救得回來」的下限
    LOAD_WARN = 6.0
    MEM_WARN_MB = 600
    TEMP_WARN = 78.0

    def __init__(self, node, state, pi_nodes: set, edge_nodes: set, callback_group):
        self._node = node
        self.state = state
        self.pi_nodes = pi_nodes
        self.edge_nodes = edge_nodes
        self._cb = callback_group
        # lifecycle 查詢用的 get_state 客戶端。只在一次查詢期間存在，
        # 查完就銷毀——留著會永久墊高執行器每一輪 wait set 的實體數。
        self._state_clients: Dict[str, Any] = {}
        self._nodes_cache: List[Dict[str, Any]] = []
        self._nodes_cache_at = 0.0
        self._nodes_lock = threading.Lock()
        self._cpu_stat_prev = None

    def _device_of(self, name: str, cmdlines: set, exes: set) -> str:
        """判斷節點跑在本機（pi）還是其他裝置（edge）

        判定順序：參數釘死 > 自己 > 內部節點不猜 > 本機行程比對 > 其餘視為遠端。
        """
        node = self._node
        if name in self.pi_nodes:
            return "pi"
        if name in self.edge_nodes:
            return "edge"
        if name == node.get_name():
            return "pi"
        if INTERNAL_NODE_RE.match(name):
            return "unknown"
        # launch 指定名稱時 cmdline 會有 __node:=<name>
        if any(name in c for c in cmdlines):
            return "pi"
        # 節點名常等於執行檔名或多／少一個 _node 後綴
        for exe in exes:
            if len(exe) < 5:
                continue
            if exe == name or name.startswith(exe) or exe.startswith(name):
                return "pi"
        return "edge"

    def node_status(self) -> List[Dict[str, Any]]:
        """列出圖上所有節點，lifecycle 節點另外回報生命週期狀態

        只看節點在不在會誤導：實測 bt_navigator 停在 inactive、amcl 停在
        unconfigured 時節點都「存在」，但 navigate_to_pose 根本沒有 server。
        所以凡是 lifecycle 節點都要看它是不是 active。

        哪些是 lifecycle 節點不寫死，而是看圖上有沒有對應的 get_state 服務——
        這樣 slam_toolbox 之類的也會自動被納入。

        這個函式會阻塞（要等服務回應），必須從執行緒池呼叫。
        """
        node = self._node
        now = time.time()
        with self._nodes_lock:
            if self._nodes_cache and now - self._nodes_cache_at < 2.0:
                return self._nodes_cache

        # bare name -> 完整路徑。預期清單用 bare name 比對，但顯示要用完整路徑，
        # 否則 /x10/lslidar_driver_node 會被寫成 lslidar_driver_node 而看不出在哪個命名空間。
        present: Dict[str, str] = {}
        try:
            for name, ns in node.get_node_names_and_namespaces():
                present[name] = name if ns in ("", "/") else f"{ns.rstrip('/')}/{name}"
        except Exception as e:
            node.get_logger().warn(f"讀取節點清單失敗: {e}", throttle_duration_sec=30.0)

        # 從服務清單反推哪些是 lifecycle 節點
        lifecycle: Dict[str, str] = {}
        try:
            for srv, types in node.get_service_names_and_types():
                if srv.endswith("/get_state") and any(t.endswith("GetState") for t in types):
                    lifecycle[srv[: -len("/get_state")].split("/")[-1]] = srv
        except Exception as e:
            node.get_logger().warn(f"讀取服務清單失敗: {e}", throttle_duration_sec=30.0)

        # 一次把所有請求送出去再一起等。逐一等待的話，十幾個節點各等 1 秒
        # 會讓這支端點慢到不能用。
        pending: Dict[str, Any] = {}
        for name, srv in lifecycle.items():
            if name not in present:
                continue
            client = self._state_clients.get(srv)
            if client is None:
                client = node.create_client(GetState, srv, callback_group=self._cb)
                self._state_clients[srv] = client
            if client.service_is_ready():
                pending[name] = client.call_async(GetState.Request())

        # ★ 量「經過多久」一律用 monotonic。本檔其餘的 time.time()
        #   是要送到瀏覽器顯示的**絕對**時間戳，那些正確、不要改。
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline and not all(f.done() for f in pending.values()):
            time.sleep(0.05)

        # 用完就拆掉。這些 get_state 客戶端原本是「建了就永久留著」，導航跑起來
        # 時 lifecycle 節點有十幾個，等於在執行器的 wait set 裡永久多出十幾個實體——
        # 而 rclpy 每一輪等待都要重新走過所有實體、逐一 enter 它們的 handle。
        # 健康頁是使用者按按鈕才刷新的，每次重建的成本遠低於一直掛著。
        # 用 popitem 而不是先 items() 再 clear()：兩個健康頁請求同時進來時
        # 每個客戶端只會被其中一邊取走一次，不會重複銷毀。
        while self._state_clients:
            try:
                _srv, client = self._state_clients.popitem()
                node.destroy_client(client)
            except Exception:  # 已被另一邊取走（KeyError）、或節點正在關閉
                pass

        cmdlines, exes = local_ros_processes()

        def row(name: str, group: str) -> Dict[str, Any]:
            label_name = present.get(name, name)
            # 沒在跑的節點談不上「在哪台機器」，另外歸類，不要塞進任一欄誤導
            device = "down" if name not in present else self._device_of(name, cmdlines, exes)
            base = {"name": label_name, "group": group, "device": device}
            if name not in present:
                return dict(base, state="未啟動", ok=False)
            if name not in lifecycle:
                return dict(base, state="執行中", ok=True)
            future = pending.get(name)
            if future is None or not future.done() or future.result() is None:
                return dict(base, state="無回應", ok=False)
            label = future.result().current_state.label
            return dict(base, state=label, ok=label == "active")

        rows: List[Dict[str, Any]] = []
        listed = set()
        for group, names in NODE_GROUPS:
            for name in names:
                rows.append(row(name, group))
                listed.add(name)

        # 沒列在預期清單裡、但確實在跑的節點（底盤、感測器、TF、launch 等）
        for name in sorted(set(present) - listed, key=lambda n: present[n]):
            rows.append(row(name, OTHER_GROUP))

        with self._nodes_lock:
            self._nodes_cache = rows
            self._nodes_cache_at = time.time()
        return rows

    def resource_tick(self) -> None:
        """獨立的資源監看（2026-08-14 新增）

        使用者要求 ASR 常駐等待，而常駐服務最怕的是**慢性資源漂移**：
        當下看起來都好，跑兩小時之後記憶體被吃光、SSH 進不去、只能斷電重開。
        8/14 就發生過一次（load average **51**、四核心、SSH 被擠掉）。

        ★ 三個量的分工不同，缺一不可：
          cpu_load   反應快，但**負載高不一定會死** —— 排隊而已
          mem_avail  ★ 這個才是致命的那個。可用記憶體歸零 = OOM killer
                     開始亂殺行程，或瘋狂 swap 導致整機失去回應
          cpu_temp   Pi 4 到 80°C 會降頻，症狀是「什麼都變慢」而不是壞掉

        ★ 為什麼讀 MemAvailable 而不是 MemFree：Linux 會把空閒記憶體拿去當
          快取，MemFree 常態就很低，看它會天天誤報。MemAvailable 是核心自己
          估的「不觸發 swap 能給出多少」，才是真正該看的數字。

        跨過門檻時**寫進 log**（不只是更新畫面）—— 事後查當機原因時，
        沒有人會有當時的平板畫面，但 log 一定留著。
        """
        logger = self._node.get_logger()
        fields: Dict[str, Any] = {}
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as fh:
                fields["cpu_temp"] = round(int(fh.read().strip()) / 1000.0, 1)
        except (OSError, ValueError):
            pass
        try:
            fields["cpu_load"] = round(os.getloadavg()[0], 2)
        except OSError:
            pass
        try:
            avail = total = None
            with open("/proc/meminfo", "r") as fh:
                for line in fh:
                    if line.startswith("MemAvailable:"):
                        avail = int(line.split()[1]) / 1024.0  # kB -> MB
                    elif line.startswith("MemTotal:"):
                        total = int(line.split()[1]) / 1024.0
                    if avail is not None and total is not None:
                        break
            if avail is not None:
                fields["mem_avail_mb"] = round(avail)
                if total:
                    fields["mem_used_pct"] = round(100.0 * (1.0 - avail / total))
        except (OSError, ValueError, IndexError):
            pass

        if fields:
            self.state.set_system(**fields)

        # ── 門檻警告 ────────────────────────────────────────
        # throttle_duration_sec 讓它最多每分鐘唸一次，不會洗版。
        # ★★ load average **不能**單獨拿來判斷「現在忙不忙」★★
        #
        # 實測：load average 顯示 36.6 的同一時刻，vmstat 的 r（真正可執行的
        # 執行緒數）只有 0~1、CPU 閒置 48~55%、IO 等待 0%。**系統根本沒在排隊。**
        # 原因是 load average 是 1/5/15 分鐘的指數加權移動平均，**它落後現實好幾分鐘**：
        # 當時反映的是稍早那段有四份 stuck_detector_cc、三份 scan_filter_cc
        # 殘留的狀況（stop_nav_cc.sh 收不乾淨造成，已修）。
        #
        # 照舊邏輯，操作者會被叫去「關掉深度相機或導航堆疊」——
        # 而真正該做的是把重複的節點收掉。**錯的警告比沒有警告更糟。**
        #
        # 改法：load 高**只是候選條件**，要再看一個即時指標才報。
        # 這裡用 /proc/stat 兩次取樣算出的 CPU 忙碌率（非 idle 佔比）。
        load = fields.get("cpu_load")
        if load is not None and load >= self.LOAD_WARN:
            busy = self._cpu_busy_ratio()
            if busy is None or busy >= 0.85:
                logger.warn(
                    f"⚠ 系統負載 {load:.1f}"
                    + (f"、CPU 忙碌 {busy*100:.0f}%" if busy is not None else "")
                    + "——考慮關掉深度相機或導航堆疊",
                    throttle_duration_sec=60.0,
                )
            else:
                # 這種情形通常代表「剛剛很忙、現在已經好了」，或有殘留節點被收掉了。
                logger.info(
                    f"系統負載 {load:.1f} 偏高，但 CPU 忙碌只有 {busy*100:.0f}%" "——是移動平均的殘影，現在沒有在排隊",
                    throttle_duration_sec=300.0,
                )
        mem = fields.get("mem_avail_mb")
        if mem is not None and mem <= self.MEM_WARN_MB:
            logger.error(
                f"⚠⚠ 可用記憶體只剩 {mem} MB（<{self.MEM_WARN_MB}）"
                "——再下去 OOM killer 會開始殺行程、SSH 會進不來，請立刻停掉非必要節點",
                throttle_duration_sec=60.0,
            )
        temp = fields.get("cpu_temp")
        if temp is not None and temp >= self.TEMP_WARN:
            logger.warn(
                f"⚠ CPU {temp:.1f}°C（>{self.TEMP_WARN:.0f} 會開始降頻，症狀是全部變慢）", throttle_duration_sec=60.0
            )

    def _cpu_busy_ratio(self) -> Optional[float]:
        """CPU 忙碌率（0~1）—— 兩次 /proc/stat 取樣的差值，是**即時**指標。

        ★ 跟 load average 的差別：load 是 1/5/15 分鐘的移動平均，會把幾分鐘前的
        尖峰一直帶著；這個看的是「上次呼叫到現在」這段區間真的用掉多少 CPU。
        第一次呼叫沒有前一筆可以比，回 None（呼叫端會退回只看 load）。
        """
        try:
            with open("/proc/stat") as f:
                parts = f.readline().split()
            vals = [int(x) for x in parts[1:11]]
            total = sum(vals)
            idle = vals[3] + vals[4]  # idle + iowait
        except Exception:
            return None
        prev = self._cpu_stat_prev
        self._cpu_stat_prev = (total, idle)
        if prev is None:
            return None
        dt, di = total - prev[0], idle - prev[1]
        if dt <= 0:
            return None
        return max(0.0, min(1.0, 1.0 - di / dt))
