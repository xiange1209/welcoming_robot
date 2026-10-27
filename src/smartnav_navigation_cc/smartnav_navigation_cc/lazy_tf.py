"""需要時才掛 TF 監聽、一段時間沒人用就拆掉（★ 2026-10-23，單板跑全功能的 CPU 優化）。

量測（maprun/tools_1020/tf_bench/tf_bench.py，Pi 4、/tf 約 70 則/秒）：Python 節點**光是掛著**
TransformListener、什麼都不查，CPU 就從 0.8% 變 17.1%（單核）——rclpy 要在 Python 裡反序列化每一則 /tf。
這個套件的 map_service、waypoint_service、path_teach 三個節點整天掛著，但大部分時間根本不查
（10/20 全功能同開時 path_teach 閒置也吃 25~38%、map_service 18%，CPU 閒置是 0%）。
同一個做法 HMI 的 imaging/pose.py 早就在用（_acquire_tf_listener／_drop_tf_listener）。

用法：
    self._tf = LazyTf(self, idle_sec=30.0)
    buf = self._tf.buffer(wait_for=("map", "base_footprint"))   # 確保監聽存在；剛接上時最多等 warmup_sec
    buf = self._tf.peek()          # 監聽已經存在才回 Buffer，不會把它叫醒、也不延長壽命（給定期 timer 用）

★ 每次都要重新呼叫 buffer()／peek() 取 Buffer，**不要把 Buffer 存起來用**：監聽拆掉之後舊的 Buffer
  不會再更新，lookup_transform(Time()) 會回「最後一筆」而不是報錯——拿到的是過時的位姿。
★ 節點要用 MultiThreadedExecutor（三個節點都是）：tf2_ros 的訂閱在可重入的 callback group，
  buffer(wait_for=…) 在別的 callback 裡等它進資料不會鎖死。
"""

import threading
import time
from typing import Optional, Tuple

from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

# ★ 2026-10-24：/tf_static 一定要 TRANSIENT_LOCAL（tf2 的標準：深度 100）。
#   舊寫法 static_qos=1 是「深度 1、VOLATILE」——**比監聽更早發的靜態轉換永遠收不到**。
#   以前監聽在節點啟動時就接上，只漏掉比節點更早起來的靜態發布者；改成按需接上之後幾乎一定漏掉。
#   實車目前沒踩到（map->odom 是 AMCL、odom->base_footprint 是 EKF，都是動態 /tf），
#   但只要那條鏈上有一段是靜態轉換（例如離線規劃基準的 map->odom），三個節點就永遠拿不到位姿。
#   （10/24 端到端測試時踩到：「取不到車子目前位置」。）靜態訊息只在接上時收一次，沒有 CPU 成本。
#   ★ 在建構時才建 QoSProfile，不要放模組層級：筆電端的 tools_1021/test_*.py 用假的 rclpy 模組 import 節點，
#     假模組的 DurabilityPolicy 沒有 TRANSIENT_LOCAL，放模組層級 import 當下就炸（10/24 test_nav_replan 踩過）。
def static_tf_qos() -> QoSProfile:
    return QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE)


class LazyTf:
    # ★ 2026-10-27：warmup 1.0 -> 3.0 秒。上機當天 demo 全開、CPU 滿載，剛接上的監聽 1 秒內收不到 map->base_footprint，
    #   map_service 的「導航前位姿對齊」3/3 次回「讀不到目前位姿」（10/20～21 沒有 LazyTf 時每次都成功）。
    def __init__(self, node, idle_sec: float = 30.0, warmup_sec: float = 3.0, qos=20, static_qos=None,
                 cache_sec: float = 10.0):
        self._node = node
        self.idle_sec = float(idle_sec)
        self.warmup_sec = float(warmup_sec)
        if not isinstance(static_qos, QoSProfile):     # 沒給或給整數（舊寫法）一律用標準的靜態 QoS
            static_qos = static_tf_qos()
        self._qos, self._static_qos, self._cache_sec = qos, static_qos, cache_sec
        self._buffer: Optional[Buffer] = None
        self._listener: Optional[TransformListener] = None
        self._created = 0.0
        self._last_use = 0.0
        self._lock = threading.Lock()
        self.activations = 0
        self.disabled = self.idle_sec <= 0.0  # idle_sec <= 0：一直掛著（等同舊行為，除錯用）
        if self.disabled:
            self.buffer()
        else:
            node.create_timer(max(1.0, min(5.0, self.idle_sec / 3.0)), self._reap)

    def active(self) -> bool:
        with self._lock:
            return self._listener is not None

    def peek(self) -> Optional[Buffer]:
        with self._lock:
            return self._buffer if self._listener is not None else None

    def buffer(self, wait_for: Optional[Tuple[str, str]] = None) -> Buffer:
        with self._lock:
            self._last_use = time.monotonic()
            if self._listener is None:
                self._buffer = Buffer(cache_time=Duration(seconds=self._cache_sec))
                self._listener = TransformListener(self._buffer, self._node, qos=self._qos,
                                                   static_qos=self._static_qos)
                self._created = time.monotonic()
                self.activations += 1
                self._node.get_logger().info(
                    f"TF 監聽接上（需要時才開；閒置 {self.idle_sec:.0f} 秒自動拆掉以省 CPU）")
            buf, created = self._buffer, self._created
        if wait_for is not None:
            deadline = created + self.warmup_sec
            waited = False
            while time.monotonic() < deadline and not buf.can_transform(wait_for[0], wait_for[1], Time()):
                waited = True
                time.sleep(0.02)
            if waited and not buf.can_transform(wait_for[0], wait_for[1], Time()):
                self._node.get_logger().warn(
                    f"TF 監聽接上 {self.warmup_sec:.0f} 秒內還等不到 {wait_for[0]}->{wait_for[1]}（CPU 吃緊或定位沒起來？）")
        return buf

    def _reap(self) -> None:
        with self._lock:
            if self.disabled or self._listener is None or time.monotonic() - self._last_use < self.idle_sec:
                return
            listener, self._listener, self._buffer = self._listener, None, None
        listener.unregister()
        self._node.get_logger().info(f"TF 監聽閒置 {self.idle_sec:.0f} 秒，先拆掉（下次要用時再接上）")


def spin_resilient(executor, node, max_errors=50):
    """取代 executor.spin()：接住「拆監聽」競態丟出的 InvalidHandle，繼續跑。

    ★ 2026-10-27：LazyTf 閒置時在計時器回呼裡 listener.unregister()（destroy_subscription），
      多執行緒執行器的主迴圈同時在組 wait set，剛好拿到正被拆的訂閱的 QoS 事件 → InvalidHandle
      「cannot use Destroyable because destruction was requested」從 spin() 丟出來、整個節點死掉。
      10/27 16:1x map_service 就這樣掛了（當天拆了 21 次、中 1 次）：導航其他節點都還在，HMI 卻偵測
      不到導航（偵測靠 map_service_cc），之後每一步要開導航都被「已由受管 launch 執行中」擋下、等 2 分鐘。
      被拆的實體下一圈就不在節點的清單裡了，執行器重新產生等待集合 —— 接住、繼續 spin 是安全的。
    """
    import rclpy
    from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
    errors = 0
    while rclpy.ok():
        try:
            executor.spin()
            return
        except _rclpy.InvalidHandle as exc:
            errors += 1
            node.get_logger().warn(
                f"執行器碰到正在拆除的實體（LazyTf 拆監聽的競態），繼續執行（第 {errors} 次）：{exc}")
            if errors >= max_errors:
                raise
