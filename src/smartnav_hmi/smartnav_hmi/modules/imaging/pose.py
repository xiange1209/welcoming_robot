"""機器人位姿來源：amcl_pose 優先、TF 備援、odom 外推。

從 hmi_server_node.py 搬出來。仍然吃 `node`：要動態建立/銷毀 amcl_pose
訂閱與 TF 監聽，這些是 rclpy Node 的資源生命週期，理由同 video.py。
"""

import math
import threading
import time
from typing import Dict, Optional

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from .geometry import normalize_angle, quaternion_to_yaw


class PoseTracker:
    # 外推的上限。超過就代表里程計或 amcl 有一邊不對勁（打滑、重定位、
    # 里程計重置），寧可顯示「舊但正確」也不要顯示「新但編出來」的位置。
    EXTRAP_MAX_M = 1.0
    EXTRAP_MAX_RAD = 1.0

    def __init__(self, node, state, map_renderer, *, map_frame: str, robot_frame: str, pose_watch_ttl: float):
        self._node = node
        self.state = state
        self._map_renderer = map_renderer
        self.map_frame = map_frame
        self.robot_frame = robot_frame
        self.pose_watch_ttl = pose_watch_ttl

        self.tf_buffer: Optional[Buffer] = None
        self.tf_listener: Optional[TransformListener] = None
        # 有人在看地圖的期限（time.monotonic()）。0 = 沒人在看。
        self._pose_watch_until = 0.0
        self._amcl_sub = None
        self._amcl_pose: Optional[Dict[str, float]] = None
        self._odom_pose: Optional[Dict[str, float]] = None
        self._odom_at_amcl: Optional[Dict[str, float]] = None
        # 這一輪觀看是否已經拿到過初始位置
        self._pose_seeded = False
        self._pose_lock = threading.Lock()
        self._last_speed_at = 0.0

    def note_watch(self) -> None:
        """記下「現在有人在看地圖」

        由前端的 /api/map/watch 心跳與所有跟地圖有關的端點呼叫。位姿訂閱與
        地圖渲染都掛在這個訊號上——沒人看就一律不做，這是節省 CPU 的關鍵。
        """
        self._pose_watch_until = time.monotonic() + self.pose_watch_ttl

    # ── ROS callbacks ─────────────────────────────────────

    def amcl_pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        """amcl 直接給的 map→base_link，取代 TF 查詢

        amcl 只在粒子重採樣時發布，機器人靜止時完全不發——所以「舊」的位姿
        其實就是「現在」的位姿，這裡不做時效判斷（見 pose_timer_cb）。
        """
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        with self._pose_lock:
            self._amcl_pose = {
                "x": float(p.x),
                "y": float(p.y),
                "yaw": quaternion_to_yaw(q.x, q.y, q.z, q.w),
            }
            # ★ 記下「這一則 amcl 對應到哪個里程計位置」。之後的外推
            #   就是拿現在的里程計減掉這個基準。
            self._odom_at_amcl = dict(self._odom_pose) if self._odom_pose else None

    def odom_speed_cb(self, msg: Odometry) -> None:
        """車速。1 Hz 節流（odom 本身是 20 Hz）。

        位姿要在節流**之前**存：外推需要最新的里程計，而車速 1 Hz 就夠看。
        存三個浮點數的成本可以忽略。
        """
        _p = msg.pose.pose.position
        _q = msg.pose.pose.orientation
        with self._pose_lock:
            self._odom_pose = {
                "x": float(_p.x),
                "y": float(_p.y),
                "yaw": quaternion_to_yaw(_q.x, _q.y, _q.z, _q.w),
            }

        now = time.monotonic()
        if now - self._last_speed_at < 1.0:
            return
        self._last_speed_at = now
        self.state.set_system(speed=round(float(msg.twist.twist.linear.x), 3))

    def _extrapolate(self, amcl, odom_ref, odom_now) -> Dict[str, float]:
        """用里程計把 amcl 位姿外推到「現在」。

        ★ 為什麼需要：`amcl_pose` 是**上一次雷射更新時**的估計。AMCL 的更新
        門檻是 `update_min_d: 0.10` / `update_min_a: 0.10 rad`，也就是車子要
        移動 10 cm 或轉 5.7 度才會發下一則 —— 顯示最多落後 10 cm、5.7 度，
        0.15 m/s 時約 0.67 秒，再加上雷射→濾波→發布的處理延遲。

        ★ 數學：amcl 給的是 map→base，odom 給的是 odom→base。兩則 amcl 之間，
        map→odom 這個修正量是**固定的**（amcl 沒發新的就沒有新的修正），
        所以「現在的 map→base」＝「上次的 map→base」⊕「這段期間的 odom 位移」，
        而位移要先從 odom 座標系轉到 map 座標系（差一個 yaw）。
        """
        if not odom_ref or not odom_now:
            return dict(amcl)  # 沒有里程計就照舊，不要編
        dx = odom_now["x"] - odom_ref["x"]
        dy = odom_now["y"] - odom_ref["y"]
        dyaw = normalize_angle(odom_now["yaw"] - odom_ref["yaw"])
        if math.hypot(dx, dy) > self.EXTRAP_MAX_M or abs(dyaw) > self.EXTRAP_MAX_RAD:
            # 位移大到不合理：打滑、重定位、或里程計被重置過。
            # 這種時候 amcl 馬上就會發新的，等它就好。
            return dict(amcl)
        # 把 odom 座標系的位移轉進 map 座標系（兩者差 amcl.yaw - odom_ref.yaw）
        th = amcl["yaw"] - odom_ref["yaw"]
        c, sn = math.cos(th), math.sin(th)
        return {
            "x": amcl["x"] + c * dx - sn * dy,
            "y": amcl["y"] + sn * dx + c * dy,
            "yaw": normalize_angle(amcl["yaw"] + dyaw),
        }

    # ── 訂閱/TF 按需開關 ──────────────────────────────────

    def _acquire_pose_sources(self) -> None:
        """開始觀看時建立 amcl_pose 訂閱（TF 留到真的需要才建）"""
        if self._amcl_sub is not None:
            return
        node = self._node
        # depth=1：只要最新一筆，補送舊位姿對畫面沒有意義
        self._amcl_sub = node.create_subscription(
            PoseWithCovarianceStamped,
            "amcl_pose",
            self.amcl_pose_cb,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            callback_group=node._cb,
        )

    def _acquire_tf_listener(self) -> None:
        """建立 TF 監聽（只在 amcl_pose 收不到時才走到這裡）"""
        if self.tf_listener is not None:
            return
        node = self._node
        # cache_time 從預設的 10 秒縮到 2 秒：我們永遠只查「最新」的轉換，
        # 留著十秒歷史只是讓每次 set_transform 都往更大的 buffer 裡插。
        self.tf_buffer = Buffer(cache_time=Duration(seconds=2.0))
        # QoS depth 從 tf2_ros 預設的 100 降到 10。堆 100 則的唯一效果是
        # 我們落後時要補做 100 則的 Python 反序列化，只會讓塞車更嚴重。
        self.tf_listener = TransformListener(
            self.tf_buffer,
            node,
            qos=QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            ),
        )
        node.get_logger().info("收不到 amcl_pose，改用 TF 取位姿（建圖模式的正常情形）")

    def _drop_tf_listener(self) -> None:
        """拆掉 TF 監聽（訂閱一消失，/tf 就再也不會喚醒這個節點）"""
        if self.tf_listener is None:
            return
        self.tf_listener.unregister()
        self.tf_listener = None
        self.tf_buffer = None

    def _release_pose_sources(self) -> None:
        """沒人在看地圖了，把位姿相關的訂閱整個拆掉

        這才是省下來的地方：訂閱不存在，rclpy 就完全不會被 /tf 喚醒，
        也不用為每一則訊息付反序列化的代價。
        """
        self._drop_tf_listener()
        self._pose_seeded = False
        if self._amcl_sub is not None:
            self._node.destroy_subscription(self._amcl_sub)
            self._amcl_sub = None
        with self._pose_lock:
            self._amcl_pose = None

    def on_map_switched(self) -> None:
        """換地圖後舊的 amcl_pose 是另一張圖的座標，留著會讓箭頭停在錯的位置"""
        with self._pose_lock:
            self._amcl_pose = None

    def pose_timer_cb(self) -> None:
        """更新機器人位姿，並在需要時渲染地圖

        沒人在看地圖時這支只花一次 monotonic() 比較就返回，是刻意的：
        迎賓展示時平板停在迎賓分頁，這條路徑一整天都不該做任何事。
        """
        watching = time.monotonic() < self._pose_watch_until

        if not watching:
            if self._amcl_sub is not None or self.tf_listener is not None:
                self._release_pose_sources()
                # 訂閱都拆了就沒有新位姿，明確清成 None，免得前端一直畫著
                # 一個早就不再更新的箭頭
                self.state.set_robot_pose(None)
            return

        self._acquire_pose_sources()
        if self._map_renderer is not None:
            self._map_renderer.render_if_needed()

        # amcl 在不在，看圖上有沒有人發布 amcl_pose 就知道。這是本機的圖快取
        # 查詢，不是服務呼叫，成本可以忽略。
        amcl_alive = self._node.count_publishers("amcl_pose") > 0

        with self._pose_lock:
            pose = self._amcl_pose
            odom_now = dict(self._odom_pose) if self._odom_pose else None
            odom_ref = dict(self._odom_at_amcl) if self._odom_at_amcl else None
        if pose is not None:
            # amcl 有資料就絕不碰 TF——這是整個改動的重點
            self._drop_tf_listener()
            self._pose_seeded = True
            self.state.set_robot_pose(self._extrapolate(pose, odom_ref, odom_now))
            return

        if amcl_alive and self._pose_seeded:
            # amcl 活著但沒發新位姿 = 機器人靜止（amcl 只在粒子重採樣時發布）。
            # 位置既然不會變，就沒有理由為了「確認它沒變」養著 /tf 訂閱。
            return

        # 走到這裡只有兩種情況：
        #   (a) 剛開始看，還沒有任何初始位置可畫；
        #   (b) 建圖模式，根本沒有 amcl，只能靠 TF。
        self._acquire_tf_listener()
        try:
            # timeout 一定要留 0：tf2_ros 的 can_transform() 在有 timeout 時是
            # **忙等**（每 20 ms 醒一次），而且會在 ROS 執行緒上原地卡住。
            # 沒有導航時 map→base_link 本來就查不到，原本每次都白等滿 50 ms。
            # 這裡是 2 Hz 的輪詢，等不到就下次再查，不需要等。
            trans = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, Time(), timeout=Duration())
        except Exception:
            # 沒跑導航時本來就沒有 map→base_link，這是正常狀態不是錯誤
            self.state.set_robot_pose(None)
            return

        r = trans.transform.rotation
        self.state.set_robot_pose(
            {
                "x": trans.transform.translation.x,
                "y": trans.transform.translation.y,
                "yaw": quaternion_to_yaw(r.x, r.y, r.z, r.w),
            }
        )
        self._pose_seeded = True
        if amcl_alive:
            # 初始位置拿到了，之後的更新交給 amcl_pose，TF 可以收掉。
            # 這條路徑對應「導航跑著、但機器人停著沒動」——最常見的看地圖情境。
            self._drop_tf_listener()
