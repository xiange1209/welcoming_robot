"""導航中暫停人臉相機（2026-10-28 新增）。

為什麼：單板全開時（完整迎賓故事線）CPU 閒置只剩 0~2%，帶位的循跡控制只跑得到 1.7~2.8 Hz，
10/27 故事線帶位去櫃台 289 秒逾時、最大偏離 142 cm；同一天只開導航的 A/B 是 4/4。
人臉推論在導航中本來就停了（face_embedding 的 skip_while_navigating），**剩下的大頭是相機本身**：
astra_camera_node ~40% + JPEG republish ~40%（單核計）。10/20 對照：全開但沒人臉 → 閒置 1%；
再把相機也關 → 閒置 42%。相機只有在「認人」時有用，帶位途中只是給平板一個畫面。

怎麼做：訂 /navigation_active（navigation_action_cc 發、latched）。
    導航開始 → 相機若是「人臉模式」在跑，就用跟平板「停止」一樣的路徑停掉它（HMI 的燈號照實顯示）
    導航結束 → 等 resume_delay 秒（讓車停穩、Nav2 收尾），再用人臉模式開回來（約 1~15 秒出畫面）
不碰的情況：
    - 相機是「避障模式（深度點雲）」：那是給導航用的，不能關
    - 雙板：相機在 B 板，A 不管它
    - 操作者在這段期間自己按了相機的開／關、或套用了動到相機的情境 → 以人為準，不再自動開回來
關掉：`ros2 param set /hmi_server_node pause_camera_while_navigating false`（馬上生效、不用重開 HMI；
      ＝8/27 的舊行為：導航中平板仍有相機畫面）。關掉之前已經暫停的那一次，導航結束照樣會開回來。
"""

import os
import threading
import time

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from ..system.procinfo import proc_cmdlines
from .supervisor import remote_units

CAMERA = "camera"
FACE_VARIANT = "face"


class NavCameraPause:
    def __init__(self, node, supervisor, face_needle: str):
        self._node = node
        self._sup = supervisor
        self._face_needle = face_needle          # 指令列裡認得出「人臉模式」的字串（camera_face_cc.launch）
        # ★ 2026-10-29：預設改成**不暫停**（使用者決定）：帶位時迎賓頁的相機縮成小窗、地圖放大，小窗要有即時畫面；
        #   暫停的代價是到點後約 4~18 秒看不到、認不了人。CPU 不夠時再打開（ros2 param set … true，即時生效）。
        node.declare_parameter("pause_camera_while_navigating", False)
        node.declare_parameter("camera_resume_delay_sec", 3.0)
        self._delay = max(0.0, float(node.get_parameter("camera_resume_delay_sec").value))
        self._lock = threading.Lock()
        self._nav = False
        self._nav_off_at = 0.0
        self._check = False        # 這次導航還沒決定要不要停相機
        self._paused = False       # 相機是這裡停掉的、導航結束要開回來
        self._warned = ""
        # 訂閱與計時器一律建立；開關每拍重讀（★「設了沒用的參數」的教訓：只在 __init__ 讀＝執行中改了也沒用）
        node.create_subscription(
            Bool, "/navigation_active", self._on_nav,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        node.create_timer(1.0, self._tick)

    # ---- 給 manager 用 -----------------------------------------------------
    def forget(self) -> None:
        """操作者自己動了相機（單元開關或情境）：以人為準，不再自動開回來／不再自動停。"""
        with self._lock:
            if self._paused or self._check:
                self._node.get_logger().info("導航中暫停相機：操作者自己動了相機 → 不再自動開關")
            self._paused = False
            self._check = False

    def paused(self) -> bool:
        with self._lock:
            return self._paused

    # ---- ROS 回呼 ----------------------------------------------------------
    def _on_nav(self, msg: Bool) -> None:
        with self._lock:
            nav = bool(msg.data)
            if nav == self._nav:
                return
            self._nav = nav
            # 迎賓頁靠這個旗標切換「地圖放大＋相機小窗」（system.navigating）
            st = getattr(self._node, "state", None)
            if st is not None:
                st.set_system(navigating=nav)
            if nav:
                self._check = True
            else:
                self._check = False
                self._nav_off_at = time.monotonic()

    def _enabled(self) -> bool:
        # ★ 用 .value 讀（10/27 看門狗被 .integer_value 讀成 0 的教訓）
        try:
            return bool(self._node.get_parameter("pause_camera_while_navigating").value)
        except Exception:  # noqa: BLE001
            return True

    def _tick(self) -> None:
        with self._lock:
            nav, check, paused, off_at = self._nav, self._check, self._paused, self._nav_off_at
        if not (check or paused):
            return                                   # 平常什麼都不做（不掃 /proc）
        if check and not self._enabled():
            with self._lock:
                self._check = False                  # 關掉了：這趟不停相機（已經停的照樣在下面開回來）
            return
        if CAMERA in remote_units() or self._sup.transition(CAMERA):
            return                                   # 雙板不管；相機正在開／關 → 下一秒再看
        if nav and check:
            if not self._face_camera_running():
                with self._lock:
                    self._check = False              # 沒開、或是避障模式 → 這趟不動它
                return
            ok, op = self._sup.request_unit(CAMERA, "stop")
            if ok:
                with self._lock:
                    if self._check:                  # 送出前沒有被 forget() 搶先
                        self._check, self._paused = False, True
                self._node.get_logger().info(
                    f"導航開始 → 暫停相機（人臉模式）讓 CPU 給導航；導航結束 {self._delay:.0f} 秒後自動恢復")
            else:
                self._warn_once(f"導航開始，但現在停不了相機：{op.get('message')}（每秒重試）")
            return
        if paused and not nav and time.monotonic() - off_at >= self._delay:
            if self._camera_running():
                with self._lock:
                    self._paused = False             # 已經有人開回來了
                return
            ok, op = self._sup.request_unit(CAMERA, "start", FACE_VARIANT)
            if ok:
                with self._lock:
                    self._paused = False
                self._warned = ""
                self._node.get_logger().info("導航結束 → 恢復相機（人臉模式），約 1~15 秒後有畫面")
            else:
                self._warn_once(f"導航結束，但現在開不了相機：{op.get('message')}（每秒重試）")

    # ---- 小工具 ------------------------------------------------------------
    def _warn_once(self, text: str) -> None:
        if text != self._warned:
            self._warned = text
            self._node.get_logger().warning(text)

    @staticmethod
    def _cmds() -> list:
        # 跟 manager.system_status 同一套排除：自己、shell 包裝（bash -c "..." 會含有關鍵字）
        me = os.getpid()
        return [cmd for pid, cmd in proc_cmdlines() if pid != me and not cmd.lstrip().startswith("/bin/bash")]

    def _camera_running(self) -> bool:
        return any("astra_camera_node" in c for c in self._cmds())

    def _face_camera_running(self) -> bool:
        # ★ 2026-10-29 修正：HMI 開相機的指令列是
        #     ros2 launch smartnav_hmi system_control.launch.py unit:=camera variant:=face
        #   裡面**沒有** camera_face_cc.launch 這個字（那是 launch 檔內部 include 的）——10/28 版只比對它，
        #   所以從平板開的相機永遠不會被暫停（10/28 晚上 23:34 那趟相機開著、沒有任何暫停紀錄）。兩種寫法都認。
        cmds = self._cmds()
        if not any("astra_camera_node" in c for c in cmds):
            return False
        return any(self._face_needle in c or {"unit:=camera", "variant:=face"} <= set(c.split()) for c in cmds)
