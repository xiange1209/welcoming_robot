"""遙控：安全夾限、20 Hz 補送/看門狗、直通備援、雷達後方遮罩。

從 hmi_server_node.py 搬出來。發布器仍由節點建立（QoS 設定是 rclpy Node
的職責，見 hmi_server_node.py 建立 teleop_pub/teleop_direct_pub 那段），
這裡只接手發布之後的所有判斷與節流。

★ 安全相關：TELEOP_MAX_LINEAR/ANGULAR 的夾限、看門狗逾時、
  collision_monitor 直通備援、雷達遮罩的「寫得進去不代表生效」警語，
  全部逐行保留，沒有更動數值或邏輯。
"""

import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Parameter, ParameterDescriptor, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters


class TeleopController:
    TELEOP_MAX_LINEAR = 0.18
    TELEOP_MAX_ANGULAR = 0.45

    def __init__(self, node, teleop_pub, teleop_direct_pub, *, watchdog_sec: float, lidar_node_name_param: str):
        self._node = node
        self.teleop_pub = teleop_pub
        self.teleop_direct_pub = teleop_direct_pub
        self._teleop_watchdog_sec = float(watchdog_sec)
        self._lidar_node_name_param = lidar_node_name_param

        self._teleop_fallback_warned = False
        self._teleop_last_cmd = 0.0  # 最後一次收到遙控指令的時間 (monotonic)
        self._teleop_active = False  # 是否還需要送停止命令
        self._teleop_linear = 0.0
        self._teleop_angular = 0.0
        self._teleop_lock = threading.Lock()
        self._teleop_tick_count = 0
        self._teleop_tick_t0 = time.monotonic()
        self._teleop_loop_count = 0
        self._teleop_loop_t0 = time.monotonic()
        self._teleop_thread: Optional[threading.Thread] = None

    def start(self) -> None:
        """啟動 20 Hz 補送迴圈（獨立執行緒）

        **用獨立執行緒而不是 ROS timer。** 實測 0.05 秒的 ROS timer 在這個
        節點的 SingleThreadedExecutor 裡排不上——要同時處理 HTTP 進來的
        服務呼叫、訂閱與動作回呼。遙控是安全相關的即時路徑，不能讓它跟
        一般回呼搶執行器；獨立執行緒不受執行器排程影響，rclpy 的 publish
        本身可以跨執行緒呼叫。
        """
        self._teleop_thread = threading.Thread(target=self._teleop_loop, name="teleop_resend", daemon=True)
        self._teleop_thread.start()

    def _publish_teleop(self, lin: float, ang: float) -> None:
        """把速度指令送到底盤，必要時自動繞過不存在的中繼節點

        teleop_cmd_topic 預設是 cmd_vel_trimmed，指令會經過 collision_monitor
        才到底盤，遙控時一樣有防撞保護。但這只在完整導航堆疊有起來時成立
        （純建圖、純感測器 launch、或 nav_bringup 剛啟動的頭幾分鐘，
        collision_monitor 不存在）。這三種情況下 cmd_vel_trimmed 沒有任何
        訂閱者，指令發出去就消失，HTTP 卻照樣回成功。

        所以發布前先看有沒有人在聽；沒有就直接發 cmd_vel（底盤自己訂的話題）。
        這是刻意的降級而不是預設值改動：有防撞層時仍然走防撞層。
        """
        node = self._node
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.teleop_pub.publish(msg)

        if self.teleop_pub.topic_name.lstrip("/") == "cmd_vel":
            return
        if node.count_subscribers(self.teleop_pub.topic_name) > 0:
            self._teleop_fallback_warned = False
            return

        if not self._teleop_fallback_warned:
            self._teleop_fallback_warned = True
            node.get_logger().warn(
                f"{self.teleop_pub.topic_name} 沒有訂閱者（collision_monitor 未啟動？），"
                f"遙控指令改直接發 cmd_vel —— 此時沒有防撞保護，請放慢速度"
            )
        self.teleop_direct_pub.publish(msg)

    def apply(self, linear: float, angular: float) -> None:
        """套用一次遙控指令並重置看門狗"""
        try:
            lin = float(linear)
            ang = float(angular)
        except (TypeError, ValueError):
            return
        # 不信任前端傳來的數值：夾在安全範圍內，也擋掉 NaN
        if lin != lin or ang != ang:  # NaN
            lin = ang = 0.0
        lin = max(-self.TELEOP_MAX_LINEAR, min(self.TELEOP_MAX_LINEAR, lin))
        ang = max(-self.TELEOP_MAX_ANGULAR, min(self.TELEOP_MAX_ANGULAR, ang))

        with self._teleop_lock:
            self._teleop_linear = lin
            self._teleop_angular = ang
            self._teleop_last_cmd = time.monotonic()
            # 收到零速就直接收工，不要讓看門狗在 0.6 秒後再多噴一次逾時警告
            self._teleop_active = lin != 0.0 or ang != 0.0

        self._publish_teleop(lin, ang)

    def _teleop_loop(self) -> None:
        """20 Hz 補送迴圈（獨立執行緒）

        用 monotonic 推算下一次的絕對時間點而不是固定 sleep(0.05)，
        這樣單次處理慢一點也不會讓整體頻率一路往下掉。
        """
        period = 0.05
        next_at = time.monotonic()
        while rclpy.ok():
            next_at += period
            delay = next_at - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                # 落後太多就重新對時，不要補一堆遲到的指令
                next_at = time.monotonic()
            try:
                self._teleop_tick()
            except Exception as exc:  # noqa: BLE001
                # 這條執行緒掛掉等於看門狗失效，車子可能停不下來 —— 絕不能讓它死
                self._node.get_logger().error(f"遙控補送迴圈異常：{exc}")

            # 量的是**迴圈本身**的頻率，不是「有補送」的次數：
            # 看門狗一觸發就把 _teleop_active 設回 False，之後的 tick 會提早 return，
            # 只計補送次數會把「執行緒沒在跑」和「沒東西要送」混為一談。
            self._teleop_loop_count += 1
            if self._teleop_loop_count % 200 == 0:
                span = time.monotonic() - self._teleop_loop_t0
                if span > 0:
                    self._node.get_logger().info(
                        f"遙控迴圈 {200.0 / span:.1f} Hz（目標 20），" f"其中實際補送 {self._teleop_tick_count} 次"
                    )
                self._teleop_loop_t0 = time.monotonic()

    def _teleop_tick(self) -> None:
        """維持指令流 ＋ 看門狗

        **補送指令**：底盤韌體有 1.0 秒指令逾時，沒收到新指令就自己停。
        如果發布頻率等於 HTTP 到達頻率（前端標稱 5 Hz），那麼 WiFi 抖動、
        TCP 重傳、Pi4 CPU 排隊只要讓連續幾則請求慢下來，底盤就會看到
        超過 1 秒的空窗 → 停車 → 下一則到達又起步，表現就是一頓一頓。
        所以這裡以固定頻率重送目前的設定值，讓網路抖動不會傳導到底盤。

        **看門狗**：連線中斷、鎖螢幕、切 App、瀏覽器分頁被回收，都會讓
        指令停止送達卻沒有任何「停止」訊息 —— 只靠「收到停止才停」
        會讓車子在失聯後繼續跑。
        """
        with self._teleop_lock:
            if not self._teleop_active:
                return
            idle = time.monotonic() - self._teleop_last_cmd
            if idle < self._teleop_watchdog_sec:
                # 還在有效期內：重送目前的設定值，維持指令流不中斷
                lin, ang = self._teleop_linear, self._teleop_angular
                timed_out = False
            else:
                self._teleop_active = False
                self._teleop_linear = 0.0
                self._teleop_angular = 0.0
                lin = ang = 0.0
                timed_out = True

        if not timed_out:
            self._publish_teleop(lin, ang)
            self._teleop_tick_count += 1
            # 每 100 次補送（20 Hz 下約 5 秒）報一次實際速率。
            # 這不是除錯殘留：補送頻率掉下來就代表執行器被塞住，
            # 而症狀會是「車子一頓一頓」，從外面很難判斷是網路還是排程。
            if self._teleop_tick_count % 100 == 0:
                now_s = time.monotonic()
                span = now_s - self._teleop_tick_t0
                if span > 0:
                    self._node.get_logger().info(
                        f"遙控補送 {self._teleop_tick_count} 次，實際 {100.0 / span:.1f} Hz（目標 20）"
                    )
                self._teleop_tick_t0 = now_s
            return

        # 連送三次零速：DDS 掉一則封包不能變成「車子繼續跑」
        for _ in range(3):
            self._publish_teleop(0.0, 0.0)
        self._node.get_logger().warn(f"遙控逾時 {idle:.1f} 秒未收到指令，已停車")

    # ── 雷達後方遮罩 ──────────────────────────────────────

    def set_lidar_rear_mask(self, enabled: bool, half_angle_deg: float) -> tuple:
        """開關雷達後方扇形遮罩

        lslidar 驅動的 angle_disable_min/max 單位是 0.01 度，
        裁掉的是 [min, max] 這個區間。車尾是 180 度。
        """
        try:
            half = max(5.0, min(80.0, float(half_angle_deg)))
        except (TypeError, ValueError):
            half = 35.0

        if enabled:
            lo = int((180.0 - half) * 100)
            hi = int((180.0 + half) * 100)
        else:
            # 兩個都設 0 = 不裁切（驅動的預設值）
            lo = hi = 0

        node_name = self._node.get_parameter(self._lidar_node_name_param).value
        results = []
        # 這兩個參數的型別是 integer_array 而不是 integer ——
        # 驅動支援同時遮蔽多段區間，所以即使只有一段也要包成陣列。
        for name, value in (("angle_disable_min", lo), ("angle_disable_max", hi)):
            ok = self._set_remote_int_array_param(node_name, name, [value])
            results.append(ok)

        if not all(results):
            return False, "設定雷達參數失敗（節點名稱可能不同）"

        # 參數寫得進去，但**驅動只在啟動時讀它** —— 實測寫入
        # angle_disable_min=[14500]、angle_disable_max=[21500] 之後，
        # 後方 43 束雷射仍然 100% 有回波，完全沒有被裁掉。
        # 所以這裡不能回報「已生效」，那會讓操作者以為自己被遮住了而放心站在車後。
        if enabled:
            return True, (
                f"參數已寫入（車尾 ±{half:.0f}°），但雷達驅動只在啟動時讀取，"
                "**本次不會生效**。要真的遮蔽請重啟感測器。"
                "在那之前請不要站在車子正後方。"
            )
        return True, "已清除遮罩參數（下次啟動感測器時生效）"

    def _set_remote_int_array_param(self, node_name: str, param: str, values: list) -> bool:
        """對別的節點設定一個整數陣列參數"""
        node = self._node
        client = node.create_client(SetParameters, f"{node_name}/set_parameters")
        try:
            if not client.wait_for_service(timeout_sec=3.0):
                node.get_logger().warn(f"{node_name} 的參數服務不存在")
                return False
            req = SetParameters.Request()
            p = Parameter()
            p.name = param
            p.value = ParameterValue(
                type=ParameterType.PARAMETER_INTEGER_ARRAY,
                integer_array_value=[int(v) for v in values],
            )
            req.parameters = [p]
            res = node.jobs.await_future(client.call_async(req), timeout=5.0)
            return bool(res and res.results and res.results[0].successful)
        except Exception as exc:  # noqa: BLE001
            node.get_logger().error(f"設定 {node_name}/{param} 失敗: {exc}")
            return False
        finally:
            node.destroy_client(client)
