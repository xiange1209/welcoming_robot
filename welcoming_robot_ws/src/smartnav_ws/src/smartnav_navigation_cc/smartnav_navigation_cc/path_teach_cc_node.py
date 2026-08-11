#!/usr/bin/env python3
"""教導-重現（Teach & Repeat）：錄下人開過的路徑，之後照著重播。

## 為什麼要這個

自動規劃在這台車上有幾個難以繞開的問題：

- 阿克曼底盤最小轉彎半徑 0.8 m，測試環境走廊只有 0.99 m 寬，掉頭需要 1.6 m
- MPPI 的預測視野 = time_steps(20) x model_dt(0.1) = 2.0 秒，
  以 vx_max 0.25 m/s 計算只能看到 0.5 公尺 —— **比完成一次轉彎所需的
  1.26 公尺還短**，控制器看不到「轉完之後會如何」
- 規劃器 tolerance 0.5 m 與 goal_checker 的 xy 0.25 m 是串接疊加的，
  最壞情況終點誤差 0.75 公尺
- Pi 4 的 CPU 吃緊，MPPI 每週期要算 400 條軌跡 x 20 步

而人開過的路徑有一個很強的性質：**它物理上保證可行**。不會出現
「理論上算得出來、實際開不過去」的路徑，也不需要即時最佳化。
對固定場地的展示（大廳 -> 接待點 -> 洗手間）這是更可靠的做法。

## 錄的是位姿不是指令 —— 這個差別很關鍵

「錄下 cmd_vel 再重播」是**開迴路**：輪胎打滑、地板材質、電池電壓下降
都會讓實際軌跡偏離，而且沒有任何機制發現偏了。走 10 公尺累積 1 度的
朝向誤差就是 17 公分側向偏移，在 0.99 m 走廊裡足以撞牆。

這裡錄的是 `map -> base_footprint` 的**位姿序列**，重播時用 AMCL 的當前
位置算橫向偏差，用純追蹤持續修回去。誤差被綁定在 AMCL 的定位精度上
（走廊裡約 5~10 公分），**不會累積**。

## 純追蹤（pure pursuit）與阿克曼

在路徑上取一個前視點，車體座標系下該點為 (x, y)，前視距離 L_d，則

    曲率 k = 2y / L_d^2          角速度 w = v * k

k 會被夾在 1/0.8 = 1.25（最小轉彎半徑的物理極限）以內。

前視距離採「隨速度調整」：L_d = clamp(k_v * |v| + L_min, L_min, L_max)。
太短會蛇行、太長會切彎（轉彎時內切撞牆），窄走廊寧可短一點。

## 折返點（cusp）

錄製時記錄每個點的行進方向（+1 前進 / -1 後退）。方向改變的地方就是
折返點。重播到折返點時**先停穩再換方向** —— 直接反向會讓底盤的
速度指令瞬間變號，實機上會頓一下，而且純追蹤的幾何在換向瞬間不成立。

這也是這套方案能做多段掉頭（K-turn）的原因：你開的時候怎麼折返，
重播就怎麼折返，不需要控制器自己想出來。

## 障礙處理

    前方 slow_distance 內有障礙 -> 減速
    前方 stop_distance 內有障礙 -> 嘗試橫向繞開，繞不過去就停下等待
    等待超過 wait_timeout       -> 放棄，回報失敗

繞開的可行性是**用即時掃描算出來的**，不是寫死的。0.99 m 走廊裡
人站著（約 0.4 m 寬）+ 車身 0.37 m = 0.77 m，只剩 0.22 m 餘裕，
物理上不該硬繞；大廳寬敞處才繞得過去。
"""
import json
import math
import os
import threading
import time
import uuid
from typing import List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from smartnav_msgs.action import FollowTaughtPath
from smartnav_msgs.msg import TaughtPathInfo
from smartnav_msgs.srv import (
    DeleteTaughtPath,
    ListTaughtPaths,
    PlanTaughtPath,
    RecordPath,
)


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_from_yaw(yaw: float):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def norm_angle(a: float) -> float:
    """把角度收斂到 (-pi, pi]。差角運算沒有這個一定會在 ±180 度附近出錯"""
    return math.atan2(math.sin(a), math.cos(a))


class PathPoint:
    """路徑上的一個點。direction 是錄製當下的行進方向，重播時要照做"""

    __slots__ = ("x", "y", "yaw", "direction")

    def __init__(self, x: float, y: float, yaw: float, direction: int = 1):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.direction = direction      # +1 前進 / -1 後退

    def as_dict(self) -> dict:
        return {"x": self.x, "y": self.y, "yaw": self.yaw, "d": self.direction}

    @staticmethod
    def from_dict(d: dict) -> "PathPoint":
        return PathPoint(float(d["x"]), float(d["y"]), float(d["yaw"]),
                         int(d.get("d", 1)))


class PathTeachNode(Node):

    def __init__(self):
        super().__init__("path_teach_cc_node")
        cb = ReentrantCallbackGroup()

        # ── 參數 ──────────────────────────────────────────────
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("cmd_topic", "cmd_vel_smoothed")

        # ── 脫困的防撞旁路（2026-08-06）★ 唯一會繞過 collision_monitor 的路徑 ──
        #
        # 實測：車子楔住時掃描點落進車身輪廓，collision_monitor 判定碰撞而
        # **停止輸出**。指令到 cmd_vel_trimmed 還是 −0.05，到 cmd_vel 就沒有了。
        #     cmd_vel_smoothed (送的)       −0.05  ✓
        #     cmd_vel_trimmed  (補償後)      −0.05  ✓
        #     cmd_vel          (防撞後→底盤)  沒有訊息  ← 擋在這裡
        # 後果：_do_escape 發了 89 筆指令、車子位移 0.00 m，七段脫困全部無效。
        # ★ 這也是 8/03「車子楔死」那個結論的真正原因 —— 不是判準錯，
        #   是最後一關把指令吃掉了。
        #
        # 已經楔住的時候「不准動」不是安全，是死鎖。所以開一條**窄通道**：
        #   - 只在 _do_escape 期間生效（_escape_bypass 旗標，try/finally 保證關閉）
        #   - 速度硬上限 escape_bypass_speed（預設 0.05 m/s，人工救車實測值）
        #   - 距離上限沿用 escape_distance_m
        #   - 繞過 collision_monitor 之後**自己**做 _side_clearance 與
        #     _arc_clearance 檢查（那兩個檢查本來就在 _do_escape 裡）
        #
        # ★ 不是關掉 collision_monitor —— 正常循跡、繞障、等待全部照舊走它。
        self.declare_parameter("escape_bypass_enabled", True)
        self.declare_parameter("escape_bypass_topic", "cmd_vel")
        self.declare_parameter("escape_bypass_speed", 0.05)
        self.declare_parameter("scan_topic", "scan")
        # 錄製時要「聽」哪個話題判斷行進方向。
        # 這跟 cmd_topic 是**不同的東西**：cmd_topic 是重播時本節點自己
        # 發布的位置（cmd_vel_smoothed，之後還會經過 steering_trim 與
        # collision_monitor）；錄製時要聽的是**真正驅動底盤的那一個**，
        # 也就是遙控直接發布的 /cmd_vel。
        # 2026-08-03 踩過：訂成 cmd_topic 的話收不到遙控指令，方向偵測全失效。
        self.declare_parameter("record_cmd_topic", "cmd_vel")

        # 錄製：距離或角度變化達到門檻才記一個點。
        # 用「走了多遠」而不是「過了多久」當觸發條件，車停著時才不會
        # 在原地堆出幾百個重複的點。
        self.declare_parameter("record_spacing_m", 0.05)
        self.declare_parameter("record_angle_rad", 0.10)
        # 兩個取樣點之間的位移上限。50 Hz tick、車速最高 0.25 m/s -> 正常 5 mm，
        # 留到 0.30 m 是給漏拍的餘裕；超過就是 AMCL 跳了。
        self.declare_parameter("jump_dist_limit_m", 0.30)
        self.declare_parameter("max_points", 20000)
        # 存檔前把過短的方向段併掉（假折返點）的門檻。
        #
        # ★ 2026-08-03 參數化：原本是 _merge_short_segments 的預設引數、
        # 硬編碼 0.35 且無法在啟動時調整。這會擋掉「在窄轉角錄製三點轉向」
        # 這個計畫——淨寬 0.967 m 的走廊裡，單次前進／後退的位移很可能
        # 不到 0.35 m（對照：脫困 v1 沿路徑退也只退了 0.38 m），
        # 人示範的修正動作會被整段當成雜訊併掉，跟修好方向偵測之前一樣白做。
        #
        # 要錄三點轉向時把它降到 0.12~0.15：
        #   ros2 param set /path_teach_cc_node merge_min_segment_m 0.15
        # 一般錄製維持 0.35（理由見 _merge_short_segments 的 docstring）。
        #
        # ★ 2026-08-04：這個值**不快取**，_merge_short_segments 每次存檔時
        # 現讀。原本跟其他參數一樣在 __init__ 讀進 self.merge_min_seg，
        # 於是上面那行 `ros2 param set` 只改到參數伺服器、不會改變行為——
        # 而它正是本參數存在的唯一理由。實測代價：錄了一趟含三點轉向的
        # 路徑，後退段 0.28 m 仍被 0.35 的舊值併掉，兩個折返點全沒了。
        # `ros2 param get` 讀回 0.15 只證明伺服器存了值，不證明節點會用它。
        self.declare_parameter("merge_min_segment_m", 0.35)

        # 重播
        self.declare_parameter("follow_speed", 0.15)
        self.declare_parameter("reverse_speed", 0.10)
        self.declare_parameter("lookahead_min_m", 0.30)
        self.declare_parameter("lookahead_max_m", 0.80)
        self.declare_parameter("lookahead_k", 1.2)      # L_d = k*|v| + min
        # ★★ 2026-08-10：最小轉彎半徑分左右，不是一個數字 ★★
        #
        # 這台車左右轉的能力差 26%，而且是**設計必然、不是故障**。
        # 8/06 直接量（不是反推）：
        #     R_right = 0.751 m   右打滿舵 23.21 度   <- 正好撞韌體硬限
        #     R_left  = 0.944 m   左打滿舵 18.83 度
        #
        # 韌體原始碼給的三個成因（`廠商原始碼分析_阿克曼控制鏈.md`）：
        #   1. 阿克曼幾何：AngleR 是**右前輪**。右轉時它是內輪（0.50 rad），
        #      左轉時是外輪（0.34 rad）——同一個舵機，兩種行程。
        #   2. PWM 映射不對稱：右滿舵 1052 / 左滿舵 824 PWM/rad，左轉弱 22%。
        #   3. 右滿舵 −0.50 rad 算出 1014.10，撞上 Servo_min=1020 被截斷，
        #      實際只到 0.7584（損失 1.1%）。左邊反而有 88 counts 空頭。
        #
        # ★ 韌體的箝制是**對稱**的（usartx.c:622，R < 0.750 就拉大到 0.750），
        #   它不知道左右不一樣。所以下一個左轉 0.76 的指令，韌體會照單全收、
        #   算出舵機到不了的角度、**然後什麼都不回報**。車子就是欠轉。
        #
        # 這很可能就是「走廊持續貼右牆 32.8 cm」的機制：
        # 左轉指令被物理欠轉 -> 車子往右漂。方向完全吻合。
        #   （仍是假說——那行 側/排斥/繞障/合計/曲率 診斷還沒撈到。）
        #
        # nav2 的 minimum_turning_radius 只吃單一對稱值，描述不了這台車；
        # 但**控制器可以**，所以在這裡分開。
        # ★ 為什麼右邊用 0.80 而不是 0.76（2026-08-10 決定）★
        #
        # 8/10 車上把 nav2 的 minimum_turning_radius 調到 0.76，理由是
        # 「0.80 時大廳→起點一律 2 折返，0.76 規劃得出 0 折返」。方向是對的
        # （十趟實測 0 折返 6 勝、多折返 5 敗），但 0.76 的餘裕太薄：
        #     韌體硬限                 0.750
        #     右滿舵被 Servo_min 截斷   0.7584   <- 廠商韌體實際做得到的
        #     8/06 直接量               0.751
        #   -> 設 0.76 只剩 0.2%~1.2% 餘裕，而那筆 0.751 是在「零位偏移
        #      still 有爭議」的當下量的（反推說偏右 2.19 度、直接量卻是
        #      偏左 17.75 度，方向相反）。拿它當唯一依據不夠穩。
        #
        # 所以先用**有大量運行時數背書的 0.80**，並在明天用
        # `~/maprun/steer_asym_check_cc.py` 把左右極限量準，再依數據調。
        # ★ 這條路線本來就跑不通（多折返 5 敗），所以退回 0.80 沒有失去
        #   任何「原本會動的能力」——真正要修的是折返做不好，見 _do_escape。
        self.declare_parameter("min_turning_radius", 0.80)          # 相容用的舊參數
        self.declare_parameter("min_turning_radius_right", 0.80)    # 實測 0.751，留 6.5% 餘裕
        self.declare_parameter("min_turning_radius_left", 0.95)     # 實測 0.944，留 0.6% 餘裕
        # 關掉就退回舊行為（左右都用 min_turning_radius）——出事時的退路
        self.declare_parameter("asymmetric_turning", True)
        # ── 路徑可行性檢查（2026-08-06）★ 門檻是 0.76 不是 0.80 ──
        #
        # 2026-08-06 實測：教導路徑 path_2702ea459c8b 的索引 44~49
        #   長 0.2664 m、轉 22.19 度  ->  需要半徑 0.688 m
        # 而車子的 min_turning_radius 是 0.80、韌體硬限 0.75。
        # **人工遙控時前輪可以刮地，硬轉過比極限更緊的彎；控制器照 0.80
        # 箝制，不會這樣做。** 所以示範裡有一個它無法重現的動作 ——
        # 重播到那一段必定卡住，再多的脫困也沒用（那天七段脫困全滅）。
        #
        # ★ 門檻用 0.76（= 0.80 × 0.95），不要用 0.80：
        #   SmacPlannerHybrid 就是**貼著** 0.80 規劃的，它的運動基元是
        #   11.25 度一格，離散化讓實測落在 0.79872（差 0.2%）。
        #   用 0.80 會對**每一條**規劃路徑誤報。實測對照：
        #       規劃路徑  R = 0.79872   比值 0.998   <- 可行
        #       教導壞段  R = 0.68795   比值 0.860   <- 真的做不到
        #   韌體硬限是 0.75，所以 0.76 同時擋掉低於硬限的路徑。
        self.declare_parameter("feasibility_min_radius", 0.76)
        # 檢查用的滑動視窗最小弧長。
        # 不逐點檢查：教導路徑點距 5 cm，單點對的朝向差被 AMCL 的 yaw 抖動
        # 主導，會把雜訊誤判成緊彎。也不整段檢查：整段平均可能沒事，
        # 而壞的是其中一小截（8/06 那個壞段只有 0.27 m，整段有 2 m 以上）。
        self.declare_parameter("feasibility_window_m", 0.15)
        # 檢查不過時是否拒絕重播。預設只警告 —— 一條路徑可能只有最後
        # 一小段做不到，而走完前面 90% 仍有價值（例如錄影、或人工接手）。
        self.declare_parameter("refuse_infeasible_path", False)
        self.declare_parameter("goal_tolerance_m", 0.12)
        self.declare_parameter("control_rate", 20.0)
        # 偏離路徑超過這個距離就中止：代表定位跑掉或被推走了，
        # 硬追回去反而危險（純追蹤在大偏差下會畫出很大的弧）
        self.declare_parameter("max_cross_track_m", 0.60)
        self.declare_parameter("cusp_pause_sec", 0.6)

        # 障礙
        self.declare_parameter("obstacle_slow_m", 1.20)
        # 0.55 -> 0.45（2026-08-03 實機）。這個距離是從**雷達**量的，
        # 而車頭在雷達前方 0.311 m（base_footprint 在後輪軸心、車頭 0.40、
        # 雷達 0.089），所以 0.55 實際代表「車頭離障礙 0.24 m 就停」——
        # 在門口這種夾道太保守。0.45 對應車頭餘裕 0.14 m，仍然安全。
        self.declare_parameter("obstacle_stop_m", 0.45)
        # 倒車的停止距離要另外給。這兩個都是從**雷達**量的，
        # 而車頭與車尾離雷達的距離差很多（base_footprint 在後輪軸心）：
        #     車頭 0.40 - 雷達 0.089 = 比雷達再前面 0.311 m
        #     車尾 0.09 + 雷達 0.089 = 比雷達再後面 0.179 m
        # 用同一個 0.45 的話：前進時保險桿餘裕 0.14 m、倒車時 0.27 m
        # —— 倒車保守了一倍，實測在走廊裡會卡住不動。
        # 0.33 讓倒車的保險桿餘裕也是 0.15 m，跟前進一致。
        self.declare_parameter("obstacle_stop_reverse_m", 0.33)
        # 0.26 -> 0.22（2026-08-03 實機）。車半寬 0.185，0.26 等於左右各留
        # 7.5 cm。實測重播卡在門口：門框邊緣落在側向 -0.26 m，**正好壓在
        # 帶寬邊界上**，量到前方淨空 0.54 m 對上停止門檻 0.55——差 1 公分
        # 被判定阻擋，等 20 秒後放棄。
        #
        # 收到 0.22（左右各 3.5 cm 餘裕）後同一幀量到 0.56 m，通過。
        #
        # 更根本的理由：**教導路徑是人開過的，物理上保證通得過**。
        # 這裡的檢查只該攔「錄製時不存在的東西」（人、臨時障礙），
        # 不該比實際車身寬那麼多而把門框當障礙。
        self.declare_parameter("obstacle_half_width_m", 0.22)
        self.declare_parameter("avoid_max_offset_m", 0.25)
        self.declare_parameter("avoid_step_m", 0.05)
        self.declare_parameter("avoid_clearance_m", 0.10)
        self.declare_parameter("wait_timeout_sec", 20.0)
        # ── 脫困（2026-08-03 加）─────────────────────────────
        #
        # 原本擋住就只會等，等滿 wait_timeout 才放棄。**障礙是牆的時候，
        # 等待永遠不會成功。** 實測倒車過轉角時右後輪頂到牆，車子等了
        # 20 秒然後放棄——而它需要的只是「往前退開一點、換個角度再進」，
        # 那正是人開車過窄轉角的標準做法（三點轉向）。
        #
        # 動態障礙（人擋路）該等、靜態障礙（牆）該脫困，兩者外觀一樣，
        # 沒辦法從單幀掃描分辨。所以策略是「先等一下下，還在就試著脫困」：
        # 人通常幾秒內會讓開，牆不會。
        self.declare_parameter("escape_after_sec", 4.0)     # 等這麼久還沒通就試脫困
        self.declare_parameter("escape_distance_m", 0.35)   # 沿路徑往回退多遠
        # 3 -> 10（2026-08-03 實測）。使用者回報「修正動作是對的，
        # 但三次前後退太少」。窄轉角的三點轉向本來就要來回好幾趟才轉得夠，
        # 每次只轉 20 度，要轉過 60~90 度的彎需要 4~6 次。
        self.declare_parameter("max_escapes", 10)
        # ── 連續三點轉向（2026-08-10，依使用者現場示範）──────────────
        # 「方向打滿 -> 前後反向打 -> 多次 -> 需要開啟避障」。
        # 一次 _do_escape 裡連做幾段（而不是做一段就把控制權還給純追蹤，
        # 讓它在段與段之間把車推回夾點、而且那幾秒沒有防撞旁路）。
        # 3 段的理由：一段目標 35 度，三段約 105 度，足夠應付 90 度轉角；
        # 再多會把車推出可用空間，而走廊淨寬只有 0.99 m。
        self.declare_parameter("escape_legs_max", 3)
        # 累計轉夠這麼多就收手，不必做滿段數
        self.declare_parameter("escape_total_dyaw_deg", 80.0)
        # 進度檢查：索引這麼久沒前進就當卡住。
        #
        # 2026-08-03 實測：脫困原本只掛在 waiting 狀態，但車子頂到牆時
        # 淨空還沒低到 stop 門檻，狀態是 **slowing**——它用 30% 速度一直
        # 頂著牆，輪子打滑、位姿慢慢漂，直到偏離超過 60 cm 才被攔下。
        # 從頭到尾索引都停在 65/149。
        #
        # 「有沒有在前進」比「看到什麼」可靠得多：不管是頂到牆、輪子打滑、
        # 還是控制器算不出可行解，索引不動就是卡住。
        self.declare_parameter("no_progress_sec", 5.0)
        # no_progress_sec 內至少要沿路徑推進這麼多才算「有在前進」。
        # 取 8 cm：slowing 時速度 30% ≈ 0.03 m/s，5 秒能走 15 cm，
        # 留一半餘裕。用弧長而不是索引數，才不會被點距影響
        # （教導路徑 5 cm、nav2 規劃路徑 15 cm，差三倍）。
        self.declare_parameter("stuck_min_advance_m", 0.08)
        # 「有下指令卻沒動」的比例門檻：實際速率低於指令速率的這個比例，
        # 且持續 no_progress_sec，才算卡住。0.3 = 只跑到指令的三成。
        # ★ 不要用「沿路徑前進多快」當判準——慢慢開不是卡住（見 _follow_loop）。
        self.declare_parameter("stuck_speed_ratio", 0.30)
        # 第二個卡住判準：no_progress_sec 內沿路徑推進不到這麼多公尺就算卡住，
        # 即使車子有在動。0 = 停用。用弧長不用索引（索引會被點距背叛）。
        # 0.10 m 的來源：正常最慢也有 0.03 m/s × 5 s = 0.15 m。
        self.declare_parameter("stall_progress_m", 0.10)
        # 脫困時側向淨空低於這個值就否決「往那一邊轉」的決定（0 = 停用）。
        # 車身半寬 0.185，0.15 表示已經非常貼牆了才介入，不干擾正常朝向修正。
        self.declare_parameter("wall_veto_m", 0.15)
        # 橫移置中（使用者配方：前進打一邊走多、再回打另一邊走少）。
        # 只在側向淨空低於 centre_trigger_m 時才做——它會佔用前方空間。
        self.declare_parameter("centre_enable", True)
        self.declare_parameter("centre_trigger_m", 0.12)
        self.declare_parameter("centre_leg_sec", 1.6)
        self.declare_parameter("centre_back_ratio", 0.65)
        # 脫困方向改由「哪一端空間大」決定，而不是路徑行進方向。
        # margin 是推翻預設所需的最小差距，避免兩端差不多時反覆橫跳。
        self.declare_parameter("escape_dir_by_space", True)
        self.declare_parameter("escape_dir_margin_m", 0.15)
        # 打滑偵測：no_progress_sec 內掃描簽章的**中位數**變化低於這個值（公尺）
        # 而且一直在下指令，就判定輪子在空轉。0 = 停用。
        #
        # 0.02 的來源：2026-08-10 實測車子靜止時中位數雜訊是 0.10~0.75 cm，
        # 取 2 cm 留約三倍餘裕。
        # ⚠️ **門檻的另一半還沒驗**：「車子真的在動時中位數會是多少」需要
        #    實機移動量測，使用者不在場時不做。走廊裡沿長軸移動時，打到側牆
        #    的光束其垂直距離幾乎不變，中位數可能比預期小 —— 這是這個判準
        #    最可能失效的地方，上機第一件事就是量它。
        self.declare_parameter("scan_still_m", 0.02)
        # base_footprint -> laser 的 x 偏移（查 TF：0.089）。
        # ★ 不要跟 0.311 搞混 —— 那是「雷達到車頭保險桿」的距離。
        #   用錯會讓每個雷射點多推 0.222 m；在走廊裡沿長軸看不出來
        #   （平行牆的縱向歧義），進到房間才會冒出來。
        self.declare_parameter("laser_x_offset_m", 0.089)
        # 單側空隙低於這個值就啟動貼牆排斥，把追蹤目標往外推。
        # 做成「靠太近才推開」而不是「隨時往中間拉」：後者會跟路徑追蹤打架，
        # 有些路徑本來就該貼一邊走。
        # ★ 2026-08-10 從 0.15 拉到 0.25 ★
        #
        # 實測診斷行：「側 左0.20/右0.39  排斥+0.00  曲率（未飽和）」——
        # 車子已經明顯偏左，但 0.20 > 0.15 所以排斥完全不作用；
        # 等它漂到 0.05 才開始推，那時需要的修正量已經超過曲率上限
        # （最小轉彎半徑 0.80 m -> 曲率上限 1.25/m），推了也做不到。
        # 使用者的描述是「太貼近左邊門框了」。
        #
        # 走廊 0.99 m、車寬 0.37 m -> 置中時兩側各約 0.31 m。
        # 0.25 的選法：置中時仍然不推（不跟循跡打架，維持原設計意圖），
        # 但一偏到 0.25 就開始溫和修正 —— 在曲率還夠用的時候。
        self.declare_parameter("wall_keepout_m", 0.25)
        self.declare_parameter("wall_push_max_m", 0.12)

        p = self.get_parameter
        self.map_frame = p("map_frame").value
        self.robot_frame = p("robot_frame").value
        self.record_spacing = float(p("record_spacing_m").value)
        self.record_angle = float(p("record_angle_rad").value)
        self.max_points = int(p("max_points").value)
        self.follow_speed = float(p("follow_speed").value)
        self.reverse_speed = float(p("reverse_speed").value)
        self.la_min = float(p("lookahead_min_m").value)
        self.la_max = float(p("lookahead_max_m").value)
        self.la_k = float(p("lookahead_k").value)
        self.min_radius = float(p("min_turning_radius").value)
        # 左右分開的最小轉彎半徑（見宣告處的長註解）。
        # 韌體是原廠的、不動它——不對稱在 ROS 2 端補償。
        self.asym_turn = bool(p("asymmetric_turning").value)
        self.min_radius_right = float(p("min_turning_radius_right").value)
        self.min_radius_left = float(p("min_turning_radius_left").value)
        if not self.asym_turn:
            self.min_radius_right = self.min_radius
            self.min_radius_left = self.min_radius
        self.jump_dist_limit = float(p("jump_dist_limit_m").value)
        # merge_min_segment_m 刻意**不**在這裡快取（見宣告處）：它要能在
        # 錄製前用 `ros2 param set` 臨時調整，快取了就永遠讀不到新值。
        self.goal_tol = float(p("goal_tolerance_m").value)
        self.control_dt = 1.0 / max(1.0, float(p("control_rate").value))
        self.max_xte = float(p("max_cross_track_m").value)
        self.cusp_pause = float(p("cusp_pause_sec").value)
        self.obs_slow = float(p("obstacle_slow_m").value)
        self.obs_stop = float(p("obstacle_stop_m").value)
        self.obs_stop_rev = float(p("obstacle_stop_reverse_m").value)
        self.obs_half_w = float(p("obstacle_half_width_m").value)
        self.avoid_max = float(p("avoid_max_offset_m").value)
        self.avoid_step = float(p("avoid_step_m").value)
        self.avoid_clear = float(p("avoid_clearance_m").value)
        self.wait_timeout = float(p("wait_timeout_sec").value)
        self.escape_after = float(p("escape_after_sec").value)
        self.escape_dist = float(p("escape_distance_m").value)
        self.max_escapes = int(p("max_escapes").value)
        self.escape_legs_max = int(p("escape_legs_max").value)
        self.escape_total_dyaw = math.radians(float(p("escape_total_dyaw_deg").value))
        self._escape_leg_yaw = None
        self.no_progress = float(p("no_progress_sec").value)
        self.stuck_min_advance = float(p("stuck_min_advance_m").value)
        self.stuck_speed_ratio = float(p("stuck_speed_ratio").value)
        self.stall_progress = float(p("stall_progress_m").value)
        self.wall_veto_m = float(p("wall_veto_m").value)
        self.centre_enable = bool(p("centre_enable").value)
        self.centre_trigger_m = float(p("centre_trigger_m").value)
        self.centre_leg_sec = float(p("centre_leg_sec").value)
        self.centre_back_ratio = float(p("centre_back_ratio").value)
        self.escape_dir_by_space = bool(p("escape_dir_by_space").value)
        self.escape_dir_margin = float(p("escape_dir_margin_m").value)
        self.scan_still_m = float(p("scan_still_m").value)
        self.laser_x = float(p("laser_x_offset_m").value)
        self.wall_keepout = float(p("wall_keepout_m").value)
        self.wall_push_max = float(p("wall_push_max_m").value)

        # ── 狀態 ──────────────────────────────────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, qos=20, static_qos=1)

        self._db_lock = threading.Lock()
        self._paths: dict = {}          # path_id -> {"meta": {...}, "points": [PathPoint]}
        self._db_dir = os.path.join(os.path.expanduser("~"), ".smartnav", "path_database")
        os.makedirs(self._db_dir, exist_ok=True)
        self._db_file = os.path.join(self._db_dir, "paths.json")
        self._load_db()

        self._recording = False
        self._rec_points: List[PathPoint] = []
        self._rec_lock = threading.Lock()

        self._scan: Optional[LaserScan] = None
        self._scan_lock = threading.Lock()

        self.current_map = ""
        self._following = False
        self._active_path_id = ""
        self._rec_jumps = 0
        self._rec_abort_reason = ""

        # ── 介面 ──────────────────────────────────────────────
        cmd_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)
        self.cmd_pub = self.create_publisher(Twist, p("cmd_topic").value, cmd_qos)

        # 脫困旁路：直接發到底盤訂閱的話題，繞過 collision_monitor。
        # 平時不發任何東西 —— 只有 _escape_bypass 為 True 時才會有輸出。
        self._live_start = None      # _plan_cb 標記用，見 _plan_leg
        self._escape_bypass = False
        self.escape_bypass_speed = abs(float(p("escape_bypass_speed").value))
        self.bypass_pub = None
        if bool(p("escape_bypass_enabled").value):
            bypass_topic = p("escape_bypass_topic").value
            if bypass_topic == p("cmd_topic").value:
                # 兩者相同就沒有旁路可言，而且會變成對同一話題重複發布
                self.get_logger().warn(
                    f"escape_bypass_topic 與 cmd_topic 都是 {bypass_topic}，旁路停用")
            else:
                self.bypass_pub = self.create_publisher(Twist, bypass_topic, cmd_qos)
                self.get_logger().info(
                    f"脫困防撞旁路已啟用：{bypass_topic}，"
                    f"速度上限 {self.escape_bypass_speed:.2f} m/s")

        # ★ 把三層轉彎半徑一次印出來（2026-08-10）★
        #
        # 這三個數字**必須滿足 韌體 ≤ 控制器 ≤ 規劃器**，否則會出現
        # 「規劃器產生控制器做不到的路徑」或「控制器發舵機到不了的指令」。
        # 8/10 就同時踩到兩個：規劃器被調到 0.76 而控制器還是 0.80，
        # 且左右都用同一個值而實車左轉只到 0.944。
        # 這種不一致在 log 裡看不見、跑起來也不報錯，只會表現成「循跡變差」，
        # 所以每次啟動都印一次，讓它變成看得見的東西。
        if self.asym_turn:
            self.get_logger().info(
                f"轉彎半徑（左右分開）：左 {self.min_radius_left:.2f} m / "
                f"右 {self.min_radius_right:.2f} m　"
                f"| 實測極限 左 0.944 / 右 0.751　| 韌體硬限 0.750（對稱）")
            self.get_logger().info(
                f"　→ 曲率上限：左 {1.0/self.min_radius_left:.3f} / "
                f"右 {1.0/self.min_radius_right:.3f} /m　"
                f"（可行性門檻 左 {self.min_radius_left*0.95:.2f} / "
                f"右 {self.min_radius_right*0.95:.2f}）")
            self.get_logger().warn(
                "　★ nav2 的 minimum_turning_radius 是單一對稱值，請確認它 ≥ "
                f"{self.min_radius_right:.2f}；規劃器比控制器緊會產生跟不上的路徑")
        else:
            self.get_logger().warn(
                f"轉彎半徑用對稱舊行為：{self.min_radius:.2f} m（asymmetric_turning=false）"
                "　★ 實車左轉極限是 0.944，這個設定會在左轉時發出舵機到不了的指令")
        # 讓 RViz / HMI 看得到目前在追哪條路徑
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        self.path_pub = self.create_publisher(Path, "taught_path", latched)

        scan_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                              history=HistoryPolicy.KEEP_LAST)
        # 錄製時的行進方向：直接看送給底盤的速度指令，不要用位置反推。
        #
        # 原本從「位移向量與車頭的夾角」推方向。慢速來回修正（門口掉頭）時
        # **AMCL 的位置抖動跟真實位移同一個量級**，夾角落進模糊帶，
        # 程式就沿用上一點的方向 —— 整段 K-turn 被記成單向行駛。
        # 重播時純追蹤把「前後修正」當成一條連續弧線去追，
        # 而那條弧線的曲率遠超物理極限（實測有 25 rad/m，半徑 4 cm），
        # 車子打滿舵也走不出去。
        #
        # 速度指令的正負號是**確定的**：使用者按前進鍵就是正、後退鍵就是負。
        # 拿最不可靠的資料（AMCL 位置）去解一個已知的問題，是本末倒置。
        self._cmd_dir = 1
        # 最後一次收到有效速度指令的時刻。**目前只寫不讀**——本來想做
        # 「指令過期就不信任方向」的 timeout，但沒有實測依據可以決定
        # 多久算過期：設太短，正常的減速停頓會被誤判成方向失效；
        # 設太長就沒有意義。留著時間戳是為了之後要補判斷時不用再動 callback。
        # 在補上之前，_cmd_dir 會無限期沿用最後一次的方向——錄製時操作者
        # 一直在遙控，所以實務上不會遇到，但換控制來源時要記得這件事。
        self._cmd_dir_t = 0.0
        # 一串脫困共用的轉向（見 _do_escape）。每次真的往前推進就清掉，
        # 讓下一次卡住重新判斷。
        self._escape_steer: Optional[float] = None
        self._cmd_v_last = 0.0        # 最後一次下的線速度指令（卡住判定用）
        from rclpy.qos import ReliabilityPolicy as _RP
        self.create_subscription(
            Twist, p("record_cmd_topic").value, self._cmd_cb,
            QoSProfile(depth=1, reliability=_RP.RELIABLE, history=HistoryPolicy.KEEP_LAST))
        self.create_subscription(LaserScan, p("scan_topic").value,
                                 self._scan_cb, scan_qos, callback_group=cb)
        self.create_subscription(String, "current_map", self._map_cb, latched,
                                 callback_group=cb)

        self.create_service(RecordPath, "record_path", self._record_cb, callback_group=cb)
        self.create_service(ListTaughtPaths, "list_taught_paths", self._list_cb,
                            callback_group=cb)
        self.create_service(DeleteTaughtPath, "delete_taught_path", self._delete_cb,
                            callback_group=cb)
        self.create_service(PlanTaughtPath, "plan_taught_path", self._plan_cb,
                            callback_group=cb)

        # 地圖點選模式借用 nav2 的規劃器。規劃器（SmacPlannerHybrid + REEDS_SHEPP）
        # 本來就會產生符合最小轉彎半徑、含折返點的阿克曼可行路徑 —— 出問題的是
        # MPPI 控制器，不是規劃器。所以「用 nav2 規劃、用純追蹤執行」。
        self._planner_client = ActionClient(
            self, ComputePathToPose, "compute_path_to_pose", callback_group=cb)

        # 規劃前先清一次全域成本地圖的 obstacle 層（2026-08-05）。
        #
        # 8/04 實測：清空前 13 個取樣點中 11 個 = 254（走廊全段不可通行），
        # 清空後只剩 2 個 —— 同一個規劃請求從失敗變成產出 51 點含 2 折返點
        # 的可行路徑。污染源已在 config 修掉（obstacle_layer 改吃 /scan_slam），
        # 這裡是第二道保險：靜態層有完整地圖，obstacle 層在一次任務開始時
        # 沒有任何值得保留的東西，清掉不會遺失資訊。
        self._clear_costmap_client = self.create_client(
            ClearEntireCostmap, "/global_costmap/clear_entirely_global_costmap",
            callback_group=cb)

        self._action = ActionServer(
            self, FollowTaughtPath, "follow_taught_path",
            execute_callback=self._execute_follow,
            goal_callback=lambda _: (GoalResponse.REJECT if self._following
                                     else GoalResponse.ACCEPT),
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=cb,
        )

        # 錄製取樣。50 Hz 只是查 TF，成本很低；真正決定點密度的是距離門檻。
        self.create_timer(0.02, self._record_tick, callback_group=cb)

        self.get_logger().info(
            f"教導路徑節點啟動：資料庫 {self._db_file}，已載入 {len(self._paths)} 條路徑"
        )

    # ==================================================================
    # 資料庫
    # ==================================================================
    def _load_db(self) -> None:
        if not os.path.exists(self._db_file):
            return
        try:
            with open(self._db_file, "r", encoding="utf-8") as fp:
                raw = json.load(fp).get("paths", {})
            for pid, entry in raw.items():
                self._paths[pid] = {
                    "meta": entry.get("meta", {}),
                    "points": [PathPoint.from_dict(d) for d in entry.get("points", [])],
                }
        except (OSError, ValueError, KeyError) as exc:
            self.get_logger().error(f"讀取教導路徑資料庫失敗：{exc}")

    def _save_db(self) -> bool:
        try:
            payload = {
                "paths": {
                    pid: {"meta": e["meta"], "points": [pt.as_dict() for pt in e["points"]]}
                    for pid, e in self._paths.items()
                }
            }
            tmp = self._db_file + ".tmp"
            with open(tmp, "w", encoding="utf-8") as fp:
                json.dump(payload, fp, ensure_ascii=False, indent=1)
            # 先寫暫存再 rename：中途斷電不會留下半個 JSON 檔把整個資料庫毀掉
            os.replace(tmp, self._db_file)
            return True
        except OSError as exc:
            self.get_logger().error(f"寫入教導路徑資料庫失敗：{exc}")
            return False

    # ==================================================================
    # 訂閱
    # ==================================================================
    def _cmd_cb(self, msg: Twist) -> None:
        """記住最後一次「有意義」的速度指令方向

        門檻 0.02 m/s：低於這個值是停車或雜訊，維持原方向不要翻轉，
        否則減速到零的那一瞬間會被記成折返點。
        """
        if abs(msg.linear.x) > 0.02:
            self._cmd_dir = 1 if msg.linear.x > 0 else -1
            self._cmd_dir_t = time.monotonic()

    def _scan_cb(self, msg: LaserScan) -> None:
        with self._scan_lock:
            self._scan = msg

    def _scan_signature(self, bins: int = 60) -> Optional[List[float]]:
        """把當前掃描壓成 bins 個數字，用來判斷「車子到底有沒有真的移動」。

        ★ 為什麼需要這個（2026-08-10 實測）★

        既有的兩個卡住判準都建立在**位姿**上（map->base_footprint 的位移、
        沿路徑的弧長推進）。但在走廊裡這條鏈是會說謊的：

            車子右側頂住牆 -> 輪子空轉打滑 -> 里程計謊報「有在前進」
            -> EKF 照收 -> AMCL 想修正，但走廊沿長軸沒有分辨力
            （掃描匹配對縱向位置給不出反對意見）-> AMCL 跟著里程計一路跑掉

        於是位姿一直在變、索引一直在推進，**兩個判準都被同一個謊言騙過去**。
        2026-08-10 大廳->起點 實測：偏離單調長到 282 cm、吻合度掉到 59.9%、
        AMCL 報出地圖邊界外的座標，而脫困**一次都沒被觸發**。

        雷射掃描不經過這條鏈。車子真的移動 10 cm，走廊裡的掃描剖面會明顯改變；
        頂著牆空轉時它幾乎不動。這是打滑偽造不了的證據。

        取每個扇區的**最小值**而不是平均：對單點雜訊穩健，
        而牆面距離正是我們要看的量。
        """
        with self._scan_lock:
            scan = self._scan
        if scan is None or not scan.ranges:
            return None
        n = len(scan.ranges)
        if n < bins:
            return None
        step = n / float(bins)
        out: List[float] = []
        rmax = scan.range_max
        for i in range(bins):
            lo, hi = int(i * step), max(int(i * step) + 1, int((i + 1) * step))
            best = rmax
            for r in scan.ranges[lo:hi]:
                if r == r and scan.range_min < r < rmax and r < best:
                    best = r
            out.append(best)
        return out

    @staticmethod
    def _sig_diff(a: Optional[List[float]], b: Optional[List[float]]) -> Optional[float]:
        """兩份掃描簽章的**中位數**絕對差（公尺）。任一為 None 就回 None。

        ★ 為什麼是中位數不是平均（2026-08-10 實測）★
        車子完全靜止、連量四次、每次間隔 5 秒：

            平均   0.98 / 0.93 / 5.26 / 5.89 cm     <- 雜訊上限 5.89 cm
            中位數 0.10 / 0.10 / 0.10 / 0.75 cm     <- 雜訊上限 0.75 cm

        平均被少數亂跳的扇區主導（掃到門框邊緣、或某些角度時有時無回波），
        雜訊比我們要偵測的訊號還大——用平均的話門檻根本訂不出來。
        中位數對這種離群值免疫，靜止時穩定在 1 cm 以內。
        """
        if not a or not b or len(a) != len(b):
            return None
        d = sorted(abs(x - y) for x, y in zip(a, b))
        n = len(d)
        return d[n // 2] if n % 2 else (d[n // 2 - 1] + d[n // 2]) / 2.0

    def _map_cb(self, msg: String) -> None:
        self.current_map = msg.data

    def _robot_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, Time())
        except Exception:
            return None
        t = tf.transform.translation
        return t.x, t.y, yaw_from_quat(tf.transform.rotation)

    # ==================================================================
    # 錄製
    # ==================================================================
    def _record_tick(self) -> None:
        """以固定頻率查位姿，但只有「走了夠遠」才真的記點"""
        if not self._recording:
            return
        pose = self._robot_pose()
        if pose is None:
            return
        x, y, yaw = pose

        with self._rec_lock:
            if not self._rec_points:
                self._rec_points.append(PathPoint(x, y, yaw, 1))
                return

            last = self._rec_points[-1]
            dx, dy = x - last.x, y - last.y
            dist = math.hypot(dx, dy)
            dyaw = abs(norm_angle(yaw - last.yaw))

            if dist < self.record_spacing and dyaw < self.record_angle:
                return
            # 角度觸發也要有最小位移。
            #
            # record_angle_rad 是 0.10 rad = 5.7 度，而 AMCL 的 yaw 抖動本來
            # 就有這個量級。車子幾乎靜止時，光靠抖動就會不斷觸發記點，
            # 錄出「位移 4 公釐、轉 5.9 度」這種點——換算曲率 25 rad/m、
            # 轉彎半徑 4 公分，純追蹤根本追不動（實測門口就是卡在這裡）。
            #
            # 1 公分是「真的有在動」的下限：0.06 m/s 下 0.17 秒的行程，
            # 遠大於 AMCL 的位置雜訊。
            if dist < 0.01:
                return
            if len(self._rec_points) >= self.max_points:
                return

            # ── 定位跳位偵測（2026-08-03 加）──────────────────────
            #
            # 錄製的是 AMCL 位姿，而 AMCL 在走廊裡會跳。實測一條 18 m 的路徑
            # 在第 88 點出現 **3.85 公尺**的瞬移（相鄰點本該只差 5 公分），
            # 等於前後半段落在兩個相差近 4 公尺的座標系裡。重播時純追蹤會
            # 朝著跳位後的點直線開過去——穿越一段從沒走過的空間。
            #
            # 這個 tick 是 50 Hz，車子最快 0.25 m/s，兩次之間最多走 5 mm；
            # 就算漏跑幾拍也不該超過 0.3 m。超過就是定位跳了，不是車子動了。
            #
            # 同理，轉角也有物理上限：最小轉彎半徑 0.80 m 之下，
            # 走 dist 公尺最多轉 dist/0.80 弧度。超過就是位姿跳動。
            #
            # 丟掉這種點而不是照收：路徑寧可少幾個點（純追蹤本來就會內插），
            # 也不能有斷點。同時記下來，存檔時一併回報。
            # ★ 跳位必須「中止錄製」而不是「丟棄這一點」★
            #
            # 第一版寫成丟棄，測試才發現那是錯的：跳位之後車子的真實位姿
            # 距離「最後保留的點」永遠是 3.85 m，於是後續每一點都被拒收 ——
            # 錄製在跳位當下實質死掉，而操作者毫無所覺，繼續開完 18 公尺
            # 才發現只錄到 5 公尺。默默失敗比大聲失敗糟得多。
            #
            # 而且定位一旦跳過，跳位前後的點就落在兩個不同的座標系裡，
            # 這條路徑本質上已經廢了，補救沒有意義。直接中止、講清楚原因、
            # 請操作者重錄，才是對的。
            if dist > self.jump_dist_limit:
                self._rec_jumps += 1
                self._rec_abort_reason = (
                    f"定位跳位 {dist:.2f} m（相鄰取樣點正常只差 {self.record_spacing:.2f} m）。"
                    f"跳位前後的點在不同座標系，路徑已無法使用"
                )
                self._recording = False
                self.get_logger().error(f"錄製中止：{self._rec_abort_reason}")
                return
            # ── 轉角檢查已移除（2026-08-03 實機驗證後） ──────────
            #
            # 原本這裡拒收「轉角超過 dist/min_radius」的點，理由是物理上
            # 做不到。**那個檢查在實機上是災難性的**，實測一趟錄製拒收
            # 570 次後中止：
            #
            #   1. 連鎖拒收。點被拒收後「最後保留的點」不動，車子繼續走，
            #      距離累積 0.05 -> 0.10 -> ... -> 0.30，最後觸發距離上限
            #      而中止。那個「0.30 m 跳位」根本不是真的跳位。
            #      這正是我在跳位那條識破的連鎖模式，卻在這裡重犯，
            #      還在註解裡寫「不會連鎖拒收」。
            #
            #   2. 門檻在 5 cm 的粒度上無法執行：上限只有 3.6 度 + 3 度容忍，
            #      而 **AMCL 的 yaw 抖動本身就有好幾度**，正常行駛也會違反。
            #
            # 小幅 yaw 雜訊不會破壞路徑，純追蹤會平滑掉它。真正會毀掉路徑的
            # 是「位置整個跳掉」，由上面的距離檢查負責且是中止而非丟棄。
            # 一道防線就夠，兩道會互相干擾。

            # 行進方向：直接取自速度指令（見 _cmd_cb），不從位置反推。
            #
            # 演進過程（兩次都錯，記下來免得再犯）：
            #   v1  投影量 > 0.01 m -> 轉彎時側向位移大、投影趨近零，
            #       AMCL 抖動就翻轉正負號，18 m 錄出 29 個假折返點。
            #   v2  改看夾角（cos > 0.5 / < -0.5）-> 直線段沒問題，
            #       但門口慢速來回修正時位置抖動與真實位移同量級，
            #       夾角落進模糊帶而沿用舊值，**整段 K-turn 被記成單向**。
            #       重播時純追蹤要追一條 25 rad/m 的弧線（半徑 4 cm），
            #       車子打滿舵也走不出去。
            #   v3  用 cmd_vel 的正負號。使用者按前進鍵就是正、後退就是負，
            #       確定、無雜訊、而且本來就有。
            #
            # 教訓：能直接量到的量，不要從別的量反推。
            direction = self._cmd_dir
            self._rec_points.append(PathPoint(x, y, yaw, direction))

    def _smooth_path(self, pts, window=2):
        """【已停用，實測無效】對 x,y 做移動平均

        2026-08-03 實測結果：超限點數 35->15 有改善，但**最大曲率
        37.50 -> 158.00 反而變成 4 倍**，路徑長度縮了 36 公分（切彎）。
        原因是移動平均會把「車子幾乎沒動」那幾段的點擠得更近，
        外接圓公式 4A/(d1*d2*d3) 的分母趨近零，曲率就爆掉。

        真正該做的是在**錄製時**就不要記那些點（已用 dist < 0.01 擋掉一部分），
        或改用保長度的平滑（例如樣條擬合），不是事後做移動平均。

        原始說明：

        相鄰取樣點只隔 5 公分，而 AMCL 的位置雜訊約 1 公分。三點外接圓
        算出來的曲率是 8*jitter/spacing^2 = 8*0.01/0.0025 ≈ 32 rad/m ——
        看起來像半徑 3 公分的急彎，實際上只是抖動。實測一條 7.45 m 的路徑
        有 35/139 個點的幾何曲率超過物理極限 1.25 rad/m。

        純追蹤的前視距離是 0.30~0.80 m（往前 6~16 個點），本身就會平滑掉
        大部分雜訊，所以未濾波也能跑（實測循跡誤差 3.4 cm）。但濾掉之後
        曲率更接近真實、前視點的選擇也更穩定。

        **只平滑 x,y，不動 yaw 與 direction**：
          - yaw 純追蹤根本不用（轉向是從前視點的位置算的）
          - direction 是折返點的依據，平滑會把它糊掉

        端點不動，避免起終點位置被拉偏。
        """
        if len(pts) < 2 * window + 1:
            return pts
        out = list(pts)
        for i in range(window, len(pts) - window):
            xs = sum(pts[j].x for j in range(i - window, i + window + 1))
            ys = sum(pts[j].y for j in range(i - window, i + window + 1))
            k = 2 * window + 1
            out[i] = PathPoint(xs / k, ys / k, pts[i].yaw, pts[i].direction)
        return out

    def _check_feasibility(self, pts: List[PathPoint]) -> Tuple[bool, str, list]:
        """檢查路徑上每一小截所需的轉彎半徑，抓出車子做不到的地方。

        回傳 `(可行, 一句話摘要, 最糟的幾段)`。

        === 為什麼需要（2026-08-06 實測）===

        教導路徑 `path_2702ea459c8b` 索引 44~49：長 0.2664 m、轉 22.19 度
        -> 需要半徑 **0.688 m**，而車子的極限是 0.80、韌體硬限 0.75。

        **人工遙控時前輪可以刮地，硬轉過比極限更緊的彎；控制器照
        `min_turning_radius` 箝制，不會這樣做。** 所以示範裡有一個它
        無法重現的動作 —— 重播到那裡必定卡住，脫困再多次也沒用
        （那天七段脫困全滅、最後人工救車）。

        對照同一個轉角的規劃路徑：`R = 0.79872 m`。
        `SmacPlannerHybrid` 的 `minimum_turning_radius` 是**建構性保證**，
        不會產生這種段落 —— 這也是「用 nav2 規劃、用純追蹤執行」
        比「重播遙控結果」可靠的原因之一。

        === 為什麼用滑動視窗 ===

        **不逐點檢查**：教導路徑點距 5 cm，單點對的朝向差被 AMCL 的
        yaw 抖動主導，會把雜訊誤判成緊彎。
        **也不整段檢查**：整段的平均半徑可能完全正常，而壞的是其中
        一小截 —— 8/06 那個壞段只有 0.27 m，它所在的方向段超過 2 m。
        取一個約 0.15 m 的視窗滑過去，既平滑掉雜訊又抓得到局部。

        折返點兩側不跨段檢查：車子在那裡會停下來換向，兩側的朝向差
        不代表任何一次連續轉彎。
        """
        # ★ 門檻由「控制器實際做得到的半徑」推導，不是寫死（2026-08-10）★
        #
        # 舊版寫死 0.76（＝當時的 min_turning_radius 0.80 × 0.95）。今天規劃器
        # 改成 0.76 之後，那個常數就同時是「規劃器的目標值」和「檢查門檻」——
        # 規劃器貼著 0.76 規劃、離散化後落在 0.7585，**每一條規劃路徑都會誤報**。
        #
        # 改成跟著控制器的能力走，兩者永遠同步，不會再各自漂走：
        #     門檻 = 該方向的最小半徑 × 0.95
        # 那 5% 是留給離散化的：SmacPlannerHybrid 的運動基元是 11.25 度一格，
        # 實測輸出落在標稱值的 0.998 倍；再加上 AMCL 的 yaw 抖動，5% 才夠。
        margin = 0.95
        min_r_right = self.min_radius_right * margin
        min_r_left = self.min_radius_left * margin
        # 相容：舊參數若被明確設過（非預設 0.76），仍以它為準覆寫兩邊
        _explicit = float(self.get_parameter("feasibility_min_radius").value)
        if abs(_explicit - 0.76) > 1e-6:
            min_r_right = min_r_left = _explicit
        min_r = min_r_right      # 預設值；下面每段會依轉向覆寫
        win = float(self.get_parameter("feasibility_window_m").value)
        if len(pts) < 3:
            return True, "點數太少，略過檢查", []

        segs = []
        start = 0
        for i in range(1, len(pts)):
            if pts[i].direction != pts[i - 1].direction:
                segs.append((start, i))
                start = i
        segs.append((start, len(pts)))

        bad = []
        worst_r = float("inf")
        for a, b in segs:
            for i in range(a, b - 1):
                arc = 0.0
                j = i + 1
                while j < b and arc < win:
                    arc += math.hypot(pts[j].x - pts[j - 1].x, pts[j].y - pts[j - 1].y)
                    j += 1
                if arc < win * 0.5:
                    break        # 這一段剩下的長度湊不出視窗了，換下一段
                # ★ 帶號：正 = 往左轉，負 = 往右轉（2026-08-10）★
                # 舊版取 abs() 把方向資訊丟掉了，於是只能用單一門檻。
                # 但這台車左右能力差 26%（左 0.944 / 右 0.751），
                # 一個門檻必然在一邊太鬆、另一邊太緊。
                dyaw_signed = norm_angle(pts[j - 1].yaw - pts[i].yaw)
                dyaw = abs(dyaw_signed)
                if dyaw < math.radians(1.0):
                    continue     # 幾乎直行，需要的半徑趨近無限大
                r = arc / dyaw
                worst_r = min(worst_r, r)
                # 該轉向做不做得到，用該轉向的門檻判
                # ★ 實際打哪邊 = sign(轉向 x 行進方向)（見 _clamp_curv 的推導）
                _left = (dyaw_signed > 0) if pts[i].direction >= 0 else (dyaw_signed < 0)
                min_r = min_r_left if _left else min_r_right
                if r < min_r:
                    bad.append({
                        "from": i,
                        "to": j - 1,
                        "arc_m": round(arc, 4),
                        "dyaw_deg": round(math.degrees(dyaw), 2),
                        "radius_m": round(r, 4),
                        # 哪一邊轉不過去——左右門檻不同，沒有這欄看不出原因
                        "turn": "左" if dyaw_signed > 0 else "右",
                        "limit_m": round(min_r, 4),
                    })

        thr_txt = (f"門檻 左{min_r_left:.2f}／右{min_r_right:.2f}"
                   if abs(min_r_left - min_r_right) > 1e-6 else f"門檻 {min_r_right:.2f}")
        if not bad:
            note = (f"可行性檢查通過：最緊處需要半徑 {worst_r:.3f} m（{thr_txt}）"
                    if worst_r < float("inf") else "可行性檢查通過：全程近似直線")
            return True, note, []

        bad.sort(key=lambda d: d["radius_m"])
        w = bad[0]
        side = w.get("turn", "?")
        note = (f"★ 有 {len(bad)} 處車子做不到：最緊的在索引 {w['from']}~{w['to']}，"
                f"{w['arc_m']:.3f} m 內往{side}轉 {w['dyaw_deg']:.1f} 度 → 需要半徑 "
                f"{w['radius_m']:.3f} m（{thr_txt}）"
                f"。控制器箝制：左 {self.min_radius_left:.2f}／右 "
                f"{self.min_radius_right:.2f} m，韌體硬限 0.75 m")
        return False, note, bad[:5]

    def _merge_short_segments(self, pts, min_seg_m=None):
        """把長度不足的方向段併進前一段，消掉假折返點。

        方向現在讀 cmd_vel 的正負號（比舊的位移投影可靠得多），但操作者
        減速、微調時仍可能產生一兩點的極短方向段。真正的折返一定要停車、
        打方向、再走一段，所以「太短的方向段」多半是雜訊。

        預設門檻 0.35 m 的來由：這台車最小轉彎半徑 0.80 m，做一次有意義的
        折返修正至少要走過該圓弧的一小段（0.8 x 0.4 rad 約 0.32 m）。
        取 0.35 m 略高於它，寧可漏掉極短的真折返，也不要留下假的——
        假折返會讓重播時無謂地停車換向。

        ★ 但這個取捨在**窄轉角錄製三點轉向**時會反過來：那裡人做的每一次
        前進／後退本來就短（走廊淨寬 0.967 m），0.35 m 會把真正要保留的
        修正動作整段吃掉。所以門檻已參數化為 `merge_min_segment_m`，
        錄三點轉向前先降到 0.12~0.15。
        """
        if min_seg_m is None:
            # 現讀，不用 __init__ 的快取——否則 `ros2 param set` 無效（見宣告處）。
            min_seg_m = float(self.get_parameter("merge_min_segment_m").value)
        if len(pts) < 3:
            return pts, 0
        # 切成同方向的段
        segs = []
        start = 0
        for i in range(1, len(pts)):
            if pts[i].direction != pts[i - 1].direction:
                segs.append((start, i))
                start = i
        segs.append((start, len(pts)))

        def seg_len(a, b):
            return sum(math.hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y)
                       for i in range(a + 1, b))

        merged = 0
        for idx, (a, b) in enumerate(segs):
            if idx == 0:
                # 第一段沒有「前一段」可併，但它同樣可能是假的：_cmd_dir 的初值
                # 是 +1，所以「收到第一則 cmd_vel 之前」錄下的點一律被標成前進。
                # 倒車起步的路徑因此必定在索引 0 多出一個極短的前進段。
                # 實測（2026-08-04「大廳到起點」）：長度 0.00 m 的前進段，
                # 讓折返點從 2 個變成 3 個，重播時開頭會多一次無謂的停車換向。
                # 太短的第一段改採**下一段**的方向。
                if len(segs) > 1 and seg_len(a, b) < min_seg_m:
                    next_dir = pts[segs[1][0]].direction
                    for i in range(a, b):
                        pts[i] = PathPoint(pts[i].x, pts[i].y, pts[i].yaw, next_dir)
                    merged += 1
                continue
            if seg_len(a, b) < min_seg_m:
                prev_dir = pts[segs[idx - 1][0]].direction
                for i in range(a, b):
                    pts[i] = PathPoint(pts[i].x, pts[i].y, pts[i].yaw, prev_dir)
                merged += 1
        return pts, merged

    def _record_cb(self, req, resp):
        if req.action == RecordPath.Request.START:
            if self._following:
                resp.success = False
                resp.message = "重播進行中，不能同時錄製"
                return resp
            if self._robot_pose() is None:
                resp.success = False
                resp.message = f"取不到 {self.map_frame} -> {self.robot_frame} 的 TF，請先確認定位已就緒"
                return resp
            with self._rec_lock:
                self._rec_points = []
            self._rec_jumps = 0
            self._rec_abort_reason = ""
            self._recording = True
            self.get_logger().info("開始錄製教導路徑")
            resp.success = True
            resp.message = "開始錄製，請把車開一遍，完成後按結束"
            return resp

        if req.action == RecordPath.Request.CANCEL:
            self._recording = False
            with self._rec_lock:
                n = len(self._rec_points)
                self._rec_points = []
            resp.success = True
            resp.message = f"已取消錄製（丟棄 {n} 個點）"
            return resp

        if req.action != RecordPath.Request.STOP:
            resp.success = False
            resp.message = f"未知的動作代碼 {req.action}"
            return resp

        # ── STOP：存檔 ──
        if not self._recording:
            resp.success = False
            resp.message = (f"錄製已於中途中止：{self._rec_abort_reason}。請重新錄製"
                            if self._rec_abort_reason else "目前沒有在錄製")
            # 報過就清掉。否則之後任何一次「沒在錄製卻按了停止」都會重播
            # 這段舊訊息，操作者會以為剛剛那次也跳位了。
            self._rec_abort_reason = ""
            return resp
        self._recording = False
        with self._rec_lock:
            pts = list(self._rec_points)
            self._rec_points = []

        if len(pts) < 2:
            resp.success = False
            resp.message = f"只錄到 {len(pts)} 個點，路徑太短未存檔"
            return resp

        pts, merged = self._merge_short_segments(pts)
        if merged:
            self.get_logger().info(
                f"存檔前合併掉 {merged} 個短於 "
                f"{float(self.get_parameter('merge_min_segment_m').value):.2f} m "
                f"的方向段（視為假折返點）")

        # 存檔時就檢查可行性，不要等到重播才發現（2026-08-06）。
        # 教導路徑最容易出現車子做不到的段落 —— 人遙控時前輪可以刮地。
        # 這裡**只警告不阻擋**：路徑仍然存檔，因為你可能就是要留著看，
        # 或只想跑前面 90%。真正的攔截在重播開始時（見 _execute_follow）。
        feasible, feas_note, _ = self._check_feasibility(pts)
        if feasible:
            self.get_logger().info(feas_note)
        else:
            self.get_logger().warn(f"⚠ 這條路徑{feas_note}")
            self.get_logger().warn(
                "   重播到那一段會卡住。建議在該處分成兩次轉，或改用 "
                "/plan_taught_path 讓規劃器產生路徑（它對最小轉彎半徑是建構性保證）")

        name = (req.name or "").strip() or f"path_{len(self._paths) + 1}"
        pid = "path_" + uuid.uuid4().hex[:12]
        length = self._path_length(pts)
        cusps = self._count_cusps(pts)
        meta = {
            "name": name,
            "map_id": self.current_map,
            "source": "teach",
            "created_at": self._now_iso(),
            "length_m": length,
            "num_cusps": cusps,
        }
        with self._db_lock:
            self._paths[pid] = {"meta": meta, "points": pts}
            ok = self._save_db()
        if not ok:
            with self._db_lock:
                self._paths.pop(pid, None)
            resp.success = False
            resp.message = "寫入資料庫失敗"
            return resp

        self.get_logger().info(
            f"教導路徑已存檔「{name}」：{len(pts)} 點、{length:.2f} m、{cusps} 個折返點"
        )
        resp.success = True
        # （舊版這裡會附上「錄製中丟棄 N 個跳位點」。跳位改成**中止錄製**
        #   之後，能執行到這一行就代表全程沒跳過位，那段訊息恆為空字串，
        #   留著只會讓人以為還有「丟棄但繼續」的路徑。已移除。）
        resp.message = (f"已存檔「{name}」：{len(pts)} 點、{length:.2f} 公尺、"
                        f"{cusps} 個折返點")
        resp.path_id = pid
        resp.num_points = len(pts)
        resp.length_m = length
        return resp

    @staticmethod
    def _path_length(pts: List[PathPoint]) -> float:
        return sum(math.hypot(pts[i + 1].x - pts[i].x, pts[i + 1].y - pts[i].y)
                   for i in range(len(pts) - 1))

    @staticmethod
    def _count_cusps(pts: List[PathPoint]) -> int:
        return sum(1 for i in range(1, len(pts)) if pts[i].direction != pts[i - 1].direction)

    @staticmethod
    def _now_iso() -> str:
        return time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime())

    # ==================================================================
    # 列表 / 刪除
    # ==================================================================
    def _list_cb(self, req, resp):
        with self._db_lock:
            items = list(self._paths.items())
        out = []
        for pid, entry in items:
            meta = entry["meta"]
            if req.current_map_only and self.current_map and meta.get("map_id") != self.current_map:
                continue
            info = TaughtPathInfo()
            info.path_id = pid
            info.name = meta.get("name", "")
            info.map_id = meta.get("map_id", "")
            info.num_points = len(entry["points"])
            info.length_m = float(meta.get("length_m", 0.0))
            info.num_cusps = int(meta.get("num_cusps", 0))
            info.source = meta.get("source", "teach")
            info.created_at = meta.get("created_at", "")
            out.append(info)
        out.sort(key=lambda i: i.created_at)
        resp.success = True
        resp.paths = out
        resp.message = f"共 {len(out)} 條路徑"
        return resp

    def _delete_cb(self, req, resp):
        if self._following and req.path_id == self._active_path_id:
            resp.success = False
            resp.message = "這條路徑正在重播中，不能刪除"
            return resp
        with self._db_lock:
            if req.path_id not in self._paths:
                resp.success = False
                resp.message = f"找不到路徑 {req.path_id}"
                return resp
            name = self._paths[req.path_id]["meta"].get("name", req.path_id)
            removed = self._paths.pop(req.path_id)
            if not self._save_db():
                self._paths[req.path_id] = removed      # 寫檔失敗就還原
                resp.success = False
                resp.message = "寫入資料庫失敗，未刪除"
                return resp
        resp.success = True
        resp.message = f"已刪除路徑「{name}」"
        return resp

    # ==================================================================
    # 障礙偵測
    # ==================================================================
    def _forward_clearance(self, direction: int, lateral_offset: float = 0.0) -> float:
        """車子沿目前朝向前進（或後退）時，多遠會撞到東西

        把雷射點轉到車體座標系，只看寬度在 ±(半車寬) 內、位於行進方向那一側
        的點，回傳最近的縱向距離。lateral_offset 是「假設車子往側邊平移這麼多」
        時的結果，用來評估繞開可不可行。

        回傳 inf 代表這個方向淨空。
        """
        with self._scan_lock:
            scan = self._scan
        if scan is None:
            # 沒有雷達資料時回 0 而不是 inf：寧可誤停也不要盲衝
            return 0.0

        best = float("inf")
        ang = scan.angle_min
        inc = scan.angle_increment
        half_w = self.obs_half_w
        for r in scan.ranges:
            a = ang
            ang += inc
            if not (scan.range_min <= r <= scan.range_max) or r != r:  # 含 NaN
                continue
            # 車體座標：x 向前、y 向左（雷達與 base_footprint 同向，只差高度）
            x = r * math.cos(a)
            y = r * math.sin(a) - lateral_offset
            if abs(y) > half_w:
                continue
            longitudinal = x if direction >= 0 else -x
            if longitudinal <= 0.0:
                continue                      # 在身後，不管
            if longitudinal < best:
                best = longitudinal
        return best

    def _side_clearance(self) -> Tuple[float, float]:
        """車身**兩側**的橫向空隙 (左,右)，單位公尺。沒看到牆就回 inf。

        跟 `_forward_clearance` 不同：那個量的是「某條縱向帶裡最近的前方障礙」，
        這裡量的是「車身旁邊還剩多少」。走廊裡要保持餘裕靠的是後者。

        只取與車身縱向重疊的掃描點（前後各放寬 5 cm），因為遠處的牆
        不影響現在會不會刮到。
        """
        with self._scan_lock:
            scan = self._scan
        if scan is None:
            return 0.0, 0.0                     # 沒資料時當作貼牆，寧可保守
        front, rear, half_w = 0.40, -0.09, 0.185    # 車身矩形（base_footprint 在後輪軸心）
        left = right = float("inf")
        ang = scan.angle_min
        for r in scan.ranges:
            a = ang
            ang += scan.angle_increment
            if not (scan.range_min <= r <= scan.range_max) or r != r:
                continue
            if r > 3.0:
                continue
            x = self.laser_x + r * math.cos(a)
            y = r * math.sin(a)
            if not (rear - 0.05 <= x <= front + 0.05):
                continue
            if y > 0:
                left = min(left, y - half_w)
            else:
                right = min(right, -y - half_w)
        return left, right

    def _arc_clearance(self, direction: int, curvature: float, max_dist: float = 1.2,
                       lateral_offset: float = 0.0) -> float:
        """沿著「打舵之後實際會走的弧線」檢查淨空，而不是正前方的直帶。

        `_forward_clearance` 檢查的是以車頭為軸的矩形帶。脫困時車子打滿舵、
        走的是半徑 0.80 m 的弧，兩者差很多：**左前方明明有空間，
        直帶檢查卻只看正前方，於是判定「淨空不足」而放棄**（2026-08-03 實測）。

        做法：沿弧線每 5 cm 取一個車體中心位置，檢查該位置的車身矩形
        有沒有掃描點落在裡面。回傳第一次碰到障礙的弧長；全程淨空回傳 max_dist。

        這比直帶保守得多也精確得多——它問的是「車子照這個方向盤角度開，
        會不會撞到」，正是真正要回答的問題。
        """
        with self._scan_lock:
            scan = self._scan
        if scan is None:
            return 0.0
        pts = []
        ang = scan.angle_min
        for r in scan.ranges:
            a = ang
            ang += scan.angle_increment
            if not (scan.range_min <= r <= scan.range_max) or r != r:
                continue
            if r > max_dist + 1.0:
                continue
            # lateral_offset 正 = 假設車子往左平移（與 _forward_clearance、
            # _do_escape 同一個慣例）。把掃描點反向平移即等效。
            pts.append((r * math.cos(a), r * math.sin(a) - lateral_offset))
        if not pts:
            return max_dist

        half_w = self.obs_half_w
        # 車身縱向範圍（相對 base_footprint，雷達在其前方 0.089 m）
        laser_dx = 0.089
        front = 0.40 - laser_dx      # 車頭在雷達前方
        rear = -(0.09 + laser_dx)    # 車尾在雷達後方

        step = 0.05
        n = int(max_dist / step)
        for k in range(1, n + 1):
            arc = k * step
            # 沿弧線走 arc 之後，車體在雷達座標系的位姿
            #
            # ★ 2026-08-03 修正：倒車的符號原本是錯的（未實機驗證，見下）。
            #
            # 本節點所有發布端都是 ω = |v|·curvature：
            #   _do_escape 的 self._publish_cmd(v, abs(v) * curv)
            #   純追蹤的  w = abs(v) * curvature
            # 也就是**不論前進或後退，車頭都往同一邊轉**——阿克曼倒車時
            # 方向盤要打反邊，韌體的 Vz_to_Akm_Angle 會自己從 R = Vx/Vz
            # 算出該打哪邊（見 廠商原始碼分析_阿克曼控制鏈.md）。
            #
            # 所以正確的運動學是：
            #     朝向變化  dθ = +curvature · arc    與 direction 無關
            #     位移方向  才隨 direction 翻號
            #
            # 原本寫成 th = curvature·arc·(±1)、s_ = ±arc，那是
            # 「ω = v·curvature（隨方向翻號）」的有號弧長公式，與發布端不符。
            # 後果：倒車時檢查的是繞**另一側**的鏡像弧線。
            # 以 curv=1.25(R=0.8)、arc=0.4 m 代入，模型算出 cy=+0.098 m
            # 而實際是 −0.098 m，差 0.196 m —— 已經超過 obs_half_w=0.22
            # 的整個帶寬，等於在檢查一塊車子根本不會經過的區域。
            th = curvature * arc
            if abs(curvature) < 1e-6:
                cx, cy = arc * (1.0 if direction >= 0 else -1.0), 0.0
            else:
                R = 1.0 / curvature
                sgn = 1.0 if direction >= 0 else -1.0
                cx = sgn * R * math.sin(arc * curvature)
                cy = sgn * R * (1.0 - math.cos(arc * curvature))
            c, sn = math.cos(-th), math.sin(-th)
            for px, py in pts:
                # 把掃描點轉到「車子開到那裡之後」的車體座標
                dx, dy = px - cx, py - cy
                bx = dx * c - dy * sn
                by = dx * sn + dy * c
                if rear <= bx <= front and abs(by) <= half_w:
                    return arc
        return max_dist

    def _pick_avoid_offset(self, direction: int, curvature: float = 0.0) -> Optional[float]:
        """找一個能繞過去的橫向偏移量；找不到回 None

        從小到大試，左右交替（同樣大小優先試左邊只是為了行為一致，
        沒有偏好的理由）。可行的定義是「該偏移下前方淨空距離 > 停止距離 + 餘裕」。
        """
        # ★ 2026-08-06：解決端必須跟觸發端量同一件事 ★
        #
        # 8/04 把主迴圈的觸發改成 `_arc_clearance`（沿實際會走的弧線）並換算了
        # 門檻，卻沒有動這裡。於是變成：
        #   觸發：arc_clearance <= obs_stop − bumper      （自由行程，已換算）
        #   解決：_forward_clearance > obs_stop + 餘裕     （含車身長度，未換算）
        # 兩個條件量的東西不同、門檻也不同單位，所以挑出來的偏移**不保證能
        # 解決觸發它的那個問題** —— 觸發持續成立、狀態一直卡在 avoiding。
        # 而且直帶忽略曲率，選邊也可能選錯。
        #
        # 實測（path_44ce57013a51，大廳->起點）：677 筆裡 avoiding 佔 69.1%，
        # 車子整段貼著右牆走（右側 0.03 m、左側 0.57 m），平均偏離 25.99 cm。
        n = int(self.avoid_max / max(0.01, self.avoid_step))
        bumper = 0.40 if direction > 0 else 0.09
        stop_d = (self.obs_stop if direction > 0 else self.obs_stop_rev) - bumper
        for i in range(1, n + 1):
            for sign in (1.0, -1.0):
                off = sign * i * self.avoid_step
                if self._arc_clearance(direction, curvature, max_dist=self.obs_slow,
                                       lateral_offset=off) > stop_d + self.avoid_clear:
                    return off
        return None

    # ==================================================================
    # 純追蹤
    # ==================================================================
    @staticmethod
    def _closest_index(pts: List[PathPoint], x: float, y: float, hint: int) -> int:
        """從 hint 附近往後找最近點

        只往前搜尋不回頭，避免路徑自我交叉（例如折返、繞圈）時
        跳回已經走過的那一段。

        ★ 2026-08-05：搜尋範圍**不可跨越折返點** —— 與 `_lookahead_point`
        同一條規則。

        折返路徑會折回自己：實測 `path_3d85e56c550d`（nav2 規劃的三點轉向）
        的 pts[12] 與 pts[14] 只相距 **3.0 cm**、pts[13] 與 pts[15] 也是 3.0 cm
        —— 車子把同一段 0.15 m 來回走了三趟。而循跡誤差本身就有 3~9 cm，
        純 argmin 會直接跳到折返**之後**那一段，車子於是以為自己已經做完
        折返動作，實際上根本還沒退。索引跳掉之後主迴圈也不會偵測到換向。

        邊界含折返點本身（`limit = i + 1`）：主迴圈靠
        `pts[idx].direction != cur_dir` 偵測折返並停車換向，索引必須
        **到得了**那個點，否則會永遠停在它前面一格。
        """
        limit = min(len(pts), hint + 120)      # 一次最多往前看 120 點（約 6 公尺）
        cur_dir = pts[hint].direction
        for i in range(hint, limit):
            if pts[i].direction != cur_dir:
                limit = i + 1                  # 含折返點，讓主迴圈偵測得到
                break

        best_i, best_d = hint, float("inf")
        for i in range(hint, limit):
            d = (pts[i].x - x) ** 2 + (pts[i].y - y) ** 2
            if d < best_d:
                best_d, best_i = d, i
        return best_i

    def _lookahead_point(self, pts: List[PathPoint], start: int,
                         x: float, y: float, ld: float) -> Tuple[int, PathPoint]:
        """從 start 往前找第一個距離超過 ld 的點；同時不可跨越折返點

        跨越折返點會讓車子直接朝著「換方向之後的那一段」開過去，
        等於把折返整個跳掉 —— 這在窄走廊裡就是撞牆。
        """
        d0 = pts[start].direction
        i = start
        while i < len(pts) - 1:
            if pts[i].direction != d0:
                return i, pts[i]                       # 停在折返點
            if math.hypot(pts[i].x - x, pts[i].y - y) >= ld:
                return i, pts[i]
            i += 1
        return len(pts) - 1, pts[-1]

    def _pure_pursuit(self, pose, target: PathPoint, direction: int,
                      speed: float, lateral_offset: float) -> Tuple[float, float]:
        """回傳 (linear, angular)"""
        x, y, yaw = pose
        # 目標點轉到車體座標
        dx, dy = target.x - x, target.y - y
        c, s = math.cos(-yaw), math.sin(-yaw)
        bx = dx * c - dy * s
        by = dx * s + dy * c
        # ★ 2026-08-04 修正符號 ★
        # 車體座標 y 向左。`_forward_clearance(dir, +off)` 的檢查帶中心在
        # y = +off，也就是「假設車子往左移 off」；`_do_escape` 也照這個慣例
        # （left = _forward_clearance(d, +0.20)）。
        # 但這裡原本寫 `by -= lateral_offset`，正的偏移會讓目標看起來偏右、
        # 車子往右打 —— 跟量測端**符號相反**。於是 `_pick_avoid_offset`
        # 找到「往左有空間」，控制器卻往右轉，正好轉進障礙物。
        # 統一成「正 = 往左」。
        by += lateral_offset          # 繞障 / 貼牆排斥時把目標往側邊挪

        ld = math.hypot(bx, by)
        if ld < 1e-3:
            return 0.0, 0.0

        # 倒車時把目標鏡射到「車尾朝向」的座標系，公式才成立
        if direction < 0:
            bx, by = -bx, -by

        curvature = 2.0 * by / (ld * ld)
        # ★ 曲率上限分左右（2026-08-10）★
        #
        # 舊寫法 `max_curv = 1.0/min_radius` 對兩邊用同一個上限，於是左轉時
        # 會發出舵機根本到不了的曲率（左極限 0.944，右極限 0.751）。
        # 韌體不會報錯——它照算 AngleR，只是舵機轉不到那麼多，車子欠轉。
        # 結果就是「指令看起來對、車子卻一路往右漂」。
        #
        # 曲率為正 = 往左（CCW），所以正的用左半徑、負的用右半徑。
        curvature = self._clamp_curv(curvature, direction)

        v = speed if direction >= 0 else -speed
        w = abs(v) * curvature
        return v, w

    def _clamp_curv(self, curvature: float, direction: int = 1) -> float:
        """把曲率箝制在**該方向實際打哪邊**做得到的範圍內。

        ★★ 2026-08-10 修正：不能只看曲率符號，要看 `曲率 × 行進方向` ★★

        第一版寫成「正 = 往左」就結案，但那只在**前進**時成立。
        韌體 `Vz_to_Akm_Angle` 是從 **`R = Vx/Vz`** 算舵角的，而 `Vx` 帶號：

            前進 ω>0 → R=+0.800 → AngleR=+0.323 rad → **左打**
            倒車 ω>0 → R=−0.800 → AngleR=−0.467 rad → **右打**   ← 翻了

        （用廠商 `Axle_spacing=Wheel_spacing=0.322` 代入 `atan(0.322/(R+0.161))` 驗算過。）

        所以倒車時第一版會把「物理上的左打」（極限 0.944）用右邊的 0.80 去箝，
        發出舵機到不了的角度、韌體不報錯 → **靜默欠轉**；
        而「物理上的右打」（極限 0.751）被箝到 0.95，白白少用 21% 的能力。

        脫困的後退段、折返點之後的倒車段、HMI 的「反向重播」都會走到這裡。
        """
        left = (curvature >= 0.0) if direction >= 0 else (curvature <= 0.0)
        cap = 1.0 / max(0.05, self.min_radius_left if left else self.min_radius_right)
        return max(-cap, min(cap, curvature))

    def _publish_cmd(self, lin: float, ang: float) -> None:
        # 卡住判定要比對「指令 vs 實際」，所以指令值必須留下來（見 _follow_loop）
        self._cmd_v_last = lin
        t = Twist()
        t.linear.x = lin
        t.angular.z = ang
        self.cmd_pub.publish(t)

        # ★ 脫困期間額外走一條繞過 collision_monitor 的旁路（見 __init__ 的說明）。
        #   正常循跡時 _escape_bypass 是 False，這段不會執行。
        if self._escape_bypass and self.bypass_pub is not None:
            b = Twist()
            cap = self.escape_bypass_speed
            b.linear.x = max(-cap, min(cap, lin))
            # 角速度必須按同一個比例縮。阿克曼 ω = v·κ —— 只箝制 v 而讓 ω
            # 不變，等於要求更小的轉彎半徑；韌體照 R = Vx/Vz 換算舵角，
            # 會打得比 min_turning_radius 還緊（前輪刮地，正是教導路徑
            # 產生不可行段的那個現象）。按比例縮才能保住曲率。
            if abs(lin) > 1e-6:
                b.angular.z = ang * (abs(b.linear.x) / abs(lin))
            else:
                b.angular.z = 0.0
            self.bypass_pub.publish(b)

    def _stop(self, frames: int = 3) -> None:
        for _ in range(frames):
            self._publish_cmd(0.0, 0.0)
            time.sleep(0.02)

    def _publish_path_viz(self, pts: List[PathPoint]) -> None:
        msg = Path()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for pt in pts:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = pt.x
            ps.pose.position.y = pt.y
            qx, qy, qz, qw = quat_from_yaw(pt.yaw)
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    # ==================================================================
    # 重播主迴圈
    # ==================================================================
    def _execute_follow(self, goal_handle):
        req = goal_handle.request
        result = FollowTaughtPath.Result()

        with self._db_lock:
            entry = self._paths.get(req.path_id)
        if entry is None:
            goal_handle.abort()
            result.success = False
            result.message = f"找不到路徑 {req.path_id}"
            return result

        meta = entry["meta"]
        if self.current_map and meta.get("map_id") and meta["map_id"] != self.current_map:
            goal_handle.abort()
            result.success = False
            result.message = (f"路徑「{meta.get('name')}」屬於地圖 {meta['map_id']}，"
                              f"目前地圖是 {self.current_map}。座標系不同，不能重播")
            return result

        pts = list(entry["points"])
        if req.reverse:
            # 反向走：點序反轉，而且每個點的行進方向也要翻過來
            pts = [PathPoint(q.x, q.y, q.yaw, -q.direction) for q in reversed(pts)]

        # ── 可行性檢查（2026-08-06）──
        #
        # 攔在這裡，不要讓它跑到一半才發現。8/06 實測：教導路徑索引 44~49
        # 需要 0.688 m 的轉彎半徑（車子極限 0.80），重播到那裡卡住、
        # 七段脫困全滅、最後人工救車。**那是幾何限制，脫困救不回來。**
        #
        # 在 reverse 之後才檢查 —— 雖然反轉不改變朝向差的絕對值、
        # 所需半徑理論上相同，但檢查的應該是「實際要走的那條」。
        #
        # 注意這幾行必須在 self._following = True 之前：下面有 return 路徑，
        # 提早離開時那個旗標還沒設，不會留下「以為還在重播」的狀態。
        feasible, feas_note, _bad = self._check_feasibility(pts)
        if feasible:
            self.get_logger().info(feas_note)
        else:
            self.get_logger().warn(f"⚠ 「{meta.get('name')}」{feas_note}")
            if bool(self.get_parameter("refuse_infeasible_path").value):
                goal_handle.abort()
                result.success = False
                result.message = (f"{feas_note}。"
                                  "要強制執行請設 refuse_infeasible_path:=false")
                return result
            self.get_logger().warn(
                "   仍會執行（refuse_infeasible_path=false）。"
                "預期會卡在那一段 —— 那是幾何限制，不是控制問題，脫困救不回來")

        scale = req.speed_scale if req.speed_scale > 0.0 else 1.0
        self._following = True
        self._active_path_id = req.path_id
        self._publish_path_viz(pts)
        rev_note = "（反向）" if req.reverse else ""
        self.get_logger().info(
            f"開始重播「{meta.get('name')}」：{len(pts)} 點{rev_note}，速度 x{scale:.2f}"
        )

        try:
            return self._follow_loop(goal_handle, pts, scale, result)
        finally:
            self._following = False
            self._active_path_id = ""
            self._stop(5)

    def _follow_loop(self, goal_handle, pts, scale, result):
        idx = 0
        cur_dir = pts[0].direction
        wait_started = 0.0
        avoid_offset = 0.0
        state = "following"
        last_fb = 0.0
        escapes = 0
        sig_hist: List[Tuple[float, List[float]]] = []   # 打滑偵測用的掃描簽章歷史
        prog_idx = -1
        prog_t = time.monotonic()
        # 每個路徑點的累計弧長，給進度檢查用（見迴圈裡的說明）
        path_s = [0.0] * len(pts)
        for i in range(1, len(pts)):
            path_s[i] = path_s[i - 1] + math.hypot(pts[i].x - pts[i - 1].x,
                                                   pts[i].y - pts[i - 1].y)
        prog_s = -1.0
        self._escape_steer = None
        move_hist: List[Tuple[float, float, float, float]] = []   # (t, x, y, |指令v|)

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._stop()
                goal_handle.canceled()
                result.success = False
                result.message = "已取消"
                return result

            pose = self._robot_pose()
            if pose is None:
                self._stop()
                goal_handle.abort()
                result.success = False
                result.message = "定位遺失（取不到 map -> base_footprint）"
                return result
            x, y, yaw = pose

            # ── 進度 ──
            idx = self._closest_index(pts, x, y, idx)
            goal = pts[-1]
            dist_to_goal = math.hypot(goal.x - x, goal.y - y)
            # 到終點的判定要「索引也走到最後」才算，否則路徑起終點靠近時
            # 一出發就會被判定成已抵達
            if idx >= len(pts) - 3 and dist_to_goal <= self.goal_tol:
                self._stop(5)
                goal_handle.succeed()
                result.success = True
                result.final_error_m = dist_to_goal
                result.final_yaw_error_deg = math.degrees(abs(norm_angle(goal.yaw - yaw)))
                result.message = (f"已抵達終點，位置誤差 {dist_to_goal * 100:.0f} cm、"
                                  f"朝向誤差 {result.final_yaw_error_deg:.0f} 度")
                self.get_logger().info(result.message)
                return result

            # ── 偏離檢查 ──
            xte = math.hypot(pts[idx].x - x, pts[idx].y - y)
            if xte > self.max_xte:
                self._stop()
                goal_handle.abort()
                result.success = False
                result.message = (f"偏離路徑 {xte * 100:.0f} cm 超過上限 "
                                  f"{self.max_xte * 100:.0f} cm，已停車。"
                                  f"可能是定位跑掉或車子被推動")
                self.get_logger().warn(result.message)
                return result

            # ── 進度檢查：索引沒前進就當卡住 ──
            #
            # 不看狀態、只看有沒有在前進。頂到牆的時候淨空可能還沒低到
            # stop 門檻（狀態是 slowing 而不是 waiting），車子會用 30% 速度
            # 一直頂著，輪子打滑、位姿漂移，直到偏離上限才被攔下。
            # ★ 用「沿路徑走了多遠」而不是「索引變了沒」（2026-08-04）★
            #
            # 索引數是壞的進度指標，因為它跟點距綁在一起：
            #   教導路徑點距 5 cm  -> slowing 時 (30% 速度 ≈ 0.03 m/s) 1.7 秒走一格
            #   規劃路徑點距 15 cm -> 同樣情況要 5.0 秒走一格
            # 而 no_progress_sec 就是 5.0。實測 path_3d85e56c550d：
            # 障礙早就過了、車子也在動，卻每 5 秒誤判一次卡住，
            # 10 次脫困全部用在「正常但慢」的路段上，最後放棄在 25/51。
            # 同一個門檻換條路徑就變成永久誤觸發。
            #
            # 改量實際弧長：走不到 stuck_min_advance 才算卡住，與點距無關。
            # ★ 2026-08-06：判準改成「有下指令卻沒動」，不再看路徑推進速率 ★
            #
            # 前兩版都在量「沿路徑前進得多快」，那是**行進速率**，不是卡住：
            #   v1 看索引變化   -> 被點距背叛（5 cm vs 15 cm 差三倍）
            #   v2 看弧長推進   -> 點距無關了，但**慢仍然會被當成卡住**
            # 實測 path_2fb1081910d7：走廊裡前方淨空長期低於 obs_slow，
            # 速度被壓到 30%（0.03 m/s），第一次觸發脫困時偏離只有 3~6 cm
            # 而且很穩定——車子沒卡住，只是慢。結果 78% 的時間在脫困。
            #
            # 卡住的定義是**指令與實際脫節**：有下速度指令、車子卻沒動。
            # 慢慢開不是卡住。位移取自 map -> base_footprint（AMCL 修正過的
            # 對地位姿），不是輪速——輪子打滑時輪速會謊報「有在動」，
            # 而打滑正是我們見過的卡住模式之一。
            now_t = time.monotonic()
            move_hist.append((now_t, x, y, abs(self._cmd_v_last)))
            while move_hist and now_t - move_hist[0][0] > self.no_progress:
                move_hist.pop(0)
            stuck_now = False
            if len(move_hist) >= 2 and now_t - move_hist[0][0] >= self.no_progress * 0.8:
                span = now_t - move_hist[0][0]
                actual = math.hypot(x - move_hist[0][1], y - move_hist[0][2]) / max(1e-3, span)
                want = sum(h[3] for h in move_hist) / len(move_hist)
                # 有明確下指令，而實際速率不到指令的 stuck_speed_ratio
                stuck_now = (want > 0.02 and actual < self.stuck_speed_ratio * want)

            # ★ 2026-08-10 第三個判準：掃描沒變 = 車子真的沒動（打滑偵測）★
            #
            # 前兩個判準都建立在位姿上，而走廊裡位姿會被打滑的里程計拖著跑
            # （AMCL 沿長軸沒有分辨力，擋不住這個謊）。詳見 _scan_signature。
            # 這一條完全不看位姿，只問「雷射看到的世界有沒有變」。
            if not stuck_now and self.scan_still_m > 0:
                sig = self._scan_signature()
                if sig is not None:
                    sig_hist.append((now_t, sig))
                    while sig_hist and now_t - sig_hist[0][0] > self.no_progress:
                        sig_hist.pop(0)
                    if len(sig_hist) >= 2 and now_t - sig_hist[0][0] >= self.no_progress * 0.8:
                        d = self._sig_diff(sig_hist[0][1], sig)
                        want_v = sum(h[3] for h in move_hist) / max(1, len(move_hist))
                        if d is not None and want_v > 0.02 and d < self.scan_still_m:
                            self.get_logger().warn(
                                f"掃描 {now_t - sig_hist[0][0]:.1f} 秒內中位數只變了 {d*100:.1f} cm"
                                f"（門檻 {self.scan_still_m*100:.0f} cm），"
                                f"但一直在下指令（平均 {want_v:.2f} m/s）"
                                f" —— 判定輪子在打滑，車子其實沒動")
                            stuck_now = True

            # ★ 2026-08-10 補上第二個判準：「有在動，但沿路徑沒有推進」。
            #
            # 上面那個判準只問「有下指令卻沒動」。它治好了 v2 的誤觸發
            # （慢被當成卡住，78% -> 0%），但**也失去了偵測打轉/貼牆的能力** ——
            # 車子左右擺盪時實際位移跟得上指令，判準就認為它沒卡住。
            #
            # 2026-08-10 大廳→起點 實測：300 秒走到 47/48（最後一點），
            # 然後在 idx 47 原地擺盪 **116 秒**，偏離從 0 長到 60 cm 觸及上限中止，
            # 全程 `escaping` 只有 2 筆 —— 脫困根本沒被叫到。
            #
            # 兩個判準的症狀不同，缺一不可：
            #   有下指令卻沒動   -> 楔住（防撞死鎖那種）
            #   有在動但索引不動 -> 打轉、貼牆擺盪（這一種）
            #
            # 這裡用弧長而不是索引：索引會被點距背叛（v1 的教訓，5 cm 與 15 cm
            # 點距差三倍），弧長與點距無關。門檻取 stall_progress_m，
            # 預設 0.10 m —— 比 no_progress_sec 內正常前進距離小一個量級
            # （0.03 m/s × 5 s = 0.15 m），慢慢開不會誤判。
            if not stuck_now and self.stall_progress > 0:
                if path_s[idx] - prog_s < self.stall_progress:
                    if now_t - prog_t > self.no_progress:
                        stuck_now = True        # 有在動，但這段時間幾乎沒往前
                else:
                    prog_s = path_s[idx]        # 有實質推進，重新計起
                    prog_t = now_t

            if not stuck_now:
                prog_idx = idx
                prog_s = path_s[idx]
                prog_t = now_t
                self._escape_steer = None      # 真的在動，下次卡住重新判斷轉向
            elif time.monotonic() - prog_t > self.no_progress:
                if escapes < self.max_escapes:
                    escapes += 1
                    self.get_logger().warn(
                        f"有下速度指令但車子沒動（實際 < 指令的 "
                        f"{self.stuck_speed_ratio*100:.0f}%）已 {self.no_progress:.0f} 秒，"
                        f"索引 {idx}/{len(pts)}（或有在動但推進 < "
                        f"{self.stall_progress:.2f} m），判定卡住，"
                        f"嘗試脫困（第 {escapes}/{self.max_escapes} 次）")
                    self._stop(3)
                    if self._do_escape(pts, idx, cur_dir, goal_handle, escapes):
                        prog_t = time.monotonic()
                        avoid_offset = 0.0
                        wait_started = 0.0
                        state = "following"
                        move_hist.clear()   # 脫困期間主迴圈沒跑，視窗裡是舊樣本
                        sig_hist.clear()
                        continue
                    prog_t = time.monotonic()      # 退不動也重計時，讓它再試
                    move_hist.clear()
                    sig_hist.clear()
                else:
                    self._stop()
                    goal_handle.abort()
                    result.success = False
                    result.message = (f"卡在第 {idx}/{len(pts)} 點，"
                                      f"脫困 {escapes} 次都無效，已放棄")
                    self.get_logger().warn(result.message)
                    return result

            # ── 折返點：先停穩再換方向 ──
            if pts[idx].direction != cur_dir:
                self._stop(5)
                was = "前進" if cur_dir > 0 else "後退"
                now_dir = "前進" if pts[idx].direction > 0 else "後退"
                self.get_logger().info(f"折返點：{was} -> {now_dir}")
                time.sleep(self.cusp_pause)
                cur_dir = pts[idx].direction
                move_hist.clear()   # 折返是刻意停車，不是卡住
                sig_hist.clear()
                continue

            # ── 障礙 ──
            #
            # ★ 2026-08-04：改用弧線淨空，不再用「沿車頭的直帶」★
            #
            # 直帶檢查在窄走廊裡會把**側牆**當成正前方障礙：車身一斜，
            # 牆就落進 ±0.22 m 的帶子裡。8/3 算過「車頭偏 35 度以上會中招」，
            # 但那是 0.967 m 的走廊；實測 0.84 m 的走廊**只要歪 19 度就中招**。
            #
            # 實測（path_2fb1081910d7，逐筆 CSV）：車子與走廊軸線差 19 度，
            # 直帶量到「後方 0.60 m 有障礙」，那個點在車體 (-0.60,+0.22)
            # ——正好卡在帶子邊緣，地圖座標 (+2.71,+1.14) 是走廊中段的側牆。
            # 後果：永遠 slowing (30% 速度) -> 走一格 4.7 秒 -> 誤判卡住
            # -> 脫困把車轉得更斜 -> 更容易誤判。78% 的時間在脫困。
            #
            # `_arc_clearance` 沿「接下來實際會走的弧線」取樣車身矩形，
            # 車子斜著走時檢查的就是斜著的那條路。它本來只用在脫困裡。
            #
            # 先算一次純追蹤的曲率當預覽（下面真正下指令時會用當時的速度
            # 重算，兩者的曲率一致——曲率只跟目標點幾何有關，與速度無關）。
            _pv_ld = max(self.la_min, min(self.la_max, self.la_k * abs(
                self.follow_speed if cur_dir > 0 else self.reverse_speed) + self.la_min))
            _pv_i, _pv_tgt = self._lookahead_point(pts, idx, x, y, _pv_ld)
            _pv_v, _pv_w = self._pure_pursuit(pose, _pv_tgt, cur_dir, 1.0, avoid_offset)
            preview_curv = (_pv_w / abs(_pv_v)) if abs(_pv_v) > 1e-6 else 0.0
            # ★ 往前看的距離必須**大於**拿來比的門檻 ★
            # 第一版寫成 max_dist=self.obs_slow（1.20），但門檻是
            # slow_d = obs_slow - bumper = 1.11 —— 回傳值上限 1.20、
            # 「算清空」要 > 1.11，可用區間只有 9 公分，於是幾乎永遠 slowing。
            # 實測（path_07068062ebc0）686 筆裡 following 只有 1 筆。
            # 多看 0.30 m，讓「真的清空」有機會成立。
            clearance = self._arc_clearance(cur_dir, preview_curv,
                                            max_dist=self.obs_slow + 0.30)
            speed = (self.follow_speed if cur_dir > 0 else self.reverse_speed) * scale

            # ★ 換算門檻的單位 ★
            # `_forward_clearance` 回傳「從 base_footprint 原點到障礙的距離」，
            # 車身自己的長度**含在裡面**；`_arc_clearance` 回傳「還能走多遠才撞到」，
            # 車身矩形已經扣掉了。obs_stop / obs_slow 是照前者調出來的，
            # 直接拿來比會保守一倍以上（0.45 會變成「還有 45 cm 就停」）。
            # 前保險桿在 +0.40、後保險桿在 -0.09，扣掉即可。
            bumper = 0.40 if cur_dir > 0 else 0.09
            stop_d = (self.obs_stop if cur_dir > 0 else self.obs_stop_rev) - bumper
            slow_d = self.obs_slow - bumper

            if clearance <= stop_d:
                # 先試著繞開
                off = self._pick_avoid_offset(cur_dir, preview_curv)
                if off is not None and abs(off) <= self.avoid_max:
                    if state != "avoiding":
                        self.get_logger().info(f"前方障礙，橫向偏移 {off * 100:+.0f} cm 繞過")
                    avoid_offset = off
                    state = "avoiding"
                    speed *= 0.5
                    wait_started = 0.0
                else:
                    # 繞不過去 -> 停下等
                    self._stop()
                    if wait_started == 0.0:
                        wait_started = time.monotonic()
                        self.get_logger().info("前方障礙且無法繞過，停車等待")
                    waited = time.monotonic() - wait_started

                    # 等一下下還沒通 -> 試著脫困。
                    # 人擋路通常幾秒內就讓開，牆不會；兩者從單幀掃描分不出來，
                    # 所以用「等了多久」當判準。
                    if waited > self.escape_after and escapes < self.max_escapes:
                        escapes += 1
                        self.get_logger().info(
                            f"等了 {waited:.0f} 秒仍不通，嘗試脫困（第 {escapes}/{self.max_escapes} 次）")
                        _t_esc = time.monotonic()
                        if self._do_escape(pts, idx, cur_dir, goal_handle, escapes):
                            wait_started = 0.0
                            avoid_offset = 0.0
                            state = "following"
                            move_hist.clear()
                            sig_hist.clear()
                            continue
                        # ★ 2026-08-10：脫困花掉的時間不算在「等障礙移開」的頭上 ★
                        #
                        # wait_started 是在脫困**之前**起算的，而 _escape_legs 改成
                        # 連跑 3 段之後，楔死時最久要 3 x (5 直線 + 2.6 橫移 + 8 打舵)
                        # = 47 秒，遠超過 wait_timeout(20)。結果是下一圈必定 abort，
                        # max_escapes=10 實質只剩 1 次，而且錯誤訊息會把脫困耗時
                        # 記成「障礙持續 N 秒未移開」——現場會往「誰擋著」查，
                        # 但其實是自己的脫困把時鐘吃光了。
                        #
                        # 不改用調高 wait_timeout：那樣訊息仍然是錯的。
                        wait_started += time.monotonic() - _t_esc
                        waited = time.monotonic() - wait_started
                        # 退不動就繼續等，等滿 wait_timeout 再放棄

                    if waited > self.wait_timeout:
                        goal_handle.abort()
                        result.success = False
                        result.message = (
                            f"障礙持續 {waited:.0f} 秒未移開，已放棄"
                            f"（嘗試脫困 {escapes} 次）")
                        self.get_logger().warn(result.message)
                        return result
                    state = "waiting"
                    self._send_feedback(
                        goal_handle, idx, len(pts), xte, state,
                        f"等待障礙移開（{waited:.0f}/{self.wait_timeout:.0f} 秒，"
                        f"已脫困 {escapes} 次）")
                    time.sleep(self.control_dt)
                    continue
            else:
                wait_started = 0.0
                if clearance <= slow_d:
                    # 線性減速：距離越近越慢，最低到 30%
                    span = max(1e-3, slow_d - stop_d)
                    speed *= max(0.3, (clearance - stop_d) / span)
                    state = "slowing"
                else:
                    state = "following"
                    # 障礙消失就慢慢收回偏移，不要瞬間切回原路徑
                    if avoid_offset != 0.0:
                        avoid_offset *= 0.9
                        if abs(avoid_offset) < 0.01:
                            avoid_offset = 0.0

            # ── 貼牆排斥 ──
            #
            # 純追蹤只管追路徑點，路徑本身歪了就跟著歪。實測在 0.84 m 的
            # 走廊裡，車子右側只剩 0.07 m（車寬 0.37，理想是兩側各 0.235）。
            # 這裡在單側低於 wall_keepout 時把追蹤目標往外推，推的量正比於
            # 侵入深度、上限 wall_push_max。
            #
            # 刻意做成「靠太近才推」而不是「隨時往中線拉」：後者會跟路徑
            # 追蹤持續打架，而有些路徑本來就該貼一邊走。這裡只保餘裕。
            # 符號慣例：正 = 往左（與 _forward_clearance / _do_escape 一致）。
            sl, sr = self._side_clearance()
            wall_push = 0.0
            if sr < self.wall_keepout and sr < sl:
                wall_push = min(self.wall_push_max, self.wall_keepout - sr)      # 往左推
            elif sl < self.wall_keepout and sl < sr:
                wall_push = -min(self.wall_push_max, self.wall_keepout - sl)     # 往右推
            if wall_push != 0.0 and state == "following":
                state = "slowing"
                speed *= 0.6
                if int(time.monotonic() * 2) % 4 == 0:
                    self.get_logger().info(
                        f"貼牆（左 {sl:.2f} m、右 {sr:.2f} m），"
                        f"目標往{'左' if wall_push > 0 else '右'}推 {abs(wall_push)*100:.0f} cm")

            # ── 純追蹤 ──
            ld = max(self.la_min, min(self.la_max, self.la_k * abs(speed) + self.la_min))
            _tgt_i, target = self._lookahead_point(pts, idx, x, y, ld)
            total_off = avoid_offset + wall_push
            lin, ang = self._pure_pursuit(pose, target, cur_dir, abs(speed), total_off)

            # ★ 把橫向偏移的組成放進 feedback（2026-08-06）★
            # 實測車子在走廊裡一路貼右牆（右 0.03 / 左 0.56）而偏離持續變大，
            # 但從外面分不出是「排斥沒算出來」「被繞障抵消」還是「被曲率箝制吃掉」。
            # 這三個對應完全不同的修法。看不見就查不了——脫困失敗隱形了三天，
            # 補一行 feedback 就真相大白（見 _do_escape 的兩個失敗出口）。
            if abs(total_off) > 0.005 or state in ("avoiding", "slowing"):
                # ★ 飽和判定要用**該方向**的上限（2026-08-10）。
                #   用對稱的 min_radius 會在左轉時低估飽和程度：左邊真正的
                #   上限是 1/0.95 而不是 1/0.80，本來已經飽和的會被標成沒飽和。
                #   這一行是明天要撈的關鍵診斷，判錯等於白跑一趟。
                # ★ 倒車時 lin<0，同一個 ang 打的是反邊（見 _clamp_curv）
                _lft = (ang * lin >= 0)
                r_dir = self.min_radius_left if _lft else self.min_radius_right
                max_curv = 1.0 / max(0.05, r_dir)
                sat = "★飽和" if abs(ang) >= abs(lin) * max_curv * 0.98 else ""
                turn_side = "左" if ang >= 0 else "右"
                dbg = (f"側 左{sl:.2f}/右{sr:.2f} 排斥{wall_push:+.2f} "
                       f"繞障{avoid_offset:+.2f} 合計{total_off:+.2f} "
                       f"曲率{sat}(轉{turn_side}/上限{max_curv:.2f})")
                self._send_feedback(goal_handle, idx, len(pts), xte, state, dbg)
            self._publish_cmd(lin, ang)

            now = time.monotonic()
            if now - last_fb > 0.25:
                last_fb = now
                self._send_feedback(goal_handle, idx, len(pts), xte, state, "")
            time.sleep(self.control_dt)

        self._stop()
        goal_handle.abort()
        result.success = False
        result.message = "節點關閉"
        return result

    def _do_escape(self, pts, idx, cur_dir, goal_handle, attempt: int = 1) -> bool:
        """脫困的外層：開啟防撞旁路，並保證無論從哪個出口離開都會關掉。

        `_do_escape_inner` 有十幾個 return 點（淨空被擋、移動超標、取消、
        逾時…），逐一在每個出口關旁路遲早會漏掉一個 —— 漏掉的後果是
        **正常循跡也在繞過 collision_monitor**，那比原本的死鎖危險得多。
        用 try/finally 把它變成不可能漏。

        離開前主動送一筆零速到旁路：底盤的指令逾時是 1.0 秒，不送的話
        車子會多滑一秒才停。
        """
        self._escape_bypass = True
        try:
            return self._escape_legs(pts, idx, cur_dir, goal_handle, attempt)
        finally:
            self._escape_bypass = False
            if self.bypass_pub is not None:
                self.bypass_pub.publish(Twist())

    def _escape_legs(self, pts, idx, cur_dir, goal_handle, attempt: int) -> bool:
        """★ 2026-08-10：一次呼叫做**連續多段**三點轉向，中間不還給純追蹤。

        === 為什麼要連起來 ===

        使用者現場示範的掉頭訣竅是：
            「方向打滿 -> 前後反向打 -> 多次 -> **需要開啟避障，
              不然每次都自己卡住還在做動**」

        最後那句是關鍵。原本的結構是「一次 `_do_escape` = 一段」，做完就
        `return`，主迴圈把控制權交回**純追蹤**，等它再卡住才呼叫下一次。
        兩個後果：

        1. **段與段之間沒有避障回饋**。純追蹤只追路徑、不看牆，它會把
           剛轉出來的角度又帶回去，甚至直接把車推回夾點。
        2. **防撞旁路只在 `_do_escape` 期間開**（那正是它該有的行為），
           所以段間那幾秒是 collision_monitor 說了算——車子貼牆時它會
           停止輸出，於是「還在做動卻不會動」。

        三點轉向的本質是**一連串動作的累積效果**，把它拆開執行等於沒做。
        所以改成：一次呼叫裡連跑 `escape_legs_max` 段，旁路全程開著，
        而每一段內部本來就有 `_side_clearance` / `_arc_clearance` 檢查
        ——那就是使用者要的「開啟避障」，只是它現在真的全程有效了。

        === 為什麼不是無限制地做下去 ===

        每段之間重新量朝向，累積轉夠 `escape_total_dyaw` 就收手：
        繼續轉只會把車推出可用空間。若某一段完全沒轉到（< 2 度），
        代表被擋死了，再做也是一樣，直接停——這比做滿次數快很多，
        也避免 8/04 那種「10 段轉了 250 度、淨變化 +9 度」的極限環。
        """
        p0 = self._robot_pose()
        yaw_start = p0[2] if p0 is not None else None
        legs = max(1, int(self.escape_legs_max))
        ok_any = False

        for leg in range(legs):
            if goal_handle.is_cancel_requested:
                return ok_any
            # attempt 的奇偶決定前進/後退，所以 +leg 就是自動交替
            ok = self._do_escape_inner(pts, idx, cur_dir, goal_handle, attempt + leg)
            ok_any = ok_any or ok

            p = self._robot_pose()
            if p is None or yaw_start is None:
                return ok_any
            total = abs(norm_angle(p[2] - yaw_start))

            if total >= self.escape_total_dyaw:
                self.get_logger().info(
                    f"連續脫困完成：{leg + 1} 段共轉了 "
                    f"{math.degrees(total):.0f} 度（目標 "
                    f"{math.degrees(self.escape_total_dyaw):.0f} 度）")
                return True

            # 這一段幾乎沒轉到 -> 被擋死了，再做也一樣
            if leg > 0 and not ok:
                p_prev_ok = getattr(self, "_escape_leg_yaw", None)
                if p_prev_ok is not None and abs(norm_angle(p[2] - p_prev_ok)) < math.radians(2.0):
                    self.get_logger().warn(
                        f"第 {leg + 1} 段幾乎沒轉到（< 2 度），停止連續脫困"
                        f"（累計 {math.degrees(total):.0f} 度）")
                    return ok_any
            self._escape_leg_yaw = p[2]

        p = self._robot_pose()
        if p is not None and yaw_start is not None:
            self.get_logger().info(
                f"連續脫困做滿 {legs} 段，累計轉了 "
                f"{math.degrees(abs(norm_angle(p[2] - yaw_start))):.0f} 度")
        return ok_any

    def _do_escape_inner(self, pts, idx, cur_dir, goal_handle, attempt: int = 1) -> bool:
        """卡住時的脫困：打舵前進改變車頭角度，不是直線退回去。

        === 為什麼要打舵（2026-08-03 實測後重寫）===

        第一版是「沿路徑往回退」——用純追蹤去追路徑上前面幾個點。
        那些點大致就在車子正後方，算出來的曲率接近零，等於**直線退**。
        實測連退三次、每次 0.38 m，車子姿態完全沒變，退完再進還是同樣
        卡在轉角。三次都失敗。

        阿克曼車卡在窄轉角時，問題從來不是「位置不對」而是**車頭角度不對**。
        直線進退不會改變角度，只有「打舵移動」才會。人開車過窄彎時做的
        三點轉向就是這件事：往一邊打舵前進、再往另一邊打舵後退，
        每一次都把車頭轉一點，直到角度足夠通過。

        === 往哪邊打 ===

        比較左右兩側的淨空，往寬的那邊打。用即時掃描判斷而不是寫死，
        因為卡住的方位每次都不一樣（實測是右後輪貼牆，但左後也可能）。

        曲率直接用物理極限 1/min_radius（打滿舵）——脫困要的就是
        最大的角度變化率，沒有理由留餘裕。

        回傳 True 代表車頭角度確實變了、值得重試。
        """
        # === 交替方向（2026-08-03 二次修正）===
        # 原本每次都往同一個方向脫困，等於「退、退、退」——那不是三點轉向。
        # 真正的三點轉向是**前進打左 -> 後退打右 -> 前進打左**：
        # 每一段都把車頭往同一個方向多轉一點，位置卻大致留在原地。
        # 單向重複只會把車子愈推愈遠，車頭角度卻不會累積。
        #
        # attempt 從 1 開始：奇數次往行進的反向、偶數次往行進方向。
        #
        # ★ 打舵方向**不跟著翻**（2026-08-03 修正；舊註解寫「也跟著翻」是錯的，
        #   下面的 steer 只由 yaw_err 決定，與 attempt / back_dir 完全無關）。
        #   阿克曼運動學 dθ/dt = v·tan(δ)/L：前進要 CCW 就 δ 往左、後退要 CCW
        #   就 δ 往右——兩段的方向盤位置相反，但**指令角速度同號**，韌體的
        #   Vz_to_Akm_Angle 會自己從 R = Vx/Vz 算出該打哪邊。
        #   翻了反而是把剛轉過來的角度又轉回去（實測：前進轉 20 度、
        #   後退只轉 17→14→8 度）。
        back_dir = -cur_dir if attempt % 2 == 1 else cur_dir

        # ★ 2026-08-10：改成「空間在哪就往哪」，而不是從路徑方向推 ★
        #
        # 上面那行只看 cur_dir（路徑的行進方向）與 attempt 的奇偶，
        # **完全沒有問過空間在哪一端**。2026-08-10 實測的後果：
        # 車子車尾楔進門框（左右車尾角各剩 1~2 cm），前方 0.63 m、後方 0.34 m，
        # 而它決定「先直線**後退**拉開距離」—— 往夾點的方向走，當然退不動。
        #
        # 同一時間使用者手動遙控脫困，錄下來的三段**全部是前進**
        # （0.01 m 試探、0.03 m 試探、2.08 m 開出去），餘隙 0.012 -> 0.368 m。
        # 人做的事很簡單：往空的那邊走。
        #
        # ★ 使用者的提醒很重要：「每次卡住都不太相同」。
        #   所以能寫進程式的**不是動作序列，是選法** ——
        #   每次都重新量前後淨空再決定，這對各種卡法都成立。
        #
        # 這與今天另外兩個修正是同一個病：用間接的量代替直接的量
        # （轉向只看朝向差不看牆、地圖端與實測端量不同東西）。
        if self.escape_dir_by_space:
            f_clear = self._arc_clearance(+1, 0.0, max_dist=1.5)
            r_clear = self._arc_clearance(-1, 0.0, max_dist=1.5)
            # 差距要夠明顯才推翻預設，免得在兩邊差不多時反覆橫跳
            if abs(f_clear - r_clear) >= self.escape_dir_margin:
                want = 1 if f_clear > r_clear else -1
                if want != back_dir:
                    self.get_logger().info(
                        f"脫困方向改由空間決定：前 {f_clear:.2f} m / 後 {r_clear:.2f} m"
                        f"，改成{'前進' if want > 0 else '後退'}"
                        f"（原本依路徑方向要{'前進' if back_dir > 0 else '後退'}）")
                back_dir = want

        start = self._robot_pose()
        if start is None:
            return False
        x0, y0, yaw0 = start

        # === 該往哪邊轉：看路徑要求的朝向，不是看哪邊比較空 ===
        #
        # 第一版用「左右淨空誰大」決定。那是錯的判準——脫困的目的是
        # **把車頭轉到能沿路徑繼續走的角度**，跟哪邊有空間無關。
        # 實測 log「左側淨空 0.91 m、右側 0.00 m」所以選了左邊，
        # 但使用者現場看到的是「該往右打」——選反了，於是愈蹭愈糟。
        #
        # 正確判準：比對車頭與路徑在前方幾個點的朝向差，往差的方向轉。
        # 前視 5 個點（約 25 cm）而不是當前點，因為要對齊的是「接下來
        # 要走的方向」而不是「現在站的位置」。
        look = min(idx + 5, len(pts) - 1)
        yaw_err = norm_angle(pts[look].yaw - yaw0)
        # 倒車路徑上，車頭朝向與行進方向相反，但**路徑點記的就是車頭朝向**，
        # 所以不需要額外處理——直接對齊即可。
        left = self._forward_clearance(back_dir, +0.20)
        right = self._forward_clearance(back_dir, -0.20)
        # ★ 不要翻角速度的符號 ★
        #
        # 阿克曼運動學：dθ/dt = v·tan(δ)/L。要讓車頭持續往同一邊轉：
        #     前進 (v>0) 要 CCW -> δ 往左 -> ω = v·tan(δ)/L > 0
        #     後退 (v<0) 要 CCW -> δ 往右 -> ω = (負)·(負)/L > 0   <- ω 仍為正
        # 兩段的**方向盤位置相反**（一左一右），但**指令角速度同號**。
        # 韌體的 Vz_to_Akm_Angle 會自己從 R = Vx/Vz 算出該打哪邊。
        #
        # 第一版額外翻了 ω 的符號，等於把後退那段轉回去，兩段互相抵消。
        # 實測「前進轉 20 度、後退只轉 17→14→8 度」就是這樣來的：
        # 車子在原地打轉、位置卻一點一點往牆邊漂。
        # 而且我當時記的是 abs(dyaw)，轉回去和轉過去印出來一樣，
        # 儀器本身掩蓋了錯誤——負面結果比正面結果更需要懷疑量測方式。
        # ω 的正負號 = 想要的旋轉方向（CCW 為正）。前進與後退**同號**，
        # 韌體的 Vz_to_Akm_Angle 會自己把它換算成左右相反的方向盤角度。
        steer = 1.0 if yaw_err > 0 else -1.0
        # 若朝向差很小（已經對齊了），那卡住的原因不是角度而是位置，
        # 這時才退而用淨空決定往哪邊挪
        if abs(yaw_err) < math.radians(5.0):
            steer = 1.0 if left >= right else -1.0

        # ★ 2026-08-10 貼牆否決權 ★
        #
        # 上面只看「路徑朝向差」，完全沒看**車子正貼著哪面牆**。
        # 於是貼右牆時，只要朝向差說要往右轉，它就往右轉——直接轉進牆裡。
        # 使用者實測看到的就是這個：「他退後的方向打錯邊了」。
        #
        # 朝向差是「想去哪」，牆是「不能去哪」。不能去的優先。
        # 只有真的很貼（< wall_veto_m）才否決，否則會干擾正常的朝向修正。
        veto = self.wall_veto_m
        if veto > 0:
            if right < veto and left > right:
                if steer < 0:
                    self.get_logger().warn(
                        f"貼牆否決：右側只剩 {right:.2f} m，"
                        f"原本要往順時針（右）轉會撞牆，改成逆時針")
                steer = 1.0
            elif left < veto and right > left:
                if steer > 0:
                    self.get_logger().warn(
                        f"貼牆否決：左側只剩 {left:.2f} m，"
                        f"原本要往逆時針（左）轉會撞牆，改成順時針")
                steer = -1.0

        # ★ 整串脫困只決定一次轉向，之後沿用（2026-08-04）★
        #
        # 上面那個判準本身是對的，但把它放在「每次嘗試都重算」的迴圈裡，
        # 它會自我推翻：脫困把車頭轉過去 -> 路徑朝向差變號 -> 下一次就往回轉。
        #
        # 實測（規劃路徑 path_3d85e56c550d，逐筆 CSV）：
        #     前進段  -8, -23, -24, -23, -26 度
        #     後退段 -14, +30, +32, +34, +31 度
        #     10 段脫困總共轉了約 250 度，車頭淨變化 +9 度。
        # 從第 3 段起是完美的極限環，每一對淨賺 7 度，然後永遠重複。
        #
        # 這是 8/3「v3 兩段互相抵消」換了個形式回來：那次是角速度符號寫錯，
        # 這次符號對了，但判準被放在一個會自我推翻的迴圈裡。
        # ★ 判準對不對，跟它被呼叫的時機對不對，是兩件事。
        if attempt > 1 and self._escape_steer is not None:
            if steer != self._escape_steer:
                self.get_logger().info(
                    f"沿用第一次脫困決定的轉向（本次算出的是"
                    f"{'逆時針' if steer > 0 else '順時針'}，已忽略）")
            steer = self._escape_steer
        self._escape_steer = steer
        side = "左" if steer > 0 else "右"
        turn_dir = "逆時針" if steer > 0 else "順時針"
        self.get_logger().info(
            f"脫困：{'前進' if back_dir > 0 else '後退'}、車頭往{turn_dir}轉"
            f"（路徑朝向差 {math.degrees(yaw_err):+.0f} 度；"
            f"左側淨空 {left:.2f} m、右側 {right:.2f} m）"
        )

        speed = (self.follow_speed if back_dir > 0 else self.reverse_speed) * 0.6
        # 打滿舵 —— 但「滿」在左右是不同的量（左 0.95 / 右 0.80）。
        # 用對稱值的話，往左脫困時會發出舵機到不了的曲率：指令看起來更急，
        # 車子卻轉得一樣多，於是「已轉角度」的判定會一直不達標而空轉。
        curv = steer / max(0.05, self.min_radius_left
                                 if steer * back_dir > 0 else self.min_radius_right)
        # 20 -> 35 度（2026-08-03）。每段轉得多，需要的來回次數就少。
        # 上限來自空間：0.80 m 轉彎半徑下轉 35 度，車子會前進約
        # 0.80 * 0.61 = 0.49 m 的弧長，走廊寬度撐得住。
        # 轉太多的風險是「衝出可用空間」，所以不會一次設到 90 度——
        # 淨空檢查隨時會提前中止，那時已轉的量仍然算數（部分成功）。
        target_dyaw = math.radians(35.0)

        # ★ 貼牆時先直線退出來，退到有空間了才打方向（2026-08-04）★
        #
        # 打滿舵的弧線第一件事是把車尾往外掃。車身側面已經貼著牆的時候，
        # 那一掃就是往牆裡送——所以 8/3 會「愈蹭愈糟」最後楔死。
        # 使用者現場的判斷是對的：「要直線後退才能打方向」，順序不能顛倒。
        #
        # 這裡不打舵、只沿 back_dir 直線走，直到兩側都有 side_min 的空間
        # 或走滿 escape_dist。走直線時車身掃掠的寬度就是車寬本身，
        # 是所有動作裡最小的，貼牆時唯一安全的選擇。
        # ★ 2026-08-06 修正閘門用錯量測 ★
        # 原本用 `left/right = _forward_clearance(back_dir, ±0.20)` 當判準，
        # 但那回傳的是「把檢查帶往側邊平移之後，**前方**最近障礙有多遠」
        # ——縱向距離，不是車身兩側的空隙。轉角處前方開闊，兩個值都很大，
        # 於是 `min < 0.15` 永遠不成立，**這整段直線脫出從來沒被執行過**。
        # 實測（path_2702ea459c8b 索引 49 楔住）：真正的側向空隙是
        # 左 +0.08 / 右 −0.09 m，而 _forward_clearance 那兩個值遠大於 0.15。
        # `_side_clearance()` 才是量車身旁邊還剩多少的函式（8/04 就寫好了，
        # 給貼牆排斥用，只是當時忘了接到這個閘門上）。
        side_min = 0.15
        s_left, s_right = self._side_clearance()
        if min(s_left, s_right) < side_min:
            self.get_logger().info(
                f"側向太窄（左 {s_left:.2f} m、右 {s_right:.2f} m），先直線"
                f"{'前進' if back_dir > 0 else '後退'}拉開距離再打方向")
            ts = time.monotonic()
            while time.monotonic() - ts < 5.0 and rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self._stop()
                    return False
                p = self._robot_pose()
                if p is None:
                    self._stop()
                    return False
                if math.hypot(p[0] - x0, p[1] - y0) >= self.escape_dist:
                    break
                l2, r2 = self._side_clearance()
                if min(l2, r2) >= side_min:
                    break
                if self._arc_clearance(back_dir, 0.0) <= 0.12:
                    self.get_logger().warn("直線方向也被擋住，無法拉開距離")
                    break
                v0 = (self.follow_speed if back_dir > 0 else self.reverse_speed) * 0.5
                self._publish_cmd(v0 if back_dir > 0 else -v0, 0.0)
                self._send_feedback(goal_handle, idx, len(pts), 0.0, "escaping",
                                    f"脫困中（直線{'前進' if back_dir > 0 else '後退'}拉開距離）")
                time.sleep(self.control_dt)
            self._stop(3)
            # 拉開之後車子位置變了，轉向的基準要重新取
            p = self._robot_pose()
            if p is not None:
                x0, y0, yaw0 = p

        # ── 橫移置中（2026-08-10，依使用者現場描述實作）────────────────
        #
        # 使用者的配方：「往前打左（走多一點）→ 往前打右（走少一點）修正姿態
        # 置中走廊 → 再往後退（微幅修左右）」。
        #
        # 為什麼需要這一段：前面的直線階段只能沿車身方向拉開，**不會改變
        # 車子在走廊裡的橫向位置**；而打舵階段是在原位轉頭。兩者都解不了
        # 「整台車貼在右牆上」。這一段是阿克曼版的橫移——兩段反向弧的
        # 淨效果是側移，朝向回到原值。
        #
        # 第二段刻意走得比第一段短（centre_back_ratio）：
        # 完全等長會把側移抵消掉一部分，短一點才留得住位移，
        # 而剩下的朝向差就交給後面的打舵階段。
        if self.centre_enable:
            s_left, s_right = self._side_clearance()
            near = min(s_left, s_right)
            if near < self.centre_trigger_m:
                away = 1.0 if s_left >= s_right else -1.0     # 往空的那邊
                side_txt = "左" if away > 0 else "右"
                self.get_logger().info(
                    f"橫移置中：左 {s_left:.2f} / 右 {s_right:.2f} m，"
                    f"往{side_txt}橫移（前進打{side_txt} -> 回打反向）")
                v_c = self.follow_speed * 0.5
                for leg, (sgn, dur) in enumerate((
                        (away, self.centre_leg_sec),
                        (-away, self.centre_leg_sec * self.centre_back_ratio))):
                    # ★ 每一段用**該方向**做得到的曲率（2026-08-10）。
                    #   兩段方向相反，共用一個 w_c 的話一定有一邊超出能力：
                    #   往左那段會被舵機物理欠轉，於是側移量比預期小，
                    #   而「回打」那段卻是足量的 —— 淨效果反而往反方向偏。
                    curv_c = self._clamp_curv(sgn / max(0.05, self.min_radius_left
                                                        if sgn > 0 else self.min_radius_right))
                    w_c = v_c * abs(curv_c)
                    tl = time.monotonic()
                    while time.monotonic() - tl < dur and rclpy.ok():
                        if goal_handle.is_cancel_requested:
                            self._stop(); return False
                        # 前方被擋就立刻收手——這一段一律往前走
                        if self._arc_clearance(1, curv_c) <= 0.25:
                            self.get_logger().warn("橫移置中：前方不足，中止這一段")
                            break
                        l3, r3 = self._side_clearance()
                        if min(l3, r3) < 0.02:          # 真的要碰到了
                            self.get_logger().warn(
                                f"橫移置中：側向剩 {min(l3, r3):.2f} m，中止")
                            break
                        self._publish_cmd(v_c, sgn * w_c)
                        self._send_feedback(
                            goal_handle, idx, len(pts), 0.0, "escaping",
                            f"橫移置中（第 {leg+1}/2 段，前進打"
                            f"{'左' if sgn > 0 else '右'}）左{l3:.2f}/右{r3:.2f}")
                        time.sleep(self.control_dt)
                    self._stop(2)
                l4, r4 = self._side_clearance()
                self.get_logger().info(
                    f"橫移置中完成：左 {s_left:.2f}->{l4:.2f}、"
                    f"右 {s_right:.2f}->{r4:.2f} m")
                p = self._robot_pose()
                if p is not None:
                    x0, y0, yaw0 = p

        t0 = time.monotonic()
        while time.monotonic() - t0 < 8.0 and rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._stop()
                return False
            pose = self._robot_pose()
            if pose is None:
                self._stop()
                return False
            x, y, yaw = pose
            dyaw_signed = norm_angle(yaw - yaw0)     # 有號：看得出是累積還是抵消
            dyaw = abs(dyaw_signed)
            moved = math.hypot(x - x0, y - y0)

            if dyaw >= target_dyaw:
                self._stop(3)
                self.get_logger().info(
                    f"脫困完成：車頭轉了 {math.degrees(dyaw_signed):+.0f} 度、移動 {moved:.2f} m")
                return True

            # 脫困方向也要看淨空，不能往牆裡開
            # 沿實際會走的弧線檢查，不是正前方的直帶（見 _arc_clearance）
            # ★ 2026-08-05：以下兩個失敗出口都必須先發一筆 feedback ★
            #
            # 它們原本直接 return，而第一筆 `escaping` 的 _send_feedback 在
            # 迴圈更下面（見下方）。第一圈就被擋的話 —— 貼牆時正是如此 ——
            # **一筆 escaping 都不會發出去**，HMI 與 CSV 的 state 停在
            # `following`，看起來就像「進度檢查根本沒觸發」。
            #
            # 8/03 記錄的「索引 66 卡 40 秒沒觸發脫困、原因未明」極可能就是
            # 這個觀測缺口：主迴圈拿到 False 之後會重設 prog_t「讓它再試」，
            # 於是每 5 秒無聲重來一次，10 次才 abort（約 50 秒），
            # 而使用者在 40 秒（約 8 次）就人工取消了 —— 連 abort 訊息都沒出現。
            #
            # 這不是脫困的 bug，是**看不見脫困失敗**的 bug。
            # 修好觀測之後，下次同樣的現象才查得下去。
            if self._arc_clearance(back_dir, curv) <= 0.12:
                self._stop(3)
                self.get_logger().warn(
                    f"脫困弧線上有障礙（轉了 {math.degrees(dyaw_signed):+.0f} 度就停）")
                self._send_feedback(
                    goal_handle, idx, len(pts), 0.0, "escaping",
                    f"脫困第 {attempt} 次：弧線被擋，只轉了 "
                    f"{math.degrees(dyaw_signed):+.0f} 度")
                # 有轉到一點就算部分成功，值得重試
                return dyaw > math.radians(5.0)
            if moved > self.escape_dist * 3.0:
                self._stop(3)
                self.get_logger().warn(f"已移動 {moved:.2f} m 但只轉了 {math.degrees(dyaw):.0f} 度")
                self._send_feedback(
                    goal_handle, idx, len(pts), 0.0, "escaping",
                    f"脫困第 {attempt} 次：移動 {moved:.2f} m 但只轉了 "
                    f"{math.degrees(dyaw):.0f} 度，放棄這次")
                return dyaw > math.radians(5.0)

            v = speed if back_dir > 0 else -speed
            self._publish_cmd(v, abs(v) * curv)
            self._send_feedback(goal_handle, idx, len(pts), 0.0, "escaping",
                                f"脫困中（{'前進' if back_dir > 0 else '後退'}），已轉 {math.degrees(dyaw_signed):+.0f}/35 度")
            time.sleep(self.control_dt)

        self._stop(3)
        return False

    def _send_feedback(self, goal_handle, idx, total, xte, state, message) -> None:
        fb = FollowTaughtPath.Feedback()
        fb.point_index = idx
        fb.total_points = total
        fb.progress = idx / max(1, total - 1)
        fb.cross_track_error_m = xte
        fb.state = state
        fb.message = message
        goal_handle.publish_feedback(fb)



    # ==================================================================
    # 地圖點選 -> 用 nav2 規劃器產生教導路徑
    # ==================================================================
    def _plan_cb(self, req, resp):
        """把使用者在地圖上點的一串位置，規劃成一條可存檔的教導路徑

        不自己做曲線內插：SmacPlannerHybrid + REEDS_SHEPP 本來就會產生
        符合最小轉彎半徑、含折返點的阿克曼可行路徑。規劃器本身沒問題，
        出問題的是 MPPI 控制器 —— 所以「用 nav2 規劃、用純追蹤執行」
        是最省事也最可靠的組合。
        """
        if self._following or self._recording:
            resp.success = False
            resp.message = "錄製或重播進行中，請先結束"
            return resp

        targets = list(req.waypoints)
        if not targets:
            resp.success = False
            resp.message = "沒有給任何位置"
            return resp

        # 起點
        if req.start_from_robot:
            pose = self._robot_pose()
            if pose is None:
                resp.success = False
                resp.message = "取不到車子目前位置，無法從車子開始規劃"
                return resp
            start = self._make_pose(pose[0], pose[1], pose[2])
            # 標記「起點＝車子現在位置」，_plan_leg 據此改用 use_start=False
            self._live_start = start
        else:
            if len(targets) < 2:
                resp.success = False
                resp.message = "不從車子開始的話至少要兩個點"
                return resp
            start = targets.pop(0)

        if not self._planner_client.wait_for_server(timeout_sec=5.0):
            resp.success = False
            resp.message = "等不到 /compute_path_to_pose 動作伺服器，nav2 有啟動嗎？"
            return resp

        # ★ 規劃前先清一次全域成本地圖 —— 見 __init__ 裡的說明。
        # 失敗不擋下規劃：清空是最佳化不是前提。
        self._clear_global_costmap()

        all_pts: List[PathPoint] = []
        cur = start
        for leg, tgt in enumerate(targets, 1):
            seg = self._plan_leg(cur, tgt)
            if seg is None:
                resp.success = False
                resp.message = f"第 {leg} 段規劃失敗（{len(targets)} 段中）。目標可能落在障礙物或膨脹區裡"
                return resp
            # 相鄰段的接點會重複，去掉後面那段的第一個點
            all_pts.extend(seg[1:] if all_pts else seg)
            cur = tgt

        if len(all_pts) < 2:
            resp.success = False
            resp.message = "規劃結果太短"
            return resp

        self._annotate_directions(all_pts)

        name = (req.name or "").strip() or f"planned_{len(self._paths) + 1}"
        pid = "path_" + uuid.uuid4().hex[:12]
        length = self._path_length(all_pts)
        cusps = self._count_cusps(all_pts)
        meta = {
            "name": name,
            "map_id": self.current_map,
            "source": "plan",
            "created_at": self._now_iso(),
            "length_m": length,
            "num_cusps": cusps,
        }
        with self._db_lock:
            self._paths[pid] = {"meta": meta, "points": all_pts}
            ok = self._save_db()
        if not ok:
            with self._db_lock:
                self._paths.pop(pid, None)
            resp.success = False
            resp.message = "寫入資料庫失敗"
            return resp

        self._publish_path_viz(all_pts)
        # 規劃器對 minimum_turning_radius 是建構性保證，這裡檢查是為了**驗證
        # 那個保證**，順便確認門檻沒設得太嚴。8/06 用 0.80 當門檻時，
        # 規劃器的正常輸出（實測 0.79872，差 0.2%）被誤報成不可行 —— 那次
        # 誤報讓人以為規劃器壞了。門檻改 0.76 之後對照組才分得開。
        feasible, feas_note, _ = self._check_feasibility(all_pts)
        if feasible:
            self.get_logger().info(feas_note)
        else:
            self.get_logger().warn(
                f"⚠ 規劃器產出的路徑{feas_note}\n"
                "   規劃器本應保證這件事 —— 先確認 SmacPlannerHybrid 的 "
                "minimum_turning_radius 沒有被改小，再看門檻是不是設太嚴")
        self.get_logger().info(
            f"規劃路徑已存檔「{name}」：{len(all_pts)} 點、{length:.2f} m、{cusps} 個折返點"
        )
        resp.success = True
        resp.message = (f"已存檔「{name}」：{len(all_pts)} 點、{length:.2f} 公尺、"
                        f"{cusps} 個折返點")
        resp.path_id = pid
        resp.num_points = len(all_pts)
        resp.length_m = length
        resp.num_cusps = cusps
        return resp

    def _make_pose(self, x: float, y: float, yaw: float) -> Pose:
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        qx, qy, qz, qw = quat_from_yaw(yaw)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    def _clear_global_costmap(self, timeout: float = 3.0) -> bool:
        """清空全域成本地圖的 obstacle 層，回傳是否成功。

        **失敗不該擋下規劃。** 清空是最佳化不是前提——服務不在
        （nav2 還沒起來、或改用別的堆疊）時照樣讓規劃跑，
        讓規劃器自己回報真正的錯誤，不要在這裡多一個失敗點。

        用 `_spin_until` 輪詢而不是 `spin_until_future_complete`：
        這是在服務回呼裡呼叫的，後者會在同一個 executor 上死鎖。
        """
        if not self._clear_costmap_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                "清空成本地圖的服務不在，跳過（規劃仍會進行）。"
                "若規劃失敗且訊息說「目標可能落在障礙物或膨脹區裡」，手動清一次：\n"
                "  ros2 service call /global_costmap/clear_entirely_global_costmap "
                "nav2_msgs/srv/ClearEntireCostmap '{}'")
            return False
        future = self._clear_costmap_client.call_async(ClearEntireCostmap.Request())
        if not self._spin_until(future, timeout):
            self.get_logger().warn("清空成本地圖逾時，仍繼續規劃")
            return False
        self.get_logger().info("已清空全域成本地圖的 obstacle 層")
        return True

    def _plan_leg(self, start: Pose, goal: Pose) -> Optional[List[PathPoint]]:
        """呼叫 nav2 規劃一段，回傳路徑點；失敗回 None

        用同步等待而不是 callback：這是服務回呼，本來就允許阻塞，
        而且節點用的是 MultiThreadedExecutor，不會卡住其他回呼。
        """
        req = ComputePathToPose.Goal()
        # ★ 2026-08-10：起點就是車子現在位置時，改用 use_start=False ★
        #
        # 原本一律 use_start=True，把 self._robot_pose() 明確餵進去。實測
        #（車子在 0.99 m 走廊中段，定位 100% 吻合、車頭淨空差 4.6 cm）：
        #     use_start=True   -> 任何目標都失敗，連正前方 1 m 也失敗
        #     use_start=False  -> 同一位置同一目標，status=4、8 個路徑點
        # 明確餵進去的起點會被 nav2 驗證，車子貼近膨脹邊界時判 START_OCCUPIED；
        # 讓 nav2 自己查 TF 就通過。
        # ★ 只有第一段能這樣做——第二段之後的起點是上一段終點，車子還沒到，
        #   那時必須明確餵。
        use_live = (self._live_start is not None and start is self._live_start)
        req.use_start = not use_live
        if not use_live:
            req.start = self._stamped(start)
        req.goal = self._stamped(goal)

        send_future = self._planner_client.send_goal_async(req)
        if not self._spin_until(send_future, 10.0):
            self.get_logger().error("送出規劃請求逾時")
            return None
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("規劃請求被拒絕")
            return None

        result_future = handle.get_result_async()
        if not self._spin_until(result_future, 20.0):
            self.get_logger().error("等待規劃結果逾時")
            return None
        wrapped = result_future.result()
        if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            return None

        poses = wrapped.result.path.poses
        if len(poses) < 2:
            return None
        return [PathPoint(ps.pose.position.x, ps.pose.position.y,
                          yaw_from_quat(ps.pose.orientation), 1) for ps in poses]

    def _stamped(self, pose: Pose) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = self.map_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose = pose
        return ps

    @staticmethod
    def _spin_until(future, timeout: float) -> bool:
        """等 future 完成。不呼叫 spin_until_future_complete —— 這裡已經在
        執行器的回呼裡，再要求執行器 spin 會遞迴進去死鎖。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if future.done():
                return True
            time.sleep(0.02)
        return False

    def _annotate_directions(self, pts: List[PathPoint]) -> None:
        """規劃器回傳的路徑沒有標行進方向，這裡從幾何反推

        Reeds-Shepp 會產生含折返的路徑：某一點的位移向量與該點朝向反向，
        就代表那一段是倒車。判定用位移在朝向上的投影，接近 0 時沿用前一點
        （避免原地小幅修正被誤判成折返）。
        """
        for i in range(len(pts) - 1):
            dx = pts[i + 1].x - pts[i].x
            dy = pts[i + 1].y - pts[i].y
            forward = dx * math.cos(pts[i].yaw) + dy * math.sin(pts[i].yaw)
            if abs(forward) > 1e-3:
                pts[i].direction = 1 if forward > 0 else -1
            elif i > 0:
                pts[i].direction = pts[i - 1].direction
        if len(pts) >= 2:
            pts[-1].direction = pts[-2].direction


def main(args=None):
    rclpy.init(args=args)
    node = PathTeachNode()
    # 動作的執行回呼裡有 sleep 迴圈，必須用多執行緒執行器，
    # 否則同一條執行緒被佔住，服務與取消請求都進不來
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
