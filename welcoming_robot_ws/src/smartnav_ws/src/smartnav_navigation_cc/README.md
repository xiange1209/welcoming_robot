# smartnav_navigation_cc

wheeltec **senior_akm**（奧克曼常規型）阿克曼底盤的建圖與導航功能包，ROS 2 Jazzy / Raspberry Pi 4。

這是 `smartnav_navigation` 的重寫版本，**不修改也不依賴原本的套件**，兩者可以並存在同一個
workspace 裡，但**同一時間只能啟動其中一套**（對外服務名稱相同）。

---

## 1. 舊版為什麼不能用

以下都是從 `~/maprun/logs/nav2.log` 與 `brain.log` 直接讀出來的：

| 現象 | 根因 |
| --- | --- |
| `planner_server` 連噴 542 次 `GridBased plugin failed to plan ...: "Start occupied"`，`bt_navigator` 538 次 `Goal failed` | 機器人跑到 global costmap 範圍外。log 裡先後出現三種不同的地圖邊界：`(-5.00,-5.00)~(4.98,4.98)`（`empty_map`）、`(-11.58,-3.40)~(-1.16,9.87)`（某張存檔的 partial map）。`static_layer` 跟著 `/map` 換來換去 |
| `Robot is out of bounds of the costmap` | 同上 |
| `create_map` 永遠回 `地圖建立超時，已自動停止探索` | 探索完成的判定是訂閱 `/rosout` 比對字串 `"Exploration finished"`，而那行 log 只在 `return_to_start_on_complete=true` 的分支才印得出來 |
| `behavior_server: Exceeded time allowance before reaching the Spin goal` | 阿克曼車不能原地旋轉，但恢復行為裡有 `Spin`，每次都走到逾時才失敗，白白吃掉重試次數 |
| 建完圖之後導航就壞掉 | `map_service_node` 建完圖只把 `map_server` 拉回 active，**`slam_toolbox` 從來沒有被 deactivate**。於是 `/map` 有兩個發布者、`map -> odom_combined` 也有兩個發布者 |
| 開機後第一次導航一定被拒絕，回「當前定位不夠準確，請嘗試全域定位」 | `navigation_action_node` 的 `current_covariance_norm` 初始值是 `inf`，而 AMCL 要移動超過 `update_min_d` 才會發第一則 `/amcl_pose` |
| 全域定位時車子完全不動 | `waypoint_service_node` 發的是 `geometry_msgs/TwistStamped`，但 `wheeltec_robot_node` 訂閱的是 `geometry_msgs/Twist`（`wheeltec_robot.cpp: Cmd_Vel_Callback`），型別不符話題配不起來。而且指令內容是原地旋轉（`linear.x=0, angular.z=0.4`），阿克曼車本來就做不到 |

---

## 2. 新架構

### 2.1 模式互斥（最重要的一條）

`map -> odom_combined` 這條 TF 與 `/map` 這個話題，**任何時刻都只有一個發布者**：

```
建圖模式 (mapping)                    定位模式 (localization)
  slam_toolbox  = active                map_server   = active
  map_server    = unconfigured          amcl         = active
  amcl          = unconfigured          slam_toolbox = unconfigured
```

四個生命週期節點（`amcl` / `map_server` / `map_saver` / `slam_toolbox`）**只由
`map_service_cc_node` 一個節點操作**。其他節點要切模式必須呼叫 `/ensure_localization`，
不可以自己去打 `/amcl/change_state`（舊版 `map_service` 和 `waypoint_service` 兩邊都在切，
是很難查的競態來源）。

退場一律退到 `unconfigured` 而不是停在 `inactive`：`inactive` 的 `map_server` 其
transient_local publisher 還活著，舊地圖會繼續留在 `/map` 上。

### 2.2 開機順序

nav2 的 `lifecycle_manager` 用 `autostart:=false`。`global_costmap` 在 activate 時會等
`map -> base_footprint`，那條 TF 要等模式決定好才會出現。所以流程是：

```
map_service_cc 決定模式 → 模式就緒（TF 有了） → 呼叫 /lifecycle_manager_navigation_cc/manage_nodes(STARTUP)
```

這樣就沒有開機競態，也不需要靠 `initial_transform_timeout` 硬等。

### 2.3 阿克曼專屬調整

| 項目 | 值 | 理由 |
| --- | --- | --- |
| footprint | `[-0.22, 0.24] × [±0.185]`，padding 0.02 | 車身 0.46 × 0.37 m，`base_footprint` 在兩軸中間 |
| inflation_radius | **0.22** | 略大於內接半徑 0.205。0.30 以上會讓 0.99 m 走廊的可行帶只剩 0.4 m（詳見第 8 節） |
| 規劃器 | `SmacPlannerHybrid` + `REEDS_SHEPP` | 可倒車，不用 Dubins |
| `minimum_turning_radius` | 0.80 | 底盤物理極限，原廠 `param_senior_akm.yaml` 的值 |
| 控制器 | MPPI `motion_model: Ackermann` | `PathAngleCritic.mode: 2` 允許倒車朝向 |
| 恢復行為 | 清 costmap → BackUp → Wait | **拿掉 Spin**，改用自訂行為樹 `navigate_to_pose_akm_cc.xml` |
| `yaw_goal_tolerance` | 0.5 | 阿克曼車無法原地修正朝向 |
| 全域定位 | 0.9 m 半徑的圓弧繞行 | 不是原地旋轉；前方太近就倒車換邊。**注意：0.99 m 窄走廊裡會撞牆，該用不需移動的 `/align_pose`** |
| frontier 距離門檻 | 1.2 m | 轉彎直徑 1.6 m，比這近的目標多半規劃不出來 |
| `default_server_timeout` | **1000** | 單位是毫秒。預設 20 ms 在 Pi 4 上必定逾時，是導航一直 ABORT 的真因 |
| `max_beams` (AMCL) | **240** | 雷達每圈 450 點，預設 60 只用了 13% 的資料 |

### 2.4 探索完成的判定

改用 `frontier_explorer` 正式發布的 `/exploration_complete`（`std_msgs/Empty`）。
這個話題是 **transient_local**，訂閱當下就會補送上一輪的舊事件，所以 `map_service_cc`
用 `_exploration_epoch` 過濾，只認「本輪開始之後」收到的事件。

---

## 3. 對外介面

**沿用舊名稱，HMI 與 `smartnav_brain` 不需要任何修改。**

| 介面 | 型別 | 說明 |
| --- | --- | --- |
| `/create_map` | action `smartnav_msgs/CreateMap` | 建圖（自動探索或手動遙控） |
| `/list_maps` | service `smartnav_msgs/ListMaps` | 列出地圖 |
| `/switch_map` | service `smartnav_msgs/SwitchMap` | 切換地圖 |
| `/current_map` | topic `std_msgs/String`（transient_local） | 目前地圖 ID |
| `/navigate` | action `smartnav_msgs/Navigate` | 導航 |
| `/create_waypoint` `/list_waypoints` `/get_waypoint` | service | 地點管理 |
| `/global_localization` | action `smartnav_msgs/GlobalLocalization` | 全域定位 |

新增的輔助介面：

| 介面 | 型別 | 說明 |
| --- | --- | --- |
| `/finish_map` | `std_srvs/Trigger` | 手動結束建圖並存檔 |
| `/ensure_localization` | `std_srvs/Trigger` | 確保切回 AMCL 定位模式 |
| `/get_nav_mode` | `std_srvs/Trigger` | 查詢目前模式（`message` 欄位回 mapping/localization） |
| `/align_pose` | `std_srvs/Trigger` | 用雷射與地圖重新對齊位姿，**不需移動車子**。導航前會自動呼叫 |
| `/start_exploration` | `std_srvs/Trigger` | 建圖中手動啟動自動探索（配合 `auto_start_exploration:=false`，用於「先遙控過窄走廊、進房間再自動探索」） |
| `/recalibrate_imu_bias` | `std_srvs/Trigger` | 重新校準 IMU 零偏（車子須靜止） |
| `/measure_steering_trim` | `std_srvs/Trigger` | 重新量測轉向零位偏移（車子會前進約 0.5 m） |
| `/camera_standby` `/camera_resume` `/camera_status` | `std_srvs/Trigger` | 相機分階段開關 |

節點名稱都有 `_cc` 後綴：`map_service_cc_node`、`waypoint_service_cc_node`、
`navigation_action_cc_node`。

---

## 4. 用法

### 4.1 前置

```bash
~/maprun/run_sensors_cc.sh          # 等同 ros2 launch turn_on_wheeltec_robot wheeltec_sensors.launch.py
```

底盤、雷達、EKF、robot_state_publisher 必須先起來。**沒有 `/scan` 的話，
`slam_toolbox` 雖然會 activate 卻永遠不會發布 `map -> odom_combined`**，
`map_service_cc` 會在 60 秒後報：

```
slam_toolbox 已 active 但等不到 map -> base_footprint
```

確認方式：

```bash
ros2 topic info /scan          # Publisher count 必須是 1
ros2 run tf2_ros tf2_echo odom_combined base_footprint
```

> **`~/cyclonedds.xml` 不可以刪。** `~/maprun/env.sh` 把 `CYCLONEDDS_URI` 指向它，
> 檔案不見時 CycloneDDS 會在建立 domain 時直接失敗，整串節點全部 SIGABRT：
> `can't open configuration file` → `rmw_create_node: failed to create domain`。
> `run_nav_cc.sh` / `run_sensors_cc.sh` 已經加了防呆，檔案不在就退回預設設定並印警告。

### 4.2 一般啟動（導航）

```bash
~/maprun/run_nav_cc.sh            # auto：有地圖就定位，沒地圖就進建圖待命
~/maprun/run_nav_cc.sh mapping    # 強制建圖模式
```

或直接：

```bash
ros2 launch smartnav_navigation_cc nav_bringup_cc.launch.py start_mode:=auto
```

### 4.3 自動探索建圖

```bash
ros2 launch smartnav_navigation_cc mapping_explore_cc.launch.py
~/maprun/create_map_cc.sh 我的地圖
```

探索完成會自動存檔並切回定位模式。想提前收工：

```bash
ros2 service call /finish_map std_srvs/srv/Trigger
```

### 4.4 沒有底座時的模擬測試

底座沒接（或人不在車旁）時，用假機器人取代 `run_sensors_cc.sh`：

```bash
~/maprun/run_fake_robot_cc.sh        # 終端機 1：虛擬底盤 + 虛擬雷射
~/maprun/run_nav_cc.sh mapping false # 終端機 2：照常啟動導航
```

假機器人提供 `/scan`、`/odom_combined` 與 `odom_combined → base_footprint → laser`
的 TF，虛擬房間 10×8 m（含隔間、1.6 m 的門、家具）。純 2D 幾何，不需要 Gazebo，
Pi 4 跑得動。

**阿克曼運動學是照實模擬的**，不是差速積分：

```
|ω| <= |v| / 0.8      # 最小轉彎半徑 0.8 m
v = 0  =>  ω = 0      # 無法原地旋轉
```

實測：送 `v=0, ω=0.5` 三秒 → 角度變化 **0.00°**；送 `v=0.3, ω=1.0` → ω 被限制到
0.375 rad/s。這一點是刻意的 —— 如果模擬器用差速積分，舊版那些原地旋轉的動作
（全域定位、`Spin` 恢復行為）會「假裝成功」，反而測不出問題。

### 4.5 手動遙控建圖（最穩，不經過 nav2）

```bash
ros2 launch smartnav_navigation_cc mapping_manual_cc.launch.py
# 另一個終端機
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# 存檔
ros2 run nav2_map_server map_saver_cli -f ~/.smartnav/map_database/data/<map_id>
```

阿克曼車不能原地轉：`j`/`l` 只有在同時有前進或後退速度時才會轉向。

要讓手動建的圖進到 SmartNav 的地圖清單，也可以改走 `nav_bringup_cc` +
`use_exploration:=false`，這樣 `/create_map` 會等你遙控走完再呼叫 `/finish_map`，
存檔與資料庫登錄都會自動完成。

---

## 5. 檔案

```
config/nav2_senior_akm_cc.yaml   Nav2 + AMCL + map_server 參數
config/slam_mapping_cc.yaml      slam_toolbox 建圖參數
config/frontier_explore_cc.yaml  自動探索參數
behavior_trees/navigate_to_pose_akm_cc.xml   阿克曼行為樹（無 Spin）
launch/nav_bringup_cc.launch.py       總啟動
launch/mapping_manual_cc.launch.py    手動遙控建圖
launch/mapping_explore_cc.launch.py   自動探索建圖
smartnav_navigation_cc/lifecycle_helper_cc.py    生命週期狀態圖導航
smartnav_navigation_cc/map_service_cc_node.py    地圖服務 / 模式仲裁
smartnav_navigation_cc/waypoint_service_cc_node.py  地點服務 / 全域定位
smartnav_navigation_cc/navigation_action_cc_node.py 導航動作
smartnav_navigation_cc/imu_bias_corrector_cc_node.py IMU 零偏補償（必要，見第 7 節）
smartnav_navigation_cc/steering_trim_cc_node.py      轉向零位補償（必要，見第 7 節）
smartnav_navigation_cc/camera_manager_cc_node.py     相機分階段管理
smartnav_navigation_cc/fake_robot_cc_node.py         無底座時的虛擬機器人
launch/sensors_cc.launch.py           底盤+雷達+IMU補償（取代原廠 wheeltec_sensors）
behavior_trees/navigate_through_poses_akm_cc.xml     兩棵樹都要換，否則 bringup 會掛
```

---

## 6. 排錯

```bash
ros2 service call /get_nav_mode std_srvs/srv/Trigger     # 目前是哪個模式
ros2 topic info /map --verbose                            # /map 應該只有 1 個發布者
ros2 run tf2_ros tf2_echo map odom_combined               # 應該只有一個來源在更新
ros2 lifecycle get /slam_toolbox                          # 定位模式下應為 unconfigured
ros2 lifecycle get /amcl                                  # 建圖模式下應為 unconfigured
```

`/map` 出現兩個發布者，或 `slam_toolbox` 與 `amcl` 同時 active，就是模式仲裁出問題了 —— 那正是舊版壞掉的樣子。

---

## 7. 這台車特有的硬體補償（2026-07-28 實測後加入）

這兩項不是可選的優化，是**這台車不做就無法正常導航**的補償。

### 7.1 IMU 陀螺儀零偏 —— `imu_bias_corrector_cc`

車子完全靜止時 z 軸角速度是 `+0.00132 rad/s`（= **+4.54 度/分鐘**）。
這個零偏被 EKF 積分進 `odom` 的 yaw，靜置五分鐘就偏 23 度，
「車頭前方一公尺」會被算到牆裡，規劃器直接回 `no valid path`。

**關鍵陷阱**：只扣掉角速度零偏**完全沒有效果**。原廠 `ekf.yaml` 的
`imu0_config` 第 6 個是 `true`，EKF 同時把 IMU 的**絕對 yaw** 當測量融合，
而那個 yaw 是 IMU 內部積分出來的、本身就含累積漂移。補償因此被繞過
（實測補償前後都是 60 秒漂約 5 度，一模一樣）。

完整解法在 `sensors_cc.launch.py`，**不動原廠設定檔**：

```
/imu/data_raw → [imu_bias_corrector_cc 扣零偏] → /imu/data_unbiased → EKF
                                                  + imu0_config[5]=false
```

實測結果：**60 秒漂 4.74 度 → 90 秒漂 0.024 度**（約 300 倍改善）。

> ★ 啟動時車子必須完全靜止，否則會把真實旋轉當成零偏記下來。
> 重新校準：`ros2 service call /recalibrate_imu_bias std_srvs/srv/Trigger`

### 7.2 轉向零位偏移 —— `steering_trim_cc`

下 `angular.z = 0`（要求直行）車子仍持續左偏。校準值 `-0.072 rad/m`。

**校準順序很重要**：必須在 IMU 零偏修好之後量。第一次量到 +6.54 度/m
（混進了感測器漂移），補 -0.105 結果補過頭 —— 原本偏左 0.076 m 變成偏右 0.123 m。
IMU 修好後重測兩次得到 +4.25 / +4.00 度/m，一致性才夠好。

接線刻意放在碰撞檢查**之前**：

```
velocity_smoother → cmd_vel_smoothed → [steering_trim_cc] → cmd_vel_trimmed
                                     → collision_monitor → cmd_vel → 底盤
```

補償量設錯也不會繞過安全層。重新校準：`/measure_steering_trim`（車子會前進約 0.5 m）。

---

## 8. 窄走廊的物理極限

```
走廊淨寬      0.99 m
最小轉彎直徑  1.60 m   ← 比走廊寬 60%
```

**車子在這種走廊裡無法掉頭，只能直線前進與倒車。** 這不是參數問題。

連帶影響（都已調整）：

| 參數 | 值 | 理由 |
| --- | --- | --- |
| `inflation_radius` | 0.22 | 0.30 時可行帶只剩 0.4 m，explorer 挑的目標全落在膨脹區被規劃器拒絕 |
| `occ_threshold` (explorer) | 75 | 60 太嚴（一啟動就說沒有 frontier，實際有 338 格）、90 太鬆（挑到規劃器拒絕的點） |
| `reverse_penalty` | 1.3 | 2.0 時規劃器寧可繞遠路也不肯倒車，但窄處掉頭只能靠來回切換 |
| `max_angular_speed_wmax` | 0.35 | 讓 explorer 的評分偏好「車頭方向就到得了」的目標 |

**180 度對稱歧義**：長走廊前後看起來一樣，全域 scan match 會找到兩個
100% 吻合、相距 0.46 m 的解，方向也可能整個反過來。
`_align_pose_with_scan` 已加「修正量超過 45 度就拒絕」的保護，
但擋不住 AMCL 一開始就處在翻轉狀態 —— 那種情況只能靠人看 HMI 的箭頭確認。

---

## 9. 相機分階段管理 —— `camera_manager_cc`

相機相關行程實測吃掉約 **110% CPU**（`astra_camera_node` 92% + `republish` 17%），
是導航失敗的主因（load average 一度衝到 52，正常應 < 4）。

```
待機 / 迎賓辨識  → 相機開
建圖中           → 相機關
導航中           → 相機關（訂閱 /navigation_active 自動切換）
導航結束         → 相機開
```

手動控制：`/camera_standby`、`/camera_resume`、`/camera_status`

感測器啟動時相機預設就是關的：

```bash
~/maprun/run_sensors_cc.sh          # 不含相機
~/maprun/run_sensors_cc.sh true     # 含相機
```
