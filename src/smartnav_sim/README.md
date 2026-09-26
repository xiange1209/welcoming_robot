# smartnav_sim —— 自動建圖模擬教學

在筆電的 WSL 裡模擬我們那台 senior_akm。導航鏈跟實車是**同一套**：同一支 launch、同一份參數檔。
**在這裡調好的參數，直接就是車上要用的參數**，這也是做這個模擬的目的。

> 只在筆電跑，不上車（`pack_car_code.py` 已排除這個套件）。安裝見 [`setup/README.md`](setup/README.md)。

**目錄**：[1. 第一次跑](#1-第一次跑10-分鐘) · [2. 看懂畫面與 log](#2-看懂畫面與-log) · [3. 做實驗的方法](#3-做實驗的方法) ·
[4. 參數：調了會怎樣](#4-參數調了會怎樣) · [5. 建議的實驗](#5-建議先做的實驗) · [6. 模擬了什麼](#6-模擬了什麼沒模擬什麼) · [7. 疑難排解](#7-疑難排解)

---

## 1. 第一次跑（10 分鐘）

**① 開 Ubuntu**：開始功能表 →「Ubuntu 24.04」。之後每個指令都在這個視窗裡打。

**② 啟動**（Gazebo 視窗＋RViz 一起開）：

```bash
ros2 launch smartnav_sim sim_explore.launch.py use_rviz:=true
```

接下來會依序發生這些事，全程不用動手：

| 大約時間 | 你會看到 | 意思 |
|---|---|---|
| 0～5 秒 | Gazebo 視窗出現：一個大廳、北邊一條走廊、東邊一條走廊通到小房間，車子在大廳中間 | 模擬世界與車子起來了 |
| 5～20 秒 | 終端機一直刷「等待 … 生命週期服務」 | 導航鏈在開機，正常 |
| 約 20 秒 | `[smartnav_sim] nav2 已就緒，送出 /create_map（sim_日期_時間）` | 自動幫你按了 HMI 的「建立地圖」 |
| 接著 | `收到建立地圖請求` → `✓ 自動探索已啟動` | 開始自動探索 |
| 之後不斷 | `Frontier goal accepted`，車子開始移動，RViz 的地圖慢慢長大 | explorer 選了一個「未知區域的邊界」當目標 |
| 約 50 秒 | `Frontier suppression startup grace period elapsed` | 開機寬限期（30 秒）結束，從這裡起失敗才會被記下來 |
| 最後 | `探索結束（…），準備存檔` → `✓ 地圖建立成功` | 結束並存圖 |

**③ 結束**：
- 想在它自己結束前收工**並存圖**：開**第二個** Ubuntu 視窗，打 `ros2 service call /finish_map std_srvs/srv/Trigger`，等出現「地圖建立成功」
- 然後回第一個視窗按 **Ctrl+C 一次**，等它自己關完（約 5～10 秒）。不要連按
- ⚠ 直接 Ctrl+C 不先 finish：**這趟的圖不會存**，log 最後會出現 `地圖建立發生例外: cannot use Destroyable…`，這是你中途關掉造成的，不是 bug

**存下來的地圖**在 WSL 的 `~/.smartnav/map_database/data/`。從 Windows 檔案總管打開：
`\\wsl$\Ubuntu-24.04\home\<Linux 帳號>\.smartnav\map_database\data\`（`.pgm` 可以用 GIMP、IrfanView 打開）。
這跟車上的 `~/.smartnav/` 是兩個不同的地方，不會互相覆蓋。

---

## 2. 看懂畫面與 log

### RViz（`use_rviz:=true`）

預設畫面會顯示地圖、全域／區域 costmap、規劃出來的路徑、車體外框。**explorer 選的目標要自己加**：
左下 **Add** → **By topic** → `/explore/frontiers`（所有候選邊界）與 `/explore/selected_frontier`（目前選中的）。

看的重點：
- **路徑（綠線）很彎、車子卻走不出那個弧度** → 規劃器以為車子轉得比實際小（見第 4 節的 `minimum_turning_radius`）
- **目標點貼著牆** → `occ_threshold` 太寬鬆
- **地圖停止長大** → 卡住了，看 log 是哪一層在處理

### 終端機 log：哪些重要、哪些可以不理

| log | 說明 |
|---|---|
| `Frontier goal accepted` / `reached` | explorer 送出目標／到達 |
| `GridBased plugin failed to plan … "no valid path found"` | 規劃器找不到路（目標在牆邊、或轉不過去）。**偶爾有正常，同一個點一直出現就是問題** |
| `All frontiers are temporarily suppressed` | 所有候選都因為失敗被封鎖，車子原地等（等封鎖過期或停滯看門狗收尾） |
| `探索中卡住第 N 次（此處第 M 次）` | 卡住預算在計數（同一處 3 次或整趟 8 次就存圖結束） |
| `akm_realism: 近 10 秒 X 筆指令：死區吃掉 A、舵角到底 左 B／右 C` | **A 很大**：導航鏈送出的速度低於 0.085，車子根本不會動；**B、C 很大**：規劃器給的路徑比車子轉得到的還彎 |
| `Failed to get result for follow_path in node halt!`（bt_navigator 的 ERROR） | **可以不理**。每次目標被取消都會印一次，是 nav2 的已知雜訊 |
| `[compute_path_to_pose] [ActionServer] Aborting handle.` | **可以不理**，同上 |
| `XML Element[gz_frame_id] … not defined in SDF` | **可以不理**，Gazebo 其實有讀這個欄位 |
| `cmd_vel_floor: ⚠ 不抬：防撞狀態超過 10.0 秒沒更新` | ★ **目前的疑點**（2026-09-26 第一次模擬抓到）：這時速度沒被抬過死區，車子會停在原地。見第 5 節實驗 0 |

把整趟 log 存下來事後查：`ros2 launch smartnav_sim sim_explore.launch.py 2>&1 | tee ~/run1.log`，之後 `grep -c "no valid path" ~/run1.log` 這樣數。

### 建了多少圖

另開一個視窗：

```bash
ros2 run smartnav_sim map_area --watch 30     # 每 30 秒印一次「已知空地 m²」
```

這是比較兩次實驗最直接的數字。瓶頸測試場全部走完大約是：大廳 24 m²＋東走廊 5 m²＋盡頭房 9 m²＋北走廊 6 m²，扣掉家具約 42 m²。

---

## 3. 做實驗的方法

**一次只改一個參數**，其他條件（場地、起點、realism）都不動，每組至少跑兩次（探索有隨機性）。每趟記下：

| 趟次 | 改了什麼 | 場地 | 跑多久 | 最後面積（m²） | no valid path 次數 | 卡住次數 | 怎麼結束的 | 備註 |
|---|---|---|---|---|---|---|---|---|
| 基準 1 | （無） | bottlenecks | 12 分 | | | | | |

**改參數的流程**：

1. 在 **Windows 的 VS Code** 直接改檔案（WSL 裡的工作區是連到 Windows 這份 repo 的，存檔就生效）
2. Ubuntu 視窗 Ctrl+C 關掉模擬 → 重新 `ros2 launch …`。**不用重新 build**（只有新增檔案才要，見第 7 節）
3. 跑、記錄

⚠ **三件事要記得**：
- **你改的就是車上用的那份檔案**。實驗完，不要的改動在 VS Code 的「原始檔控制」按「捨棄變更」；要保留的就 commit，上車前 `git diff` 看一次
- **`ros2 param set` 在這裡幾乎都沒用**：這些節點（除了 `steering_trim_cc`）只在啟動時讀一次參數，`param set` 會回報成功、`param get` 也讀得到新值，但行為完全不變。一律改檔案、重開
- **轉向補償會記憶**：`steering_trim_cc` 會把學到的補償值存在 WSL 的 `~/.smartnav/steering_trim.json`，下次開機沿用。要每趟條件一致，跑之前刪掉它：`rm -f ~/.smartnav/steering_trim.json`

---

## 4. 參數：調了會怎樣

分兩類：**A 是要驗證、會上車的導航參數**；**B 是模擬器自己的開關**。

### A1. 探索器（`車子/src/smartnav_navigation_cc/config/frontier_explore_cc.yaml`）

檔案裡每個參數都有很長的註解寫當初為什麼設這個值，改之前先讀。標 ★ 的是檔頭寫明「待模擬驗證」的。

| 參數 | 現值 | 調大 | 調小 | 模擬裡看什麼 |
|---|---|---|---|---|
| ★ `occ_threshold` | 75 | 更貼牆的目標也會被選 → 規劃器 `no valid path` 變多 | 只選離牆遠的目標；**降到 40 左右，0.99 m 走廊裡的候選會全被濾光**，一開始就 `No more frontiers` 結束 | `no valid path` 次數、走廊有沒有被探索。**檔頭列為模擬調參第一順位** |
| ★ `max_angular_speed_wmax` | 0.12 | 認為「轉向很快」→ 常選身後的目標 → 在窄處要掉頭（阿克曼掉不了） | 更偏好車頭前方的目標 | 車子是否一直來回掉頭 |
| ★ `frontier_candidate_min_goal_distance_m` | 0.9 | 目標要離車更遠（但上游實作會退回最近點，效果有限） | 目標可以貼近車頭 | 目標點離車多遠 |
| `frontier_suppression_attempt_threshold` | 1 | 到不了的點會被重試更多次 | 1 已是最小 | 同一個點出現幾次 `no valid path` |
| `frontier_suppression_timeout_s` | 150 | 被封鎖的點更晚才重試；**超過約 164 就會在重試前被停滯看門狗收尾**（見下方規則 2） | 到不了的點更常被拿出來再試 | 封鎖過期後是否又去試同一個點 |
| ★ `frontier_suppression_no_progress_timeout_s` | 15 | 窄處多段倒車有更多時間；但真的卡住時車子要推更久才放棄 | 更快放棄目標 | 車子卡在牆邊推多久。**要小於 nav2 的 20 秒**（規則 3） |
| `frontier_suppression_startup_grace_period_s` | 30 | — | — | **不要單獨改**（規則 1） |

### A2. 卡住與收尾（改程式裡的預設值，存檔後重開模擬）

| 參數 | 位置 | 現值 | 作用 | 調了會怎樣 |
|---|---|---|---|---|
| `exploration_stall_timeout_sec` | `map_service_cc_node.py:127` | 240 | 地圖 240 秒沒長大 0.5 m² 就存圖結束 | 大：卡住時等更久才收尾；小：探索還在努力時被提早收掉 |
| `exploration_stuck_repeat_limit` / `_total_limit` | 同檔 :136-137 | 3 / 8 | 同一處（0.6 m 內）卡 3 次、整趟卡 8 次就存圖結束 | 大：卡住時花更多時間脫困；小：更早放棄 |
| `exploration_timeout_sec` | 同檔 :106 | 1800 | 整趟上限 30 分鐘 | HMI 與 LLM 的等待上限是 1920，**要一起改** |
| `stuck_time_sec` | `stuck_detector_cc_node.py:120` | 4 | 有指令（≥0.10）卻沒動（<0.02）多久算卡住 | 小：容易誤判；大：真的卡住要推更久 |
| `defer_max_sec` | 同檔 :144 | 18 | 探索中先讓 explorer 自己處理，最多等這麼久才強制取消 | ★ **不要改成 15**：檢查是每 5 秒一次，實際生效大約在 20 秒，改小會跟 explorer 搶 |
| `min_speed` | `cmd_vel_floor_cc_node.py:114` | 0.095 | 把低於死區的速度抬上來 | 小於 0.085 就等於沒抬 |

### A3. nav2（`車子/src/smartnav_navigation_cc/config/nav2_senior_akm_cc.yaml`）

| 參數 | 行號 | 現值 | 調大 | 調小 |
|---|---|---|---|---|
| Smac `minimum_turning_radius` | :907 | 0.95 | 規劃更接近實車（1.03／1.18），但很多地方會變成「規劃不出路」→ 探索提早結束 | 規劃出更彎的路，實車跟不上 → 卡住（akm_realism 的「舵角到底」會暴增） |
| `reverse_penalty` | :1017 | 2.5 | 更不願意倒車，窄處更難就位 | 更願意多段倒車；4.0 時實測變成 20 次短折返 |
| 全域 `inflation_radius` / `cost_scaling_factor` | :801-802 | 0.55 / 3.5 | 路徑離牆更遠；窄走廊可能直接不通 | 路徑更貼牆；要跟 `occ_threshold` 一起想 |
| MPPI `vx_max` | :293 | 0.25 | 開更快 | 開更慢（別低於 0.1，會被死區吃掉） |
| `movement_time_allowance`（進度檢查） | :246 | 20 | — | **要大於 explorer 的 15 秒**（規則 3） |

### 參數之間的綁定規則（違反的話，單獨看每個值都合理，合起來會出事）

1. **三個寬限期必須同值（現在都是 30）**：`frontier_suppression_startup_grace_period_s`（yaml）、
   `defer_grace_sec`（`stuck_detector_cc_node.py:135`）、`exploration_stuck_grace_sec`（`map_service_cc_node.py:143`）
2. **封鎖時間 ＋ 開過去（≤60）＋ 無進展逾時（16）＜ 停滯看門狗**：150 + 60 + 16 = 226 < 240。
   否則偶發失敗的最後一個目標，還沒輪到重試就被收尾
3. **explorer 無進展逾時（15）＜ nav2 進度檢查（20）**：讓 explorer 先放棄並記下失敗，
   而不是讓 nav2 進入恢復動作（原地前後移）
4. 改完 1～3 相關的參數，在 Windows 的 `專題/` 跑一次 `python scripts/explore_watchdog/test_explore_fixes.py`（本機才有），規則 1 它會自動檢查

### B. 模擬器自己的開關（啟動時加在指令後面，例如 `… sim_explore.launch.py gui:=false realism:=false`）

| 參數 | 預設 | 用途 |
|---|---|---|
| `gui` | true | false = 不開 Gazebo 視窗，最省資源（光達照樣有資料）。長時間實驗建議關 |
| `use_rviz` | false | true = 開 RViz |
| `world` | bottlenecks.sdf | 換場地（`worlds/` 底下的檔名或絕對路徑） |
| `map_yaml` | （空） | 拿實車地圖當場地（見實驗 3） |
| `realism` | true | false = 理想車：沒有死區、左右都是 0.75 m（機械偏移保留） |
| `x` `y` `yaw` | auto／auto／0 | 起點（公尺、弧度） |
| `auto_explore` | true | false = 不自動開始，自己送 `/create_map` |
| `render_engine` | （空） | Gazebo 視窗一開就崩時設 `ogre` |
| `lidar_x_actual` | （空） | 模擬光達的**實際** x 位置（TF 仍是 0.0887），用來看 TF 量錯對建圖的影響 |

實車不對稱的細部數字（死區 0.085、半徑 1.030／1.183、偏移 +0.235）在
[`smartnav_sim/akm_realism_node.py`](smartnav_sim/akm_realism_node.py) 的預設值裡。
想做「假設舵機修好了會怎樣」，就把半徑改成 0.80 跑一趟對照。

---

## 5. 建議先做的實驗

### 實驗 0：確認死區守門員的疑點（★ 先做這個）

2026-09-26 第一次整合模擬（12 分鐘、預設參數）裡，`cmd_vel_floor` 多次印出「防撞狀態超過 10 秒沒更新，不抬」，
同一段時間 akm_realism 報告一個 10 秒窗口內 206 筆指令有 200 筆被死區吃掉。
推測原因：`collision_monitor_state` 只在**狀態改變時**才發布，狀態維持 10 秒不變，守門員就誤判成「不知道狀態」而停止抬速度
（`cmd_vel_floor_cc_node.py:242`；8/27 修過「一則都沒收到」的情況，這條路徑沒一起修）。**實車應該也會發生。**
驗證：跑一趟，數 `grep -c "不抬：防撞狀態超過" ~/run.log`，對照 akm_realism 的「死區吃掉」。

### 實驗 1：基準

```bash
rm -f ~/.smartnav/steering_trim.json
ros2 launch smartnav_sim sim_explore.launch.py gui:=false 2>&1 | tee ~/base1.log
```

另開視窗 `ros2 run smartnav_sim map_area --watch 30`。記下最後面積、怎麼結束的。
第一次整合模擬的參考結果：12 分鐘停在約 30 m²（大廳加北走廊的一部分），**沒有去東走廊**；
`no valid path` 大多集中在西牆與沙發旁。

### 實驗 2：理想車對照

同上，加 `realism:=false`。如果理想車能建完、實車版不行，差距就是「轉向不足＋死區」造成的，
方向是調規劃半徑或選路，而不是調探索器。

### 實驗 3：實車地圖

```bash
ros2 launch smartnav_sim sim_explore.launch.py \
  map_yaml:=/mnt/c/Users/<Windows 使用者名稱>/Desktop/專題/車子/.smartnav/map_database/data/map_127f250e543f.yaml
```

三張舊實驗室的圖：`map_79a3793e6c07`（7/31）、`map_6aa732f181c6`（7/31 第二張，原點在未知區，會自動換起點）、
`map_127f250e543f`（8/03）。終端機一開始會印出牆塊數與起點。

### 實驗 4：`occ_threshold`（檔頭列的第一順位）

75 → 60 → 50 各跑兩趟，比較 `no valid path` 次數與最後面積。降太多會一開始就 `No more frontiers`，那就是下限。

---

## 6. 模擬了什麼、沒模擬什麼

模擬車對導航鏈提供**跟實車一模一樣的介面**：`/scan`（frame `laser`）、`/odom_combined`、
TF `odom_combined → base_footprint → {base_link, laser}`，吃 `/cmd_vel`。導航那邊一行都不用改。

| 項目 | 模擬 | 出處 |
|---|---|---|
| 車體幾何 | 原點在**後軸**；軸距／輪距 0.322、胎徑 0.125、外框 −0.09～+0.40 | 韌體 `robot_select_init.h`、nav2 footprint |
| 光達 | ls_N10：每圈 390 點、0.15～12 m、10 Hz、雜訊 1 cm，從 −180° 起算 | `wheeltec_param.yaml`；390 點由「車尾 80° 內 87 根」推估，待 V0 核對 |
| **線速度死區** | 0.085 m/s 以下不動 | 8/24 逐筆 1583 筆 |
| **最小迴轉半徑** | 左 1.030／右 1.183 m | 8/29 實測（`車子/README.md`） |
| **機械零位偏移** | 下 0 往左偏 +0.235 rad/m；導航鏈的 `steering_trim` 剛好補回 | 8/29 `steer_asym_check` |
| 命令看門狗 | 1 秒沒指令就停 | 韌體（`docs/底盤與韌體分析.md`） |
| 里程計 | 後輪輪速積分（取代實車的 EKF），有輪胎側滑造成的自然誤差 | Gazebo AckermannSteering |

粗體三項在 `akm_realism` 節點，`realism:=false` 可以關掉做對照。

**2026-09-26 在空場地實測**（0.2 m/s，舵打穩後量 5 秒）：

| 動作 | 目標 | 里程計 | Gazebo 真實位姿 |
|---|---|---|---|
| 左轉到底 | 1.030 m | 1.007 m | 1.058 m |
| 右轉到底 | 1.183 m | 1.163 m | 1.233 m |
| 下 0 不補償（機械偏移） | 4.255 m | 4.250 m | 4.482 m |
| 倒車下 0 不補償 | 4.255 m、反號 | 4.248 m、−17.7° | 4.424 m |
| 經 steering_trim 補償 | 直線 | 0.0° | 0.0° |
| 0.08 m/s（死區） | 不動 | 不動 | 不動 |

真實半徑比里程計大 4～5%（輪胎側滑，里程計以為自己轉得比較多）——這個誤差要靠 SLAM 的掃描比對修正，跟實車一樣。
實車有 IMU 補航向，誤差會比模擬小。整趟模擬後 `steering_trim` 自己學到的補償是 −0.2339（預設 −0.235），代表偏移與補償對得上。

**沒模擬（實車有、模擬沒有）**：IMU 零偏、Pi4 滿載造成的 TF 空窗（筆電快很多）、跟在車後的人、低矮障礙（電線、地毯邊）、
電池電壓對舵機的影響。所以**模擬過了不保證車上過；但模擬裡就過不了的，車上通常也過不了**。

⚠ **即時率（RTF）**：`stuck_detector_cc`、`steering_trim_cc`、`cmd_vel_floor_cc` 用牆上時鐘。RTF 低於 0.9 時它們的時間判斷會失準。
查法：`gz topic -e -n 1 -t /stats | grep real_time_factor`。第一次整合模擬（不開視窗）大多是 1.00，偶爾掉到 0.26～0.78，
那幾段時間附近的「卡住」判定要打折看。

### 場地：`worlds/bottlenecks.sdf`（預設）

```
  ┌──────────── 北走廊 0.99 ────────────┐
  │   ╔══ 0.967 窄門口（接 90 度轉角）   │
  │ 大廳 6×4 m                          ├── 東走廊 0.99 × 5 m ──┐ 盡頭房 3×3 m
  │   起點 (0,0) 面向東 →                │                       │（需三點轉向）
  └─────────────────────────────────────┘                       ┘
```

- 大廳、東走廊（直進）、盡頭房：設計上到得了
- **北走廊**：窄門口進去要立刻 90° 轉彎。規劃器用 0.95 m、控制器用 0.80 m，可能覺得過得去；
  但模擬車跟實車一樣只轉得到 1.03／1.18 → 這正是 explorer 看門狗、卡住後備、卡住預算三層要處理的情境
- 兩個瓶頸都刻意低於選路門檻（淨寬 1.17 m）
- 改版面：直接改 `worlds/bottlenecks.sdf` 裡的方塊，每塊都有中文標籤

實車地圖轉成的世界：`ros2 run smartnav_sim pgm_to_world <地圖.yaml> -o my_world.sdf`（會印出牆塊數與建議起點），
之後 `world:=$PWD/my_world.sdf x:=… y:=…`。世界座標＝地圖座標。

---

## 7. 疑難排解

| 症狀 | 對策 |
|---|---|
| Gazebo 視窗一開就崩 | 加 `render_engine:=ogre`；還是不行就 `gui:=false`，改用 RViz 看 |
| 模擬很慢、車子像慢動作 | 查 RTF（上面）。關掉 Gazebo 視窗（`gui:=false`）通常就回到 1.0 |
| 找不到 `map_area` 等新指令、或新增了檔案沒生效 | `cd ~/sim_ws && colcon build --symlink-install --packages-select smartnav_sim && source install/setup.bash` |
| Windows 那邊 `git pull` 之後 | 同上一行（把 `smartnav_sim` 換成有新檔案的套件）；只是改檔內容則不用 |
| 車子完全不動 | 看 akm_realism 的「死區吃掉」：很多 → 速度太低（見實驗 0）；0 → 導航鏈沒送指令，看 log 前面的錯誤 |
| 上一次沒關乾淨，這次話題重複、行為怪 | `ps -eo pid,cmd \| grep "gz sim"` 找出殘留的 PID，`kill <PID>`（★ 不要用 `pkill -f`） |
| 遙控：自己開車 | 只開 `ros2 launch smartnav_sim sim_robot.launch.py`，另開視窗 `ros2 run smartnav_navigation_cc teleop_keyboard_cc --ros-args -p speed:=0.15`（預設 0.06 低於死區，車子不會動；直接發 `/cmd_vel` 沒有轉向補償，會往左偏，跟實車一樣） |

## 檔案

| 檔案 | 是什麼 |
|---|---|
| `launch/sim_explore.launch.py` | 一行跑完整趟自動建圖 |
| `launch/sim_robot.launch.py` | 只有世界＋車（取代實車的 `sensors_cc.launch.py`） |
| `urdf/senior_akm_sim.urdf.xacro` | 模擬車（阿克曼外掛＋gpu_lidar） |
| `smartnav_sim/akm_realism_node.py` | 實車不對稱（死區、左右半徑、機械偏移、看門狗） |
| `smartnav_sim/pgm_to_world.py` | 地圖 → Gazebo 世界 |
| `smartnav_sim/map_area.py` | 印出已知面積 |
| `config/bridge.yaml` | Gazebo ↔ ROS 話題對接 |
| `worlds/bottlenecks.sdf` | 瓶頸測試場 |
| `test/` | 單元測試（`python3 -m pytest test/`，不需要 ROS） |
| `setup/` | WSL＋ROS 2＋Gazebo 安裝腳本 |
