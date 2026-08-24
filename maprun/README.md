# maprun 工具索引

工具依**寫的日期**分目錄（`tools_0810` ~ `tools_0819`），但你要找工具時想的是
「我要回答什麼問題」。這份索引照問題分類，路徑寫全。

★ 規則兩條，違反過很多次：
- **不要用 `pkill -f` / `pgrep -f`**。停節點用 `kill_node_cc.sh <package> <executable>`，
  查行程用 `pgrep -x`。理由寫在 `kill_node_cc.sh` 檔頭（踩過五次，每次都是 shell 自己被殺）。
- **感測器一定要早於導航啟動**。反過來會進入救不回來的 mapping 死結。

---

## 每天開機的四支

| 指令 | 做什麼 |
|---|---|
| `run_sensors_cc.sh` | 底盤 + 雷達 + IMU 補償。**第一個跑** |
| `run_nav_cc.sh` | 導航堆疊（`auto` / `mapping`） |
| `run_asr_cc.sh` | 語音（會自己找音效卡編號、設 `Mic,0` 與 `Mic,1` 增益） |
| `stop_nav_cc.sh` / `kill_sensors_cc.sh` | 收乾淨。★ 重啟導航前一定要先跑，否則會產生重複節點 |

單起一個節點：`run_node_cc.sh <package> <executable> [log名] [--ros-args ...]`
單停一個節點：`kill_node_cc.sh <package> <executable>`

---

## 車子現在在哪？定位對不對？

| 工具 | 回答 |
|---|---|
| `tools_0814/where_am_i.py <半徑> <角度>` | 「車子現在在哪個**已知地點**」。不在地點上就沒轍 |
| `tools_0817/relocalize.py [--set] [--free360]` | 「AMCL 跟丟了，幫我在地圖上找回來」。★ 車被**人搬過**才加 `--free360` |
| `pose_check_cc.py` | 「現在這個估計**對不對**」。不給正確答案，只給是非 |
| `tools_0810/set_pose_here.py <地點名>` | 直接把 AMCL 設到某個已知地點 |
| `tools_0817/costmap_at_robot.py` | 印出成本地圖在車子所在位置的實際數值。★ 話題是 **0~100** 不是 0~254 |

---

## 車子為什麼不動 / 亂撞？

| 工具 | 回答 |
|---|---|
| `tools_0817/odom_feedback_check.py` | ★ **「指令速度 → 實際輪速 → EKF 速度」這條鏈斷在哪**。死區問題先用這支 |
| `tools_0817/nav_trace.py <秒>` | 全程逐筆 CSV：指令 vs 實速，專抓 `/cmd_vel` 的**發布空窗** |
| `steer_asym_check_cc.py` | 左右轉彎半徑不對稱（實測左 0.944 / 右 0.751） |
| `odom_gyro_check_cc.py` | 里程計與陀螺儀對不對得上 |
| `tools_0810/scan_front.py` | 車頭前方剖面：擋住的是**人**還是**牆** |
| `tools_0817/goal_yaw_effect.py` | 同一個終點、只改朝向，比較折返點數（實測 0 vs 14 個） |
| `tools_0814/plan_only.py` | 只規劃不執行，確認規得出路徑 |
| `nav_cc_check.py` | 導航堆疊整體健檢 |

## 車子卡住了（現場救車）

| 工具 | 做什麼 |
|---|---|
| `tools_0817/unwedge.py` | 把楔在牆邊、規劃器已拒絕出手的車推開 |
| `tools_0810/rescue.py` | 人工把貼右牆的車拉開 |
| `tools_0810/turn_and_go.py` | 大廳掉頭 → 純前進開回起點 |
| `estop_cc.sh` | 緊急停止 |
| `tools_0817/estop_watch.py` | 量緊急停止的實際反應秒數（實測 0.768 s） |

---

## 語音：沒觸發？認錯字？

★ 先分清楚是**收音**還是**模型** —— `asr_breakdown.py` 就是為這件事寫的。

| 工具 | 回答 |
|---|---|
| `tools_0819/asr_breakdown.py` | ★ 「VAD 沒觸發」還是「辨識錯」。同時出 `cer_first` 與 `cer_best` |
| `tools_0819/asr_cer.py` | 字錯率 CER 與即時率 RTF。★ README 的 25.6% 要跟 **best** 比 |
| `tools_0814/asr_inject_cc.py` | 把 WAV 當麥克風重播。★ 重播前要先停 `voice_trigger` |
| `tools_0814/asr_vad_sim_cc.py` | 離線重現 VAD 狀態機 |
| `tools_0814/asr_mic_probe_cc.py` | 麥克風能不能用節點的參數開起來 |
| `tools_0810/asr_test.py` | 拿實錄 WAV 直接餵 sherpa-onnx |

### 雙麥克風（Astra S 內建，卡號每次開機會變）

| 工具 | 做什麼 |
|---|---|
| `tools_0817/rec_dual_cc.sh <名稱> <秒>` | 錄雙聲道 |
| `tools_0817/mic_snr_cc.py --fit <wav>` | 訓練消噪係數 |
| `tools_0817/mic_validate_cc.py` | 跨場次驗證，決定要不要補增益 |
| `tools_0817/mic_lag_sweep_cc.py` | 不同到達時間差下的 SNR 改善 |

★ 係數檔 `tools_0817/mic_cancel_coeffs.txt`。**重訓前一定要另存備份** ——
單聲道在這個環境 VAD 觸發 **0 次**，靠這組係數才有 19 次。

---

## 人臉

| 工具 | 回答 |
|---|---|
| `tools_0814/sync_lag.py` | ★ 時間戳落後多久 + 模擬固定佇列。「遮住鏡頭還是 VIP」用這支 |
| `tools_0814/face_rate.py` | 管線每一段的速率，找註冊瓶頸 |
| `tools_0814/face_verify_and_register.py` | 站定一次同時做「驗同步」與「註冊」 |
| `tools_0819/face_distance.py` | 臉寬 vs 雷射，兩個獨立來源對照距離 |
| `tools_0819/grab_frames.py` | 抓幾張有臉的影格存 JPG |
| `tools_0814/reg_watch.py` | 註冊時盯 `registration_progress` |

★ 定案值：門檻 **0.70**、觸發距離約 **1 公尺**（不是越近越好）、維持 **buffalo_sc**。

---

## LLM

| 工具 | 回答 |
|---|---|
| `tools_0814/llm_tool_bench.py` | 工具選擇準確率（只發 `user_text`，不碰動作） |
| `tools_0814/offline_probe.py` | 不啟 ROS，直接拿節點的工具定義問 ollama |
| `tools_0814/compare_prompts.py` | 離線比較不同 system prompt |
| `tools_0810/llm_bench.py` | 端到端延遲 |

★ ollama 在筆電 `192.168.137.1:11434`，模型 `qwen2.5:3b`。

---

## 資源占用 / 效能

| 工具 | 回答 |
|---|---|
| `tools_0814/cpu_report.py <秒>` | 各節點 CPU／記憶體，直接輸出報告用的表格（E7） |
| `tools_0810/proc_sampler.py` | 逐行程採樣：「哪些該留 Pi、哪些該搬筆電」 |
| `tools_0810/cpu_sampler.py` | 每秒 CPU／記憶體／溫度寫 CSV |
| `tools_0814/rate.py <話題>` | 話題實際頻率（取代 `ros2 topic hz`，CLI 排不到時仍準） |
| `tools_0819/e2e_latency.py` | E6 端到端延遲。★ 量之前要先降迎賓冷卻 |

★ 已量到的天花板：車子靜止、沒開人臉沒開 ASR，就已經 **382~398% / 400%**。

---

## 建圖

`create_map_cc.sh` 建圖 · `mapping_watch_cc.py` 盯建圖過程 · `corridor_trim_cc.py` 修走廊 ·
`loop_watch_cc.py` 盯迴圈閉合

---

## 目錄

```
maprun/
├── *.sh            每天在用的啟動 / 停止腳本
├── *.py            常用的檢查工具（定位、里程計、健檢）
├── tools_0810/     走廊實跑、救車、資源採樣
├── tools_0814/     ASR 離線工具、人臉管線、LLM 對照、where_am_i
├── tools_0817/     雙麥克風、重新定位、軌跡記錄、脫困
├── tools_0819/     ASR 拆解與 CER、端到端延遲、人臉距離
├── logs/           所有節點的執行 log（nav_cc.log 是符號連結指向最新那份）
├── logs_0814/      ★ ASR 兩個節點的 log 在這裡，不在 logs/
├── maps/           建圖過程的分段地圖
├── maps_backup/    地圖資料庫備份
└── 舊版_勿用/       舊 smartnav_navigation 的啟動腳本，看 README.txt 就知道為什麼別碰
```

★ `logs_0814/` 這個名字會誤導 —— 它裝的是**現在**的 ASR log，
`run_asr_cc.sh` 寫死指向那裡。看 ASR 就去那裡，不要在 `logs/` 找。
