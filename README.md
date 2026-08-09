# 具人臉辨識與語音對話功能之迎賓機器人 — 實機開發主線

![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue)
![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu-24.04-orange)
![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%204B%208GB-red)
![循跡精度](https://img.shields.io/badge/循跡精度-3.42%20cm-brightgreen)

大學部畢業專題。WHEELTEC 阿克曼實車 ＋ 人臉辨識 ＋ 語音對話 ＋ 自主導航。

> **這是 `車子` 分支——目前實際在實車上運行的完整系統。**
> 專案總覽、硬體層說明與完整實測數據在 [`master` 分支](../../tree/master)。
> 兩個分支的 git 歷史互相獨立，是同一專案的兩個層次。

---

## 這個分支有什麼

```
.
├── welcoming_robot_ws/          完整 ROS 2 workspace
│   └── src/smartnav_ws/src/
│       ├── smartnav_msgs/           介面定義（msg / srv / action）
│       ├── smartnav_vision/         人臉向量擷取（InsightFace 512D）
│       ├── smartnav_brain/          身份驗證、使用者管理、銀行迎賓劇本
│       ├── smartnav_audio/          喚醒詞、ASR、TTS（sherpa-onnx）
│       ├── smartnav_llm/            LLM 對話 Agent（LangChain + Ollama）
│       ├── smartnav_navigation_cc/  ★ 導航重寫版：SLAM、AMCL、教導-重現路徑
│       ├── smartnav_navigation/     舊版（模擬用，frame 不同不可混用）
│       ├── smartnav_hmi/            平板網頁介面（FastAPI）
│       └── smartnav_bringup/        統一啟動
├── maprun/                      建圖／導航／節點管理的操作腳本
├── .smartnav/                   地圖、地點、教導路徑資料
└── *.md                         交接與分析文件（見下）
```

## 開發文件

本專案的內部開發文件（實機日交接記錄、硬體驗收清單、廠商韌體分析、
車上 AI agent 的操作手冊）**不隨此儲存庫發布**，隨部署包提供給實際操作的人。

原因有二：一是那些文件記錄的是逐日的除錯過程與未完成項目，對外部讀者
沒有參考價值；二是韌體分析的對象是廠商未公開發布的原始碼。

這份 README 已涵蓋對外部讀者有意義的部分——系統架構、實測數據、
設計取捨的理由，以及踩過的坑。

## 最近進度（2026-08-03 實機日）

| 成果 | 數據 |
|---|---|
| **教導-重現循跡精度** | 624 筆取樣，平均偏離 **3.42 cm**、最大 8.60 cm |
| 地點停止誤差 | 0.17 m（容差 0.25 m） |
| AMCL 掃描過濾 | 地圖吻合度 71% → **90%** |
| 實機抓出並修復的缺陷 | 10 個（靜態檢查全都看不出來） |

**未解**：門口窄轉角（**該處**淨寬 0.967 m；走廊本身 0.99 m）自動脫困未成功。

> 2026-08-06 補充：人示範的三點轉向**也可能重現不了**——實測有一段需要
> `R = 0.688 m`，低於韌體硬限 0.750 m，控制器照 0.80 箝制做不出來。
> 存檔時已加入 0.76 m 的可行性檢查並在重播前攔截。
> 目前**穩定通過窄轉角的組合是 SmacPlannerHybrid 規劃 + 純追蹤執行**。

現行對策是錄製教導路徑時由人在轉角示範三點轉向：先把
`merge_min_segment_m` 降到 0.15，否則短於預設門檻 0.35 m 的示範動作
會在存檔時被當成假折返合併掉。

## 核心設計：教導-重現路徑（Teach & Repeat）

阿克曼車不能原地旋轉，最小迴轉半徑 0.750 m，而測試走廊淨寬僅 0.99 m。
MPPI 控制器的預測視野只有 `30 × 0.1 × 0.25 = 0.75 m`，
而完成 90° 轉彎需要 `(π/2) × 0.80 = 1.26 m`——**看得到的距離只有所需的 60%**，
評分時分不出好壞，規劃不出可行軌跡。

解法不是把控制器調得更聰明，而是**換方法**：錄下人開過的位姿序列，
重播時用純追蹤跟隨。

```bash
# 錄製（★ 服務在根命名空間，不是 /path_teach_cc/...）
ros2 service call /record_path smartnav_msgs/srv/RecordPath \
  "{action: 0}"                                     # 0=START，開始遙控車子
ros2 service call /record_path smartnav_msgs/srv/RecordPath \
  "{action: 1, name: '大廳到貴賓室'}"                # 1=STOP，存檔（name 在這裡才生效）

# 重播
ros2 action send_goal /follow_taught_path smartnav_msgs/action/FollowTaughtPath \
  "{path_id: 'path_xxxx', reverse: false, speed_scale: 1.0}"
#   speed_scale 是倍率不是絕對速度；實際速度由節點參數 follow_speed 決定（預設 0.15 m/s）
```

**搜尋 vs 示範**：MPPI 是「讓機器在所有可能軌跡裡搜出一條」，
教導-重現是「人走一次給它看」。當環境約束緊到讓可行解幾乎不存在時，
搜尋的成本會爆炸，而示範的成本不變。

## 快速開始

```bash
git clone -b 車子 https://github.com/xiange1209/welcoming_robot.git ~/welcoming_robot_ws_repo
cd ~/welcoming_robot_ws_repo/welcoming_robot_ws
colcon build --symlink-install     # ★ 一律從 workspace 頂層建置
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

```bash
# 建圖
./maprun/run_nav_cc.sh mapping true

# 定位＋導航（★ 兩個參數都要給，只給第一個會誤啟探索模式）
./maprun/run_nav_cc.sh localization false
```

## 開發時必須知道的四件事

1. **改任何 `.msg`/`.srv`/`.action` 都要全 workspace 重建並重啟所有節點。**
   type hash 會變，只重建部分套件的症狀是「節點都在跑、topic list 看得到，
   但訂閱端一則訊息都收不到」，而且**不會有任何錯誤訊息**。

2. **不要用 `pkill -f`。** 字串比對誤判過七次，最嚴重一次殺掉 IMU 補償節點
   導致兩次建圖全毀，而健檢只查 `/scan` 與 `/odom` 所以完全沒發現。
   用 `maprun/kill_node_cc.sh`。

3. **低電壓 20 V 只切馬達不切舵機**（韌體行為）。症狀是「方向舵會轉但車不走」——
   先量電池，不要查 ROS。

4. **開機後 1.0~2.0 秒車子會自己前進 0.02 m/s**，那是廠商自檢，不是故障。
   2.5 秒後才吃 ROS 指令。不要放在桌上開機。

## 授權

MIT（見 [`master` 分支的 LICENSE](../../blob/master/LICENSE)）。
`src/` 下的 WHEELTEC 廠商程式碼依其原始授權條款。
