# 基於 ROS2 與 LLM 之智慧迎賓機器人系統

![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue)
![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu-24.04-orange)
![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%204B%208GB-red)
![人臉門檻](https://img.shields.io/badge/人臉辨識門檻-0.70%20(實測定出)-blue)
![License](https://img.shields.io/badge/License-MIT-green)

大學部畢業專題。一台以銀行大廳為應用場域的迎賓機器人：**認得來賓、聽得懂口語、依身分帶位**。

- **硬體**：Raspberry Pi 4B（8 GB）＋ WHEELTEC senior_akm 阿克曼底盤、LSLiDAR N10 光達、Orbbec Astra S 深度相機、平板
- **運算分工**：人臉、語音、導航、網頁在機器人上；語言模型在**同一個區域網路的筆電**（Ollama），人臉影像與錄音不上傳雲端
- **特色**
  1. **語言模型答錯，機器人也不會越權帶位**——語言模型只能提出帶位要求，能不能走由 `nav_arbiter` 依身分判斷
  2. **同樣的要求，依身分提供不同服務**——貴賓以姓名問候、可前往貴賓室；一般訪客婉拒；黑名單不帶位並通知行員
  3. **人臉與語音不上傳雲端**——對外連線只有通知行員的一則文字（Telegram）與查匯率

## 功能

| 功能 | 做法 |
|---|---|
| 人臉辨識 | InsightFace（buffalo_sc）512 維向量，門檻 0.70；在 Pi 4 上約 1 秒處理一張 |
| 語音辨識 | 平板收音 → Silero VAD → sherpa-onnx 串流 Zipformer（解碼器 fp32），支援熱詞 |
| 對話 | llama3.1:8b（Ollama）＋ 銀行知識庫（RAG：BGE-M3、Chroma）＋ 工具呼叫；每輪只開放相關的工具 |
| 帶位 | SLAM 建圖、AMCL 定位、Smac Hybrid A* 規劃、自行開發的路徑追蹤（Pure Pursuit）與路線重播 |
| 身分權限 | `nav_arbiter` 是唯一的導航入口：語音、按鈕、語言模型的要求都要經過它 |
| 通知行員 | 黑名單出現、來賓求助時送 Telegram 文字 |
| 平板介面 | React ＋ FastAPI：迎賓頁、管理頁（註冊人臉、建圖、標地點、系統開關）、緊急停止（不經仲裁器） |
| 處理器資源分配 | 帶位時導航優先、迎賓時恢復，由程式自動切換 |

## 測試結果

下列數字各自有測試條件，**不同條件的數字不能互相比較**；場地是實驗室模擬的銀行大廳，受測者是組員。

| 項目 | 結果 | 條件 |
|---|---|---|
| 人臉辨識 | 2 公尺內辨識率 93～100%，測試中未誤認為他人 | 門檻 0.70；註冊人數很少，不能外推到大量註冊者 |
| 語音辨識 | 字錯率約 4% | 平板收音、安靜的實驗室、朗讀同一份文稿（同條件下相機麥克風約 76%） |
| 語音模型 | 解碼器 int8 → fp32：字錯率 81% → 22% | 同一批合成語音，處理速度不變 |
| 語言模型選擇 | llama3.1:8b：業務問答 8 題答對 7 題、需用工具 5 題都有呼叫、不需工具 3 題有 1 題誤呼叫 | 同一台 8 GB 顯示記憶體的筆電測 11 個模型 |
| 知識庫 | 業務問答答對率 28% → 77% | 11 個模型平均 |
| 自主帶位 | 11 次成功 10 次，終點誤差 0.1～0.3 m | 各項功能分開測、先移開低於光達的障礙物、由管理頁發出要求 |
| 全部功能同時開 | 處理器分配調整並重建地圖後，語音帶位抵達貴賓室與櫃檯（終點誤差 0.21 m）；另一趟因導航節點當掉中止 | 當天連同調整前要求帶位 16 次、抵達 7 次；處理器仍接近滿載 |
| 帶位權限 | 20 種話術要求前往貴賓室，全部被拒絕 | 一般訪客、黑名單各一輪；文字輸入、模擬身分、車不動 |
| 緊急停止 | 行進中按下約 0.5 秒停住 | 平板急停 |
| 資料本地處理 | 61 分鐘內機器人程式沒有對外連線 | 期間沒有觸發通報與查匯率 |

## 已知限制

- 全部功能同時開啟時處理器仍接近滿載（單板 4 核心）。
- 平面光達看不到低於它高度的障礙物（例如椅腳）；場地要先移開。
- 阿克曼底盤不能原地旋轉，通道需要約 1.2 公尺；實測最小迴轉半徑 0.75～1.18 公尺，會隨左右方向與校正改變。
- 多人環境會收到旁人的聲音；貴賓偶爾沒被辨識到，暫時視為一般訪客。
- 語言模型仍可能**說錯**（管得住行動，管不住說話）；沒有活體偵測，不能當門禁。
- `1ef997a` 的修正（貴賓寬限、錯誤訊息、換主機、黑名單通知…）**通過單元測試，尚未上車**。

逐日的開發記錄、實驗數據與踩過的坑 → [HISTORY.md](HISTORY.md)。

> **`master` 是唯一的開發主線**。車上要跑還需要 **`wheeltec` 分支**（17 個 WHEELTEC 廠商驅動），兩個分支歷史各自獨立，**不合併**。

---

## 目錄

repo 根目錄**就是** ROS 2 workspace（在這裡 `colcon build`）。

```
.
├── src/
│   ├── smartnav_msgs/           介面定義（msg / srv / action）
│   ├── smartnav_vision/         人臉向量擷取（InsightFace 512D）
│   ├── smartnav_brain/          業務決策中樞：身份驗證、使用者管理、迎賓劇本與通報、導航仲裁（nav_arbiter）
│   ├── smartnav_audio/          喚醒詞、ASR（sherpa-onnx）、雙麥克風降噪
│   ├── smartnav_llm/            迎賓對話 Agent（LangChain + Ollama）：只理解與回話，動作一律向 brain 請求
│   ├── smartnav_navigation_cc/  ★ 導航重寫版：SLAM、AMCL、教導-重現路徑、自動探索
│   ├── smartnav_hmi/            平板網頁介面（React 前端＋FastAPI）＋ 瀏覽器 TTS 出聲
│   ├── smartnav_bringup/        demo.launch.py（311 行，五階段延遲啟動）＋ systemd
│   ├── smartnav_sim/            （開發中）筆電 WSL 的 Gazebo 建圖模擬，不上車
│   └── frontier_exploration_ros2/  ★ git 子模組（上游 frontier 探索的 fork，含我們的修補）
├── maprun/                      操作腳本（Pi 上位於 ~/maprun）
├── cyclonedds.xml               DDS 設定（env.sh 先找 ~/，再找 repo 根目錄）
├── pack_car_code.py             打包上車（實機更新的正式途徑）
├── .smartnav/                   地圖、地點、教導路徑資料
└── *.md                         交接與分析文件（見下）
```

## 套件職責邊界

原則：**決策只在 `smartnav_brain`，其餘套件只「感測／表達／執行」，不各自做業務判斷。**

```
 smartnav_audio ──user_text──▶ smartnav_llm ──RequestNavigation──▶ ┐
 （聽、說）                     （理解與回話）                       │
                                    │ staff_notify_request          ▼
 smartnav_vision ─FaceEmbedding─▶ smartnav_brain ◀──────RequestNavigation── smartnav_hmi
 （看）                          （身分、權限、仲裁、通報）             （人機介面、管理功能）
                                    │ Navigate
                                    ▼
                             smartnav_navigation_cc（建圖、定位、導航、教導路徑）
```

| 套件 | 負責 | 明確不負責 |
|---|---|---|
| `smartnav_msgs` | 全系統的介面定義；時限等跨套件常數的單一來源（`RequestNavigation.action`） | 任何邏輯 |
| `smartnav_vision` | 人臉偵測與向量擷取 | 判斷這張臉是誰、該怎麼對待 |
| `smartnav_audio` | 喚醒、語音辨識、語音合成與播放 | 理解語意 |
| `smartnav_llm` | 把客人的話變成回話與工具呼叫（帶位、通報、查匯率與時間）；FAQ 與提示詞 | 能不能帶位（問 `nav_arbiter`）、改地圖、直接呼叫 `/navigate` |
| `smartnav_brain` | **唯一的業務決策點**：身分辨識與使用者資料（`user_auth`）、迎賓劇本與通報（`bank_reception`）、導航仲裁與權限規則（`nav_arbiter`、`access_policy.py`） | 感測、推論模型、路徑規劃 |
| `smartnav_navigation_cc` | 建圖、定位、路徑規劃與追蹤、教導-重現、自動探索；提供 `/navigate` | 誰可以去哪裡（只執行 `nav_arbiter` 放行的目標） |
| `smartnav_hmi` | 平板網頁、遙控、管理者的地圖／使用者管理、系統控制面板、緊急停止 | 客人身分的權限判斷；迎賓頁與管理者導航都經 `nav_arbiter` |
| `smartnav_bringup` | 分階段啟動整套系統 | 任何執行期邏輯 |
| `smartnav_sim` | 筆電 WSL 的 Gazebo 模擬（不上車） | — |

導航的唯一入口是 `request_navigation`（`RequestNavigation.action`），來源分 `LLM`／`GUEST_UI`／`OPERATOR`；
只有 `nav_arbiter` 會呼叫 `/navigate`。**緊急停止不依賴 `nav_arbiter`**：HMI 直接對 `/navigate` 與 nav2 送「取消全部」。
`nav_arbiter` 沒起來時，客人端的帶位一律失敗（預設拒絕），不會退回直接導航。

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

## 部署到 Pi

**現在的車（2026-10-28 起）**：車上是兩個 git clone（`~/welcoming_robot_ws/src/smartnav_ws`＝master、`src/wheeltec_ws`＝wheeltec）＋複製式安裝，
更新＝在 clone 裡 `git pull` → `cd ~/welcoming_robot_ws && colcon build --packages-select <改過的套件>` → `sudo systemctl restart smartnav-hmi`
（前端改過要先在筆電 `npm run build`，再把 `dist/` 帶到車上）。細節見 [CLAUDE.md](CLAUDE.md) §2。
處理器分時要裝 `src/smartnav_bringup/systemd/smartnav-hmi.service.d/20-asr-priority.conf`（安裝方式在同資料夾的 README）。

**沒有網路、要整包帶過去時：用打包更新**（在筆電跑 `python pack_car_code.py`，它會印出完整的 Pi 端步驟）。
打包只帶我們自己的程式，不含廠商驅動；車上現有的廠商驅動解壓時不會被動到。

**全新的 Pi（或換新 SD 卡）：clone 兩份、建兩個 workspace。**廠商驅動（`wheeltec` 分支）自成一個
底層 workspace，我們的疊在它上面——這是 ROS 的標準 underlay／overlay 做法，repo 裡也就不會出現巢狀的
`*_ws/`（本 repo 的 `CLAUDE.md` 明文禁止）。

```bash
source /opt/ros/jazzy/setup.bash
# 0. astra 相機驅動要的系統套件（rosdep 蓋不到）
sudo apt install -y libuvc-dev libgoogle-glog-dev

# 1. 底層：廠商驅動（wheeltec 分支），先建
git clone -b wheeltec --single-branch https://github.com/xiange1209/welcoming_robot.git ~/wheeltec_ws
cd ~/wheeltec_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build                                    # 廠商的 C++ 驅動，第一次要一段時間
source install/setup.bash                       # ★ 先 source 底層再建主線，主線才疊得上去

# 2. 主線。★ --recurse-submodules：frontier 探索是子模組，漏掉它 src/frontier_exploration_ros2 是空的
git clone --recurse-submodules https://github.com/xiange1209/welcoming_robot.git ~/welcoming_robot_ws
cd ~/welcoming_robot_ws
ln -s ~/welcoming_robot_ws/maprun ~/maprun      # 所有腳本都寫死 ~/maprun
# 平板網頁的前端要先建置（dist/ 不進 git，沒建 = 平板白畫面）。
# Pi 上沒有 Node.js 的話，在筆電建好再 scp 整個 dist/ 過來
(cd src/smartnav_hmi/frontend && npm ci && npm run build)
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install                  # ★ 一律從 workspace 頂層建置
source install/setup.bash                       # 之後用 maprun/env.sh；主線的 setup.bash 會自動帶上 ~/wheeltec_ws
```

17 個廠商套件裡我們直接用到的是底盤 `turn_on_wheeltec_robot`、光達 `lslidar_driver`、相機 `astra_camera`
（連同它們依賴的 `serial`、`wheeltec_robot_msg`、`wheeltec_robot_urdf`、`lslidar_msgs`、`astra_camera_msgs`），
其餘照車上現況一起建，不影響執行。

> 現在車上的放法不一樣：廠商驅動在 `~/welcoming_robot_ws/src/wheeltec_ws/`，跟我們的套件同一次 build
> （2026-09-25 查 Pi 備份確認，與 `wheeltec` 分支 528 個檔案逐一相同）。那樣也能跑，**不用搬**；
> 只是新裝的一律照上面分兩個 workspace。

## 兩人協作（都在 master 上）

| 要做的事 | 怎麼做 |
|---|---|
| 開始工作前 | `git pull --rebase`；子模組有更新時再 `git submodule update --init` |
| frontier 子模組 | 指向我們的 fork [`xiange1209/frontier_exploration_ros2`](https://github.com/xiange1209/frontier_exploration_ros2/tree/smartnav-patches) 的 `smartnav-patches` 分支（上游 + 看門狗修補，說明見 `patches/`）。**2026-09-25 以前 clone 的要做一次**：`git pull` 之後依序執行 `git submodule sync`、`git submodule update --init`（分兩行打：Windows PowerShell 5.1 不認 `&&`），否則還是上游原版（`pack_car_code.py` 會擋） |
| 推上去 | `git push origin master`。被拒絕＝對方先推了 → 先 `git pull --rebase` 再推。**不要 `--force`** |
| 大一點的改動 | 開**英文名**的分支（例 `feature/auto-mapping`），做完跟對方說一聲再合進 master；分支**不刪** |
| 改廠商驅動 | 到 `~/wheeltec_ws/`（`wheeltec` 分支的 clone）裡改、commit、`git push origin wheeltec`。**廠商碼不要進 master** |
| 不進 git（已 gitignore） | `build/ install/ log/`、`frontend/dist/`、`node_modules/`、`.smartnav/secrets/`（Telegram token）、`.smartnav/face_database/` |
| 在 Windows 上 | 子模組會被系統的 `autocrlf` 改成 CRLF，上車就壞 → `git -C src/frontier_exploration_ros2 config core.autocrlf false` 後重新簽出（`pack_car_code.py` 會擋並給完整指令） |

```bash
# 建圖
./maprun/run_nav_cc.sh mapping true

# 定位＋導航（★ 兩個參數都要給，只給第一個會誤啟探索模式）
./maprun/run_nav_cc.sh localization false
```

## 開發時必須知道的幾件事

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

5. **重複的節點會靜默污染數據。** 一天內重啟幾次導航之後，實測機器上同時有
   四份卡住偵測器、三份雷達過濾節點；多份過濾節點**同時發布過濾後的雷達**，
   定位收到交錯訊息，吻合度掉到 88%——而畫面上一切正常。
   `run_nav_cc.sh` 現在會在啟動前偵測並拒絕啟動（要強制請設 `FORCE_NAV=1`）。

6. **日誌不要用 `>` 覆蓋。** 8/17 當天八趟成功導航的 log 全被下一次重啟蓋掉，
   數字留在報告裡但原始證據沒了。現在改成帶時間戳的檔＋符號連結指向最新。

7. **`ros2 param get` 不算驗證，要量行為。** 9/17 相機以 `color_fps:=15` 啟動，
   `ros2 param get` 回 15、驅動的啟動 log 也印「640x480@15Hz」，
   但 `ros2 topic hz` 實測 **29.64 Hz**——那次「降 fps 省 CPU」的改動
   **在這台相機上從未生效**。參數伺服器存了值、log 印了值，
   都不代表硬體照做；只有量到的輸出才算數。

## 開發文件

本專案的內部開發文件（實機日交接記錄、硬體驗收清單、廠商韌體分析、
車上 AI agent 的操作手冊）**不隨此儲存庫發布**，隨部署包提供給實際操作的人。

原因有二：一是那些文件記錄的是逐日的除錯過程與未完成項目，對外部讀者
沒有參考價值；二是韌體分析的對象是廠商未公開發布的原始碼。

這份 README 已涵蓋對外部讀者有意義的部分——系統架構、實測數據、
設計取捨的理由，以及踩過的坑。

## 授權

MIT（見 [LICENSE](LICENSE)）。
`wheeltec` 分支裡的 WHEELTEC 廠商程式碼依其原始授權條款。
