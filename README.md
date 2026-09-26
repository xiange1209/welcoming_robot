# 具人臉辨識與語音對話功能之迎賓機器人 — 實機開發主線

![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue)
![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu-24.04-orange)
![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%204B%208GB-red)
![循跡精度](https://img.shields.io/badge/循跡精度-3.42%20cm-brightgreen)
![人臉門檻](https://img.shields.io/badge/人臉辨識門檻-0.70%20(實測定出)-brightgreen)

大學部畢業專題。WHEELTEC 阿克曼實車 ＋ 人臉辨識 ＋ 語音對話 ＋ 自主導航。

> **`master` 是唯一的開發主線**（2026-09-24 起，原本的 `車子` 分支已併入並刪除），兩位組員都在這裡開發。
> 車上要跑還需要 **`wheeltec` 分支**：17 個 WHEELTEC 廠商驅動（底盤、光達、相機），
> 另外 clone 成 `~/wheeltec_ws` 當底層 workspace，我們的疊在上面（見「部署到 Pi」）。兩個分支歷史各自獨立，**不合併**。

---

## 目錄

repo 根目錄**就是** ROS 2 workspace（在這裡 `colcon build`）。

```
.
├── src/
│   ├── smartnav_msgs/           介面定義（msg / srv / action）
│   ├── smartnav_vision/         人臉向量擷取（InsightFace 512D）
│   ├── smartnav_brain/          身份驗證、使用者管理、銀行迎賓劇本
│   ├── smartnav_audio/          喚醒詞、ASR（sherpa-onnx）、雙麥克風降噪
│   ├── smartnav_llm/            LLM 對話 Agent（LangChain + Ollama）
│   ├── smartnav_navigation_cc/  ★ 導航重寫版：SLAM、AMCL、教導-重現路徑、自動探索
│   ├── smartnav_navigation/     舊版（模擬用，frame 不同不可混用）
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

## 開發文件

本專案的內部開發文件（實機日交接記錄、硬體驗收清單、廠商韌體分析、
車上 AI agent 的操作手冊）**不隨此儲存庫發布**，隨部署包提供給實際操作的人。

原因有二：一是那些文件記錄的是逐日的除錯過程與未完成項目，對外部讀者
沒有參考價值；二是韌體分析的對象是廠商未公開發布的原始碼。

這份 README 已涵蓋對外部讀者有意義的部分——系統架構、實測數據、
設計取捨的理由，以及踩過的坑。

## 進度總覽

### 2026-09-25 主線合併與自動建圖修正（待上車驗證）

- 9/23 組員重構（目錄攤平成 `src/`、HMI 改 React＋模組化 FastAPI、frontier 改子模組），9/24 合併成單一 `master`。
- 自動建圖：逾時與停滯改為**存圖**（舊版逾時直接丟圖）；卡住偵測在探索中讓給 explorer 的看門狗處理，
  避免「取消 → 又選回同一點」的無限迴圈；切換測試情境時先停導航再用正確模式重啟。
  三個角度的程式審查後補修，行為測試 53/53——**尚未上車**。
- 下次上車：平板「系統開關 → 建圖（自動探索）」跑一趟，結束後按一鍵驗證的 **V5**，它會把卡住原因分成 CPU／TF、幾何、實體三類。

### 2026-09-17 新實驗室靜態驗證

搬進學校實驗室第一天。**只做靜態項目**——相機／人臉／語音／LLM，
未建圖、未移動（電池 24.8 V）。

| 項目 | 結果 |
|---|---|
| 人臉辨識（門檻 0.70，未改） | ✅ 通過，但**註冊環境的影響比想像中大** |
| 誤接受（未註冊者被認成已註冊者） | ✅ **19 次比對 0 次誤接受**（首次有數據） |
| 多人同框 | ✅ 三人同時入鏡，**未觀察到跨人誤認** |
| LLM 對話 | ✅ 反應 **0.94 ~ 1.79 秒**，`query_datetime_tool` 呼叫成功 |
| ASR 實際辨識率 | ⚠ 阻斷已排除，但**尚未驗證** |
| 建圖與移動 | 未做 |

**★ 在哪裡註冊，就在哪裡比較準。** 同一支相機、同一天、同一份程式的乾淨對照：

| 註冊時地 | 辨識相似度 |
|---|---|
| 8/19 **在家**註冊 | 0.710 ~ 0.778 |
| 9/17 **在實驗室當場**註冊 | **0.802 ~ 0.845** |

當場註冊整體高約 **0.09**。在家註冊的那一筆，最差的幀只高出門檻 **0.010**——
**部署前要在現場重新註冊**，這不是調參數能補的。

**誤接受首次有數據。** 先前量不了，因為資料庫裡只有一個人；
現在 VIP／GUEST／BLACKLIST 各一才測得起來：未註冊者 **19 次比對 0 次誤接受**。
⚠ 但那一組是在取景不佳（臉被切在畫面下緣）的條件下量的，會壓低相似度；
另一組量到最高 **0.693**——距門檻 0.70 只差 **0.007**。**風險上界要採 0.693。**

**LLM 可用，但知識庫是空的。** 20 個段落全是「請填寫」，
而模型在**沒有工具依據時會自行編造**場地與營業資訊。
→ 填完知識庫之前，任何「櫃檯在哪」這類回答都不可信。
有工具可用時則正常——問時間會確實去呼叫 `query_datetime_tool`。

**2026-09-18 修正（本次 commit，尚未上機驗證）**

9/17 抓到的三個問題形狀完全一樣：**節點正常啟動、話題正常連接、永遠不出結果。**

1. **`smartnav_audio` 套件裡沒有 `models/` 目錄**——安裝腳本掃套件內的檔案來裝模型，
   掃不到就什麼都不裝，於是 ASR 與 VAD **同時**靜默失效。
   → 改成可覆寫的模型路徑：`asr_model_dir` / `vad_model_dir` 參數 →
   環境變數 `SMARTNAV_AUDIO_MODELS_DIR` → 套件 share → 退到 `~/models/<type>`。
   並且**找不到模型時改印 `error` 並列出找過的每一條路徑**，
   不再只印一行 warning 就繼續跑。

2. **`face_embedding_node` 的 `process_interval_sec` 不能熱改**——只在初始化讀一次，
   `ros2 param set` 會回報成功但節點照舊。
   → 補上 `add_on_set_parameters_callback`，並把**預設值 0.0 改成 1.0**：
   0.0 實測 **198% CPU、全機 `idle 0.0%`、load 14.7**；1.0 是 **127% CPU**，
   而偵測產出 **0.97 Hz** 幾乎沒掉。**舊的預設值等於預設組態即飽和。**

3. **回音過濾漏掉「開頭對、後半被 ASR 聽爛」的自我回饋**——
   TTS 出聲 → 麥克風收回去 → ASR → LLM，機器人會自己跟自己對話。
   → `_is_echo` 新增共同前綴規則（`echo_prefix_chars`，預設 6 字），11/11 案例通過。

**校準值更新**：`run_asr_cc.sh` 預設增益 **66 → 60**、模式 **`cancel` → `left`**。
兩者必須綁在一起改——零點消噪的 FIR 係數跟麥克風增益綁定，改增益就得重訓。
麥克風增益刻度實測 **1.38 dB/單位**（舊文件寫的 0.5 dB/單位是錯的）。

### 2026-08-30 現況

搬去學校實驗室前的最後一輪。導航鏈已跑得通、精度達標，剩下的是**轉向能力**：

| 項目 | 狀態 |
|---|---|
| 教導-重現循跡精度 | ✅ **3.42 cm**（624 筆） |
| 人臉辨識門檻 | ✅ 0.70（由資料定出，非猜測） |
| 雙麥克風降噪 | ✅ CER 0.0%（單聲道觸發不了 VAD） |
| 直角彎自主通過 | ❌ **幾何上就過不去**（見下） |
| 倒車循跡 | ⚠ 會往外拋，成因已定位、修法待上機 |

**核心問題：實測迴轉半徑漂到韌體規格的 1.5 倍。**
韌體設計是對稱 0.750 m，但 8/29 實測左 **1.030** / 右 **1.183** m
（8/17 量到的是左 0.944 / 右 0.751 —— 方向還反過來，這個量會漂）。

半徑一大，過直角彎需要的走廊就變寬：`w = 2R(1−1/√2) + √2·(半車寬+餘裕)`
對 R **單調遞增**。用 `maprun/corner_check_cc.py` 實算，最壞情況（控制器箝制 1.20）
需要淨寬 **0.993 m**，加上 95 百分位循跡誤差 0.175 m → **選路門檻 1.17 m**。
測試場地門口只有 0.967 m，**負餘裕**。

同一個半徑問題也解釋了倒車外拋：純追蹤能執行的最大曲率是 `1/R`，
超出的修正指令會被無聲截斷，而倒車的增益本來就只有前進的三分之一。

→ 下一步不是調控制器參數，而是**舵機中位／行程校準**：
把 R 拉回 0.750 可同時省下 25 cm 的走廊需求並拿回 58% 的修正權限。
算下來需要 PWM 1855（硬限 1890），而實車只用到 1740 —— **行程是夠的，只是沒用到**。

### 2026-08-19 實機日

單日 **19 項驗證通過**。以下每個數字都有對應的 log 或 CSV。

### 導航

| 成果 | 數據 |
|---|---|
| **教導-重現循跡精度** | 624 筆取樣，平均偏離 **3.42 cm**、最大 8.60 cm |
| 路線 B 自主導航 | **4 趟成功**，含「大廳 → 起點」**首次成功** |
| 地點停止誤差 | 0.12~0.17 m（容差 0.25 m） |
| AMCL 掃描過濾 | 地圖吻合度 71% → **90%** |

**兩個從來沒人量過的底層事實，各自解釋了一批長期症狀：**

- **底盤有低速死區**：指令低於 **0.085 m/s 馬達根本不轉**（0.05/0.07/0.08 都是 0.000 m）。
  它一次解釋了三件事——脫困做三版都沒效（脫困速度就是 0.05）、
  障礙減速 `0.15 × 0.30 = 0.045` 一減就停、以及先前歸納的
  「0 折返全勝、多折返全敗」其實是**假相關**（折返點要減速 → 掉進死區）。
  ★ 之所以難發現，是因為卡住偵測器記的是**指令值**，於是報成「像被東西卡住」。

- **規劃器用對稱半徑描述一台不對稱的車**：實測左 **0.944 m** / 右 **0.751 m**。
  `minimum_turning_radius` 是單一值，設 0.80 時右轉做得到、左轉物理上不可能。
  判準是一個 **10:0 的統計**——當天所有可行性警告全部是往左轉。
  改 0.95 後警告 **4 → 0**。
  ★ 這組半徑後來在 8/29 重量成左 1.030 / 右 1.183（**左右關係反轉**），
  所以「不對稱」是真的、但**哪一邊比較緊會漂**，引用前要重量。

- **AMCL `do_beamskip`**：大廳多一張椅子就讓四趟連續失敗（偏離 72~127 cm，
  但走廊只有 0.99 m 寬，物理上不可能）。真因是對不上地圖的光束把粒子權重帶歪，
  吻合度掉到 78.6%。開啟後車子能在有人擋路時繼續完成導航。

### 語音

雙麥克風降噪（Astra S，兩顆麥克風間距 4.3 cm）——**同音源、同一分鐘**的對照：

| 模式 | SNR | VAD 觸發 | 辨識結果 |
|---|:--:|:--:|---|
| 只用左聲道 | 6.1 dB | **0 次** | 沒有輸出 |
| 只用右聲道 | 12.5 dB | **0 次** | 沒有輸出 |
| **雙麥克風零點消噪** | — | **19 次** | **CER 0.0%** |

★ 這組數字說明**單聲道在這個環境下根本觸發不了 VAD**。
離線量到的 SNR 淨增益是 +8.2 dB（噪音 −13.6 dB、人聲只 −5.4 dB）。
★ 4.3 cm 是極小孔徑，**做不出波束、只做得出零點**——
延遲相加波束成形實測 ΔSNR 只有 −0.07 dB，等於沒用。

★ **2026-09-17 更新：上表是在家裡那個環境、增益 66 之下量的。**
零點消噪的 FIR 係數與環境和麥克風增益綁定，換場地就得重訓，
所以搬進實驗室後預設改為**增益 60 ＋ `left` 單聲道**（見上）。

其他：熱詞編碼失敗 **12 → 0**（詞表是簡體，熱詞檔原本寫繁體所以從未生效）；
模型檔改用 int8 後 encoder **315 MB → 173 MB**。整體 CER **25.6%**。

### 人臉辨識

**相似度 vs 距離**（同一人、同一光線）：

| 距離 | 相似度 | 判讀 |
|---|:--:|---|
| 50 cm | 0.725 | 太近，臉部超出取景或變形 |
| **1 m** | **0.760** | ★ **最佳** |
| 2 m | 最低 0.4434 | **低於「不同人」的 0.515 → 已開始亂認** |

→ **這台機器人的可靠辨識範圍是 1 公尺左右**，迎賓劇本應該在這個距離觸發。

`recognition_threshold` 由 **0.8（猜的）改為 0.70（由資料定出）**。

**模型選型結論：不換 buffalo_l。** 它在 MR-ALL 上比現用的 buffalo_sc 高 19.38 分
（91.25 vs 71.87），但實測單幀 **6462 ms**——即使只以 1 Hz 執行也來不及。

★ 附帶一個對報告有用的觀察：兩者在 LFW 只差 0.13 分（99.83 vs 99.70）。
**LFW 已經飽和，用它比較模型會得到「sc 已經夠好」的錯誤結論。**

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

**已經在跑的車：用打包更新**（在筆電跑 `python pack_car_code.py`，它會印出完整的 Pi 端步驟）。
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

## 授權

MIT（見 [LICENSE](LICENSE)）。
`wheeltec` 分支裡的 WHEELTEC 廠商程式碼依其原始授權條款。
