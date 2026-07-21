# 智慧銀行 VIP 迎賓與安全通報系統

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue)
![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu-24.04-orange)
![Python 3.12](https://img.shields.io/badge/Python-3.12-green)

大學部畢業專題。一台 **WHEELTEC 阿克曼實車**在銀行大廳擔任迎賓機器人：
用人臉辨識分辨來客身分（GUEST／VIP／ADMIN）、以語音與 LLM 對話、可導航帶位，
平板即時顯示影像、身分與對話。

本儲存庫是**硬體層**（WHEELTEC 廠商套件、建置腳本、專案文件）。
應用層在同學 swient 的 [smartnav-bot](https://github.com/swient/smartnav-bot)，兩者疊加執行。

> **首次接手請先讀 [`docs/專案總覽.md`](docs/專案總覽.md)**——全貌、事件流、名詞、現況與鐵則都在那裡。

## 📋 目錄

- [專案結構](#專案結構)
- [系統架構](#系統架構)
- [人機介面](#人機介面fastapi)
- [環境需求](#環境需求)
- [快速開始](#快速開始)
- [目前狀態](#目前狀態)
- [實測數據](#實測數據)
- [已知限制](#已知限制)
- [文件索引](#文件索引)

## 專案結構

```
welcoming_robot/
├── smartnav_ws/
│   └── smartnav-bot/              swient/smartnav-bot：應用層 7 套件
│       └── src/
│           ├── smartnav_msgs/           介面定義（msg / srv / action）
│           ├── smartnav_vision/         人臉向量擷取（InsightFace）
│           ├── smartnav_brain/          身份驗證與使用者管理
│           ├── smartnav_audio/          喚醒詞、ASR、TTS、播放（sherpa-onnx）
│           ├── smartnav_llm/            LLM 對話 Agent（LangChain + Ollama）
│           ├── smartnav_navigation/     地圖／地點／導航服務（nav2）
│           └── smartnav_hmi/            平板網頁介面（FastAPI）
└── wheeltec_ws/
    └── welcoming_robot/           本儲存庫：硬體層 17 個廠商套件
        ├── src/                         底盤、雷達、相機、麥克風陣列、nav2 參數…
        ├── scripts/                     建置與實驗工具
        └── docs/                        專案文件
```

**兩個 workspace 分開建置、疊加載入**（先 source 硬體層，再 source 應用層）。

## 系統架構

```text
┌────────────────────────────┐   HTTP :11434   ┌────────────────────────┐
│ Raspberry Pi 4B (8GB)      │ ───────────────→│ Windows 筆電            │
│ Ubuntu 24.04 / ROS 2 Jazzy │                 │ RTX 3050 4GB           │
│ 跑全部 ROS 2 節點           │←─────────────── │ Ollama (qwen2.5:3b)    │
└──────────┬─────────────────┘    串流回覆      └────────────────────────┘
           │ USB
   ┌───────┴────────┬──────────────┬──────────────┐
   │ senior_akm 底盤 │ N10 光達      │ ASTRA S 相機  │   平板瀏覽器 → :8080
   └────────────────┴──────────────┴──────────────┘
```

**Pi 跑全部節點、筆電只當 Ollama 推理伺服器**——所有人臉辨識在本機完成（隱私＋低延遲）。

### 事件流

```text
image_raw ─→ face_embedding_node ─→ face_embedding ─→ user_auth_node
   │            (InsightFace 512D)      (向量，不含身分)      (比對資料庫)
   │                                                             │
   │                                                             ▼
   │                                                      user_identity
   │                                              (姓名／類型／相似度／是否認出)
   │                                                             │
   └─────────────────────────────────────────────────────────────┴─→ HMI 顯示

麥克風 ─→ voice_trigger ─→ speech_recognizer ─→ user_text
          (喚醒詞＋VAD)        (ASR)                  │
                                                      ▼
                                        llm_service_node（可呼叫導航工具）
                                                      │
                              llm_response / llm_stream / speech_text
                                                      │
                              speech_synthesizer ─→ voice_playback ─→ 喇叭
```

## 人機介面（FastAPI）

平板瀏覽器開 `http://<Pi的IP>:8080`，**單一服務**提供全部功能：

| 端點 | 用途 |
|---|---|
| `GET /` | 網頁：身分橫幅、即時影像、對話流、系統狀態 |
| `GET /video` | MJPEG 影像串流（`<img>` 直接吃，前端零函式庫） |
| `WS /ws` | 狀態即時推播 |
| `GET /api/state` · `/api/health` | 狀態快照與健康檢查 |
| `POST /api/register` | 觸發人臉註冊 |

> **為什麼改用 FastAPI**：舊版要同時跑 rosbridge(9090)＋靜態網頁(8081)＋web_video_server(8080)
> 三個服務，任一沒起來畫面就壞（實際踩過：launch 漏帶 web_video_server，平板永遠停在
> 「相機畫面載入中…」）。現在收斂成一個節點、一個埠。

## 環境需求

| 項目 | 需求 |
|---|---|
| Raspberry Pi 4B（8GB） | Ubuntu 24.04.4、ROS 2 Jazzy、`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` |
| Windows 筆電 | Ollama（`qwen2.5:3b`），需設 `OLLAMA_HOST=0.0.0.0` 才能被 Pi 連到 |
| 車體 | WHEELTEC senior_akm 阿克曼底盤＋N10 光達＋Orbbec ASTRA S 深度相機 |
| Python 相依 | `insightface`、`onnxruntime`、`sherpa-onnx`、`langchain`、`fastapi`、`uvicorn` |

## 快速開始

完整步驟（相依安裝、numpy 衝突處理、各節點啟動）→ [`docs/部署與執行.md`](docs/部署與執行.md)

```bash
# 環境（每個新終端都要，順序不能反）
source /opt/ros/jazzy/setup.bash
source ~/welcoming_robot/wheeltec_ws/welcoming_robot/install/setup.bash   # 硬體層
source ~/welcoming_robot/smartnav_ws/smartnav-bot/install/setup.bash      # 應用層
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 相機（只開彩色——同時開深度會超過 USB 2.0 頻寬導致彩色斷流）
ros2 launch astra_camera astra.launch.xml \
  enable_depth:=false enable_point_cloud:=false enable_ir:=false color_fps:=15

# 人臉辨識鏈（Pi 無 CUDA，enable_gpu 必須 false）
ros2 run smartnav_vision face_embedding --ros-args \
  -p enable_gpu:=false -r /image_raw:=/camera/color/image_raw
ros2 run smartnav_brain user_auth --ros-args -r /image_raw:=/camera/color/image_raw

# 平板介面
ros2 launch smartnav_hmi hmi.launch.py image_capture_topic:=/camera/color/image_raw

# LLM（筆電先開好 Ollama 並開放區網）
ros2 run smartnav_llm llm_service --ros-args \
  -p ollama_base_url:=http://<筆電IP>:11434 -p model_name:=qwen2.5:3b
```

### 註冊人臉

```bash
ros2 service call /register_face smartnav_msgs/srv/RegisterFace \
  "{user_name: '陳佳憲', user_type: {type: 1}, description: '專題負責人', num_samples: 10}"
```

呼叫後**立刻站到相機前並持續站著**（約 20~30 秒），稍微轉頭讓樣本多樣化。
`user_type`：0=GUEST、1=VIP、2=ADMIN。

> **正式展示務必用多樣本**——實測單張照片註冊的辨識置信度距門檻僅 0.002（隨時誤判），
> 10 樣本可拉到 0.071。

### 底盤操作

阿克曼車：後輪驅動、前輪只打角、**不能原地旋轉**。

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p repeat_rate:=20.0
#   i 前進   , 後退   u 前進+左轉   o 前進+右轉   k 停止
#   ★ j / l 是原地旋轉，阿克曼車做不到，按了沒反應是正確的
#   ★ repeat_rate 必加，且送出指令後約 10 秒輪子才會動（STM32 韌體保護）
```

## 目前狀態

| 分類 | 內容 |
|---|---|
| **已驗證**（實機、有輸出證據） | 底盤（含遙控）、N10 光達 10.06 Hz、ASTRA S 相機 29.6 Hz、SLAM 建圖存檔、人臉辨識端到端（10 樣本註冊 65 次零誤判） |
| **已實作待驗證** | FastAPI HMI、`user_identity` 發布——**尚未在 Pi 上建置或執行過** |
| **未實作** | 銀行場景劇本（迎賓詞／通報／帶位／FAQ）、Telegram 通報、麥克風陣列語音、導航帶位端到端、巡邏模式 |

逐項與驗收步驟 → [`docs/驗證清單.md`](docs/驗證清單.md)

> ⚠️ 已驗證項目是在 **2026-07-21 重整前**的程式碼上取得的。硬體結論（光達型號、
> USB 頻寬、底盤行為、資源瓶頸）仍然成立；軟體行為需在新結構上重新驗證。

## 實測數據

### 感測器與資源

| 項目 | 結果 |
|---|---|
| N10 光達 | 10.06 Hz、std dev 0.0013s、360°（±π）、約 450 點 |
| ASTRA S 彩色（只開彩色） | **29.6 Hz**；同時開深度時彩色會完全斷流（USB 2.0 頻寬不足） |
| 關閉深度串流的效益 | 彩色 20.8→29.6 Hz、astra CPU 175%→71%、load average 19.4→4.45 |
| 人臉辨識（10 樣本註冊） | 65 次 100% 正確、置信度 0.671~0.875、平均 0.751、標準差 0.057 |
| 人臉辨識（單張照片註冊） | 0.602~0.723，距門檻僅 0.002，**有誤判** |
| Pi4 資源（全節點運行） | load average 19.4（4 核心超載 4.8 倍），記憶體僅 40% → **瓶頸在 CPU** |

### LLM 模型延遲（筆電 RTX 3050 4GB，2026-07-02）

方法：`scripts/benchmark_llm_models.py`，每模型 1 次暖機＋5 次正式，串流 `/api/chat`，
`temperature 0.1`、`num_ctx 512`。

| 模型 | 首次輸出平均 (ms) | 總生成平均 (ms) | 成功率 |
|---|---:|---:|---:|
| **qwen2.5:3b** | **2218** | **3076** | 100% |
| gemma3:4b | 2303 | 4700 | 100% |
| gemma4:e2b | 42235 | 47909 | 100% |
| gemma4:e4b | 95066 | 107123 | 100% |

gemma4 系列模型檔（7.2GB／9.6GB）遠超 4GB VRAM，被卸載到 CPU 而慢 20~40 倍，
**不適合即時互動**。結論：用 `qwen2.5:3b`。

### LLM 工具呼叫（qwen2.5:3b，2×2 因子實驗）

| | 3 個工具 | 10 個工具 |
|---|:---:|:---:|
| 無串流回呼 | ✓ | ✗ |
| 有串流回呼 | ✓ | ✗ |

**決定性變因是工具數量，與是否串流無關。** 小模型綁超過約 5 個工具就選不出正確工具。

## 已知限制

1. **numpy 版本衝突**：`insightface`／`onnxruntime` 會裝進 numpy 2.x，而 apt 的 `cv_bridge`
   是對 numpy 1.x 編譯的 C 擴充，衝突時節點一收到影像就 segfault。
   解法見 [`docs/部署與執行.md`](docs/部署與執行.md) 第二節。
   **驗證相容性必須跑到真正的 C 執行路徑，只 `import` 成功沒有意義。**
2. **USB 2.0 頻寬**：ASTRA S 同時開深度＋彩色會超過頻寬導致彩色斷流；只開彩色反而更快。
3. **同時跑兩個 `astra_camera_node` 會搶 USB**，啟動前確認節點數為 1。
4. **slam_toolbox 在 Jazzy 是 lifecycle 節點**，廠商 Humble 世代 launch 不會自動 activate，
   症狀是 `/map` 話題不存在。需手動 `ros2 lifecycle set /slam_toolbox configure/activate`。
5. **`--symlink-install` 後不可改名或搬移 workspace**，否則 install 內連結全懸空
   （`ls` 看得到但 `test -f` 失敗）。
6. **兩條導航鏈不可混用**：WHEELTEC 鏈全程用 `odom_combined` frame（EKF 輸出），
   `smartnav_navigation`（模擬）用 `odom`。
7. **`frontier_exploration_ros2` 未內含**：`brain.launch.py` 需要它，須另外 clone。
8. **銀行場景劇本尚未移植**——重整前有（VIP 迎賓詞、黑名單通報、帶位、FAQ），
   新結構下需重新實作，這是 12 月發表前的第一優先。

## 文件索引

| 文件 | 內容 |
|---|---|
| [`docs/專案總覽.md`](docs/專案總覽.md) | 入口：全貌、事件流、名詞、現況、鐵則 |
| [`docs/架構參考.md`](docs/架構參考.md) | 套件、節點、話題、服務、參數速查 |
| [`docs/部署與執行.md`](docs/部署與執行.md) | 建置、相依、啟動、常見問題 |
| [`docs/驗證清單.md`](docs/驗證清單.md) | 已驗證／待驗證／未實作逐項與驗收步驟 |

外部參考：[swient/smartnav-bot](https://github.com/swient/smartnav-bot)（應用層上游）·
[ROS 2 Jazzy](https://docs.ros.org/en/jazzy/) · [Nav2](https://navigation.ros.org/) ·
[InsightFace](https://github.com/deepinsight/insightface) ·
[sherpa-onnx](https://github.com/k2-fsa/sherpa-onnx) · [Ollama](https://ollama.com/)

## 授權

本專案採用 [MIT License](LICENSE)。開發規範見 [`.clinerules`](.clinerules)。
廠商套件（`src/` 下的 WHEELTEC 程式碼）依其原始授權條款。
