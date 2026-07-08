# 智慧銀行 VIP 迎賓與安全通報系統

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue)
![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu-24.04-orange)
![Python 3.12](https://img.shields.io/badge/Python-3.12-green)

基於 **SmartNav**（[swient/smartnav-bot](https://github.com/swient/smartnav-bot)）與 **ROS 2 Jazzy** 的智慧銀行服務機器人專題。本專案與同學的 SmartNav 導航機器人系統協作開發：以 SmartNav 的視覺、語音、LLM、導航框架為主體，加上銀行場景的擴充——VIP / 黑名單 / 訪客三類身分辨識、性別欄位、辨識事件轉自然語言進入 LLM Agent，目標場景為銀行大廳的 VIP 迎賓、訪客引導與黑名單安全通報。

硬體平台：Raspberry Pi 4（8GB）+ TurtleBot3 為感知與行動端，Windows 筆電（RTX 3050 4GB）為 LLM 運算端。

## 📋 目錄

- [主要功能](#-主要功能)
- [系統架構](#️-系統架構)
- [套件說明](#-套件說明)
- [環境需求](#-環境需求)
- [安裝與建置](#-安裝與建置)
- [快速開始](#-快速開始)
- [配置說明](#️-配置說明)
- [主題與服務](#-主題與服務)
- [LLM 模型選擇（實測）](#-llm-模型選擇實測)
- [驗證狀態](#-驗證狀態)
- [已知限制](#-已知限制)
- [故障排除](#️-故障排除)
- [文件索引](#-文件索引)

## 🎯 主要功能

### 銀行場景人臉辨識（`smartnav_vision`，本專案擴充）

- **三類身分**：`VIP` / `BLACKLIST`（黑名單）/ `VISITOR`（訪客），註冊時指定 `person_type`
- **性別欄位**：註冊時輸入 `gender`（M / F / Other），隨辨識結果一併發布
- **顏色框標示**（debug 影像）：🟩 綠框 = VIP、🟥 紅框 = 黑名單、🟦 藍框 = 訪客
- **InsightFace buffalo_sc**：512D 特徵向量，餘弦相似度比對，動態註冊免重新訓練
- **黑名單安全通報定位**：辨識到黑名單人員時發布高優先事件，供後續通報流程使用（通報動作本身尚未實作，見[已知限制](#-已知限制)）

### 辨識事件 → LLM 對話橋接（本專案擴充）

- `recognition_text_bridge_node` 將 `/face_recognition/result` 轉為自然語言句子，發布到 `/user_text`，讓 LLM Agent 能「看到」誰走進銀行

### 語音介面（`smartnav_audio`，sherpa-onnx）

- 關鍵詞喚醒（KWS + VAD）→ 語音辨識（ASR）→ 文字進 `/user_text`
- LLM 回覆文字 → 語音合成（TTS）→ 喇叭播放

### 對話式 LLM Agent（`smartnav_llm`）

- LangChain + Ollama，訂閱 `/user_text`，可呼叫 ROS 2 導航工具（建圖、切換地圖、建立/列出地點、導航、全域定位）
- Ollama 跑在筆電上，RPi4 透過區網呼叫（`ollama_base_url`）

### 地圖與導航（`smartnav_navigation`）

- 地圖服務（建圖含自動探索、列出、切換）、地點服務、Nav2 到點導航動作

### 統一啟動（`smartnav_bringup`）

- `smartnav.launch.py` 一鍵啟動 vision + audio + llm + navigation，各模組可獨立開關

## 🏗️ 系統架構

### 雙機架構

```text
┌────────────────────────────────────────────┐          ┌─────────────────────────────┐
│  Raspberry Pi 4 (8GB)                      │          │  Windows 筆電                │
│  Ubuntu 24.04 + ROS 2 Jazzy + TurtleBot3   │          │  RTX 3050 Laptop 4GB VRAM   │
│                                            │          │                             │
│  📷 相機 ─→ smartnav_vision（人臉辨識）      │          │  🤖 Ollama (:11434)          │
│  🎤 麥克風 ─→ smartnav_audio（喚醒/ASR）     │   HTTP   │     qwen2.5:3b（建議）       │
│  🧠 smartnav_llm（LangChain Agent）─────────┼─────────→│     gemma3:4b（次選）        │
│  🗺️ smartnav_navigation（地圖/導航/Nav2）    │  區網    │                             │
│  🔊 smartnav_audio（TTS/播放）              │          │  ollama_base_url:=          │
│  🚗 TurtleBot3 底盤                         │          │  http://<筆電IP>:11434       │
└────────────────────────────────────────────┘          └─────────────────────────────┘
```

- **RPi4（感知與行動端）**：跑 vision / audio / navigation / bringup，所有人臉辨識推理在本機完成（隱私 + 低延遲）
- **筆電（LLM 運算端）**：只跑 Ollama；RPi4 的 `smartnav_llm` 以 `ollama_base_url:=http://<筆電IP>:11434` 連線

### 銀行場景事件流

```text
相機 /image_raw
   │
   ▼
face_recognition_node ──(InsightFace 512D 比對 VIP/黑名單/訪客資料庫)
   │
   ▼
/face_recognition/result (smartnav_msgs/RecognitionResult)
   │                                  │
   ▼                                  └─→ /face_recognition/debug_image（可選，含顏色框）
recognition_text_bridge_node ──(轉自然語言，例如「VIP 陳先生出現在門口」)
   │
   ▼
/user_text (std_msgs/String) ←──────── smartnav_audio 語音辨識結果也走同一話題
   │
   ▼
smartnav_llm Agent（LangChain + Ollama）
   │
   ├─→ 呼叫導航工具（navigate / list_waypoints / ...）→ smartnav_navigation → Nav2 → TurtleBot3
   └─→ /speech_text → speech_synthesizer → /audio_out → voice_playback（語音回覆）
```

## 📦 套件說明

| 套件 | 類型 | 功能 |
| --- | --- | --- |
| [`smartnav_msgs`](src/smartnav_msgs/) | ament_cmake | 訊息/服務/動作定義。本專案擴充：`RecognitionResult.msg`（person_uuid / person_name / gender / person_type / confidence / timestamp / bbox）、`RegisterFace.srv`（含 gender、person_type）；另有 `AudioData.msg`、`MapInfo.msg`、`WaypointInfo.msg`、地圖/地點服務與 `CreateMap` / `GlobalLocalization` / `Navigate` 動作 |
| [`smartnav_vision`](src/smartnav_vision/) | ament_python | 人臉辨識核心。節點：`face_recognition_node`（即時辨識）、`face_registration_node`（線上註冊）、`recognition_text_bridge_node`（辨識結果轉文字）。模組：`face_engine.py`（InsightFace 封裝 + 顏色框）、`database_manager.py`、`face_utils.py` |
| [`smartnav_audio`](src/smartnav_audio/) | ament_python | sherpa-onnx 語音管線。節點：`voice_trigger`（KWS 喚醒 + VAD）、`speech_recognizer`（ASR）、`speech_synthesizer`（TTS）、`voice_playback`（播放） |
| [`smartnav_llm`](src/smartnav_llm/) | ament_python | LangChain + Ollama 對話式 Agent（`llm_service_node`），工具集：create_map / list_maps / switch_map / create_waypoint / list_waypoints / navigate / global_localization。含 `launch/llm.launch.py`、`config/system_prompt.txt` |
| [`smartnav_navigation`](src/smartnav_navigation/) | ament_python | 地圖/地點/導航（2026-07-01 由原 smartnav_brain 重構拆出）。節點：`map_service_node`、`waypoint_service_node`、`navigation_action_node`。Launch：`brain.launch.py`（三節點 + AMCL + slam_toolbox + frontier_explorer + map_server）、`nav2.launch.py`（完整 Nav2 棧 + RViz）。設定：`config/burger.yaml`、`config/empty_map` |
| [`smartnav_brain`](src/smartnav_brain/) | ament_python | 空殼套件，保留給未來 orchestrator / 多模態決策（如黑名單通報策略） |
| [`smartnav_hmi`](src/smartnav_hmi/) | ament_python | 空殼套件，保留給未來人機介面 |
| [`smartnav_bringup`](src/smartnav_bringup/) | ament_cmake | 統一啟動。`launch/smartnav.launch.py` 一次啟動 vision + audio + llm + navigation |

工作區以外的輔助檔案：

| 路徑 | 用途 |
| --- | --- |
| `scripts/benchmark_llm_models.py` | LLM 延遲基準測試（TTFT / 總時長，串流 Ollama `/api/chat`，輸出 Markdown 表與 JSON） |
| `scripts/download_audio_models.sh` | 下載 sherpa-onnx KWS / ASR / VAD / TTS 模型 |
| `Modelfile_qwen` / `Modelfile_gemma3` / `Modelfile_gemma4e2b` / `Modelfile_gemma4e4b` | Ollama 銀行場景模型設定檔 |
| `benchmark_llm_results.json` | 2026-07-02 LLM 延遲實測原始數據 |

## 🔧 環境需求

### RPi4（感知端）

- **硬體**：Raspberry Pi 4（8GB RAM）、相機、麥克風/喇叭、TurtleBot3（導航功能需要）
- **作業系統**：Ubuntu 24.04 LTS（Noble Numbat）
- **ROS 2**：Jazzy Jalisco
- **Python**：3.12+
- **主要 Python 套件**：`insightface`、`opencv-python`、`onnxruntime`、`numpy`、`sherpa-onnx`、`langchain`、`langchain-ollama`

### 筆電（LLM 端）

- **GPU**：建議 4GB VRAM 以上（實測平台為 RTX 3050 Laptop 4GB）
- **軟體**：[Ollama](https://ollama.com/)（實測版本 0.24.0）
- 筆電只需要 Ollama，不需要安裝 ROS 2

## 📥 安裝與建置

### 1. RPi4：安裝 ROS 2 Jazzy

```bash
# 新增 ROS 2 GPG 金鑰與套件庫
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

sudo apt update
sudo apt install ros-jazzy-desktop python3-colcon-common-extensions python3-rosdep
```

### 2. 複製專案

```bash
cd ~
git clone https://github.com/xiange1209/welcoming_robot.git smartnav_ws
cd smartnav_ws
```

> ⚠️ **frontier_exploration_ros2 需另外 clone**：`brain.launch.py` 的自動探索建圖依賴
> [mertgulerx/frontier_exploration_ros2](https://github.com/mertgulerx/frontier_exploration_ros2)。
> 同學的 repo 以 git submodule 引入，本 repo 尚未加入 submodule，請手動：
>
> ```bash
> git clone https://github.com/mertgulerx/frontier_exploration_ros2 src/frontier_exploration_ros2
> ```

### 3. 安裝依賴

```bash
source /opt/ros/jazzy/setup.bash
sudo rosdep init 2>/dev/null; rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Python 依賴（rosdep 未涵蓋者）
pip install insightface onnxruntime opencv-python sherpa-onnx langchain langchain-ollama

# 下載語音模型（KWS / ASR / VAD / TTS）
bash scripts/download_audio_models.sh
```

### 4. 建置工作區

```bash
cd ~/smartnav_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 或只建置部分套件
colcon build --packages-select smartnav_msgs smartnav_vision --symlink-install
```

### 5. 設定環境

```bash
echo "source ~/smartnav_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 6. 筆電：準備 Ollama 模型

```powershell
# 在 Windows 筆電上
ollama pull qwen2.5:3b            # 實測最快，建議使用
ollama create gemma3-bank -f Modelfile_gemma3   # 次選（可選）

# 讓 Ollama 監聽區網（預設只綁 localhost）
$env:OLLAMA_HOST = "0.0.0.0"
ollama serve
```

## 🚀 快速開始

### 方式 1：bringup 一鍵啟動（RPi4）

```bash
source ~/smartnav_ws/install/setup.bash

# ollama_base_url 指向筆電的區網 IP；model_name 建議 qwen2.5:3b（見下方實測）
ros2 launch smartnav_bringup smartnav.launch.py \
  ollama_base_url:=http://192.168.1.xxx:11434 \
  model_name:=qwen2.5:3b
```

常用 launch 參數（皆可用 `參數名:=值` 覆寫）：

| Launch 參數 | 預設值 | 說明 |
| --- | --- | --- |
| `image_topic` | `/image_raw` | 相機影像話題 |
| `enable_gpu` | `true` | 視覺推理是否使用 GPU（RPi4 上建議 `false`，走 CPU） |
| `ollama_base_url` | `http://localhost:11434` | Ollama API 位址，請改成筆電區網 IP |
| `model_name` | `gemma4:e2b` | smartnav_llm 使用的 Ollama 模型（實測建議改為 `qwen2.5:3b`） |
| `use_sim_time` | `false` | 是否使用模擬時間（Gazebo） |
| `enable_vision` / `enable_audio` / `enable_llm` / `enable_navigation` | `true` | 各模組開關，硬體未就緒可個別關閉 |
| `enable_nav2` | `false` | 是否啟動完整 Nav2 棧 + RViz（需 TurtleBot3 或模擬環境） |

範例——只測「人臉辨識 → LLM」串接，不啟動語音與導航：

```bash
ros2 launch smartnav_bringup smartnav.launch.py \
  ollama_base_url:=http://192.168.1.xxx:11434 \
  model_name:=qwen2.5:3b \
  enable_audio:=false enable_navigation:=false
```

### 方式 2：個別啟動模組

```bash
# 人臉辨識節點（RPi4 走 CPU）
ros2 run smartnav_vision face_recognition --ros-args \
  -p image_topic:=/image_raw -p enable_gpu:=false -p publish_debug_image:=true

# 辨識結果轉文字橋接
ros2 run smartnav_vision recognition_text_bridge

# LLM Agent（單獨啟動）
ros2 launch smartnav_llm llm.launch.py \
  ollama_base_url:=http://192.168.1.xxx:11434 model_name:=qwen2.5:3b

# 地圖/地點/導航服務
ros2 launch smartnav_navigation brain.launch.py
```

### 註冊 VIP / 黑名單人臉

請本人站在相機前，呼叫註冊服務（會連續擷取 `num_samples` 張臉部樣本）：

```bash
# 先啟動註冊節點（bringup 已包含）
ros2 run smartnav_vision face_registration

# 註冊 VIP
ros2 service call /face_registration/register smartnav_msgs/srv/RegisterFace \
  "{person_name: '陳佳憲', gender: 'M', person_type: 'VIP', num_samples: 10}"

# 註冊黑名單
ros2 service call /face_registration/register smartnav_msgs/srv/RegisterFace \
  "{person_name: '可疑人士A', gender: 'F', person_type: 'BLACKLIST', num_samples: 10}"

# 註冊後刷新辨識快取
ros2 service call /face_recognition/refresh_cache std_srvs/srv/Empty
```

### 觀察辨識結果

```bash
# 結構化辨識結果
ros2 topic echo /face_recognition/result

# 轉成自然語言後、送進 LLM 的文字
ros2 topic echo /user_text

# 帶顏色框的除錯影像（需 publish_debug_image:=true）
ros2 run rqt_image_view rqt_image_view /face_recognition/debug_image
```

## ⚙️ 配置說明

### `face_recognition_node` 參數

| 參數名稱 | 型態 | 預設值 | 說明 |
| --- | --- | --- | --- |
| `image_topic` | `string` | `/image_raw` | 輸入相機影像話題 |
| `model_name` | `string` | `buffalo_sc` | InsightFace 模型名稱（支援 `buffalo_sc`、`buffalo_l`） |
| `confidence_threshold` | `double` | `0.5` | 人臉檢測信心閾值（0.0–1.0） |
| `enable_gpu` | `bool` | `true` | 是否啟用 GPU 加速（RPi4 請設 `false`） |
| `recognition_threshold` | `double` | `0.6` | 人臉辨識相似度閾值（0.0–1.0） |
| `publish_debug_image` | `bool` | `false` | 是否發布帶顏色框的除錯影像 |
| `result_republish_interval` | `double` | `15.0` | 同一身分持續出現時，重新發布結果的最小間隔秒數（去抖動，避免 LLM 被同一人的事件洗版） |

### `recognition_text_bridge_node` 參數

| 參數名稱 | 型態 | 預設值 | 說明 |
| --- | --- | --- | --- |
| `result_topic` | `string` | `/face_recognition/result` | 訂閱的辨識結果話題 |
| `user_text_topic` | `string` | `/user_text` | 輸出給 smartnav_llm 的文字話題 |

### `llm_service_node` 參數

| 參數名稱 | 型態 | 預設值 | 說明 |
| --- | --- | --- | --- |
| `ollama_base_url` | `string` | `http://localhost:11434` | Ollama API 位址（雙機架構請改筆電 IP） |
| `model_name` | `string` | `gemma4:e2b` | Ollama 模型（實測建議 `qwen2.5:3b`） |
| `temperature` | `double` | `0.0` | 取樣溫度 |

### 使用 YAML 參數檔

```yaml
face_recognition_node:
  ros__parameters:
    image_topic: /image_raw
    model_name: buffalo_sc
    enable_gpu: false
    recognition_threshold: 0.6
    publish_debug_image: true
```

```bash
ros2 run smartnav_vision face_recognition --ros-args --params-file face_recognition_config.yaml
```

## 📡 主題與服務

### 主要話題

| 話題名稱 | 訊息型態 | 發布者 → 訂閱者 | 說明 |
| --- | --- | --- | --- |
| `/image_raw` | `sensor_msgs/Image` | 相機驅動 → face_recognition / face_registration | 相機影像串流 |
| `/face_recognition/result` | `smartnav_msgs/RecognitionResult` | face_recognition → recognition_text_bridge | 辨識結果（uuid / 姓名 / 性別 / 身分類型 / 信心度 / bbox） |
| `/face_recognition/debug_image` | `sensor_msgs/Image` | face_recognition → rqt 等 | 帶顏色框除錯影像（🟩VIP 🟥黑名單 🟦訪客，可選） |
| `/user_text` | `std_msgs/String` | recognition_text_bridge、speech_recognizer → llm_service | 進入 LLM Agent 的統一文字入口 |
| `/llm_response` / `/llm_stream` | `std_msgs/String` | llm_service → | LLM 完整回覆 / 串流片段 |
| `/speech_text` | `std_msgs/String` | llm_service → speech_synthesizer | 要合成語音的文字 |
| `/audio_in` | `smartnav_msgs/AudioData` | voice_trigger → speech_recognizer | 喚醒後的語音片段 |
| `/audio_out` | `smartnav_msgs/AudioData` | speech_synthesizer → voice_playback | 合成的語音資料 |
| `/voice_triggered` | `std_msgs/Bool` | voice_trigger → | 喚醒觸發訊號 |
| `/playback_status` | `std_msgs/Bool` | voice_playback → speech_recognizer | 播放狀態（避免自己聽自己） |

### 服務與動作

| 名稱 | 型態 | 提供者 | 說明 |
| --- | --- | --- | --- |
| `/face_registration/register` | `smartnav_msgs/srv/RegisterFace` | face_registration | 註冊人臉（姓名 / 性別 / VIP 或 BLACKLIST / 樣本數） |
| `/face_recognition/refresh_cache` | `std_srvs/srv/Empty` | face_recognition | 重新載入資料庫並刷新特徵向量快取 |
| `list_maps` / `switch_map` | `smartnav_msgs/srv/*` | map_service | 列出 / 切換地圖 |
| `create_waypoint` / `list_waypoints` / `get_waypoint` | `smartnav_msgs/srv/*` | waypoint_service | 地點管理 |
| `create_map`（action） | `smartnav_msgs/action/CreateMap` | map_service | 建圖（含 frontier 自動探索） |
| `global_localization`（action） | `smartnav_msgs/action/GlobalLocalization` | waypoint_service | 全域定位 |
| `navigate`（action） | `smartnav_msgs/action/Navigate` | navigation_action | 導航到指定地點 |

## 🤖 LLM 模型選擇（實測）

**測試方法**：`scripts/benchmark_llm_models.py`，Windows 筆電 RTX 3050 Laptop 4GB VRAM、Ollama 0.24.0；每模型 1 次暖機 + 5 次正式，串流 `/api/chat`，銀行場景 system prompt，`temperature 0.1`、`num_ctx 512`。測試日期 **2026-07-02**，原始明細見 [`benchmark_llm_results.json`](benchmark_llm_results.json)。

| 模型（Ollama tag） | 基底模型 | 首次輸出平均 (ms) | 總生成平均 (ms) | 最快 (ms) | 最慢 (ms) | 成功率 |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| **qwen2.5:3b** | qwen2.5:3b | **2218** | **3076** | 3017 | 3196 | 100% |
| gemma3-bank | gemma3:4b | 2303 | 4700 | 4583 | 4755 | 100% |
| gemma4e2b-bank | gemma4:e2b | 42235 | 47909 | 44838 | 50206 | 100% |
| gemma4e4b-bank | gemma4:e4b | 95066 | 107123 | 100231 | 123839 | 100% |

**關鍵解讀**：

- gemma4 系列模型檔（e2b 7.2GB / e4b 9.6GB）**遠超 4GB VRAM**，Ollama 將大部分權重卸載到 CPU，導致比可全載入 VRAM 的模型慢 **20~40 倍**，完全不適合即時互動。
- gemma3:4b（約 4GB）與 qwen2.5:3b（約 1.9GB）能塞進 VRAM；在此硬體上 **qwen2.5:3b 最快**（總生成約 3 秒），gemma3:4b 次之（約 4.7 秒）。
- **結論**：本專題雙機架構建議 `model_name:=qwen2.5:3b`。注意 `smartnav_llm` 的預設值仍是 `gemma4:e2b`（沿用上游 SmartNav 預設），在本硬體上務必以 launch 參數覆寫。

## ✅ 驗證狀態

本節誠實區分「已實機驗證」與「僅完成程式碼、尚未實機驗證」的部分。

### 已驗證

| 項目 | 驗證方式與結果 |
| --- | --- |
| RPi4 即時人臉檢測 | InsightFace buffalo_sc 於 RPi4 實機達 **15+ FPS**（於舊架構驗證；ROS 2 版使用相同引擎與模型） |
| 筆電 LLM 延遲基準 | 上方實測表格，2026-07-02，`scripts/benchmark_llm_models.py` 全數 100% 成功 |

### 已實作、待實機驗證

- **整個 ROS 2 workspace 尚未在 RPi4 實機 `colcon build` 與運行過**，以下皆屬此類：
  - vision → recognition_text_bridge → LLM 的完整事件流
  - `smartnav_bringup` 一鍵啟動
  - `smartnav_audio` 語音管線（喚醒 / ASR / TTS / 播放）在 RPi4 上的效能
  - `smartnav_navigation` + Nav2 + TurtleBot3 實機導航
  - RPi4 跨區網呼叫筆電 Ollama 的實際延遲與穩定性

## ⚠️ 已知限制

- **LLM 工具集全為導航導向**：目前 `smartnav_llm` 只有 create_map / navigate 等導航工具，**沒有銀行場景工具**（如「帶 VIP 到貴賓室」「通報行員」），`config/system_prompt.txt` 也是導航機器人設定。人臉事件進入 `/user_text` 後只會被當成一般對話處理，銀行場景的決策與通報動作是下一階段工作（預計落在 `smartnav_brain`）。
- **frontier_exploration_ros2 未內含**：自動探索建圖需另外 clone（見[安裝與建置](#-安裝與建置)），否則 `brain.launch.py` 的 frontier_explorer 無法啟動。
- **黑名單通報僅到「事件定位」層**：辨識結果會標示 BLACKLIST 並發布事件，但實際通報管道（LoRa / 訊息推播）尚未實作。
- **bringup 預設模型不適合本硬體**：`model_name` 預設 `gemma4:e2b`，在 4GB VRAM 筆電上實測極慢，請覆寫為 `qwen2.5:3b`。

## 🛠️ 故障排除

### 問題：LLM 沒有回應 / 逾時（Ollama 連線）

1. 確認筆電 Ollama 有監聽區網：設定 `OLLAMA_HOST=0.0.0.0` 後重啟 `ollama serve`
2. 從 RPi4 測試連線：`curl http://<筆電IP>:11434/api/tags`，應回傳模型清單
3. 檢查 Windows 防火牆是否放行 11434 連接埠（TCP 輸入規則）
4. 確認 launch 參數 `ollama_base_url` 是筆電**區網 IP**，不是 `localhost`
5. 若回應極慢（數十秒），檢查模型是否超出 VRAM：`ollama ps` 查看 CPU/GPU 分配比例，改用 `qwen2.5:3b`

### 問題：人臉辨識效能低下

1. RPi4 上請確認 `enable_gpu:=false`（RPi4 無 CUDA，強開 GPU provider 會失敗或退回 CPU）
2. 調整 `confidence_threshold` 與 `recognition_threshold`
3. 確保光照充足、影像清晰；檢查相機話題 `ros2 topic hz /image_raw`

### 問題：辨識錯誤率高（認錯人 / 認不出）

1. 增加註冊樣本數（`num_samples`，建議 10 以上，含不同角度）
2. 調整 `recognition_threshold`（調高減少誤認、調低減少漏認）
3. 註冊或刪除人員後記得呼叫 `/face_recognition/refresh_cache`
4. 換用更大的模型 `model_name:=buffalo_l`（RPi4 上 FPS 會下降，需權衡）

### 問題：找不到依賴套件 / 建置失敗

```bash
rosdep install --from-paths src --ignore-src -r -y
pip install insightface opencv-python onnxruntime sherpa-onnx langchain langchain-ollama
```

- 若 `brain.launch.py` 找不到 frontier_explorer：確認已 clone `frontier_exploration_ros2` 到 `src/` 並重新 `colcon build`

### 問題：語音節點啟動失敗

1. 確認已執行 `bash scripts/download_audio_models.sh` 下載 KWS / ASR / VAD / TTS 模型
2. 檢查麥克風/喇叭裝置：`arecord -l`、`aplay -l`，必要時以 `device` 參數指定

## 📚 文件索引

- [`docs/專題計畫.md`](docs/專題計畫.md)：專題企畫、里程碑與下一步
- [`docs/架構參考.md`](docs/架構參考.md)：各套件節點/話題/參數細節、指令大全、Key Files 對照、LLM 實測明細
- [`docs/LLM規劃書.md`](docs/LLM規劃書.md)：LLM 模型選擇與效能分析
- [`docs/部署與重現指南.md`](docs/部署與重現指南.md)：組員最快速的部署與 smoke test 流程
- [`docs/驗證與實作清單.md`](docs/驗證與實作清單.md)：待驗證項目與實作清單
- 上游專案：[swient/smartnav-bot](https://github.com/swient/smartnav-bot)
- [ROS 2 Jazzy 官方文檔](https://docs.ros.org/en/jazzy/) / [Nav2](https://navigation.ros.org/) / [InsightFace](https://github.com/deepinsight/insightface) / [sherpa-onnx](https://github.com/k2-fsa/sherpa-onnx) / [Ollama](https://ollama.com/)

## 授權

本專案採用 [MIT License](LICENSE)。開發規範見 [`.clinerules`](.clinerules)。
