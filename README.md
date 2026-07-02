# 智慧銀行 VIP 迎賓與安全通報系統

部署於 Raspberry Pi 4 與筆電協作的邊緣 AI 專案。人臉辨識在 RPi4 本機執行，LLM 決策由筆電上的 Ollama + FastAPI 提供，目標場景是銀行入口的 VIP 迎賓、訪客引導與黑名單警示。

## 專案現況

| 項目 | 狀態 | 說明 |
|------|------|------|
| Phase 1 環境建置 | 已完成並驗證 | RPi4 Ubuntu 24.04、InsightFace、OpenCV、ONNX Runtime 可安裝與啟動 |
| Phase 2a 即時人臉檢測 | 已完成並驗證 | InsightFace buffalo_sc 於 RPi4 實機約 12-15 FPS |
| Phase 2b VIP / 黑名單辨識 | 代碼完成，待端對端驗證 | 已有資料庫、註冊 CLI、即時辨識顯示，但仍需完整實機流程驗證 |
| Phase 3a 筆電 LLM API Server | 代碼完成，待 LAN 驗證 | FastAPI、狀態頁、聊天頁、JSON 回應格式已完成，仍需驗證 RPi4 實際呼叫 |
| Phase 3b Whisper STT | 未開始 | 僅保留規劃 |
| Phase 3c TTS 回覆 | 未開始 | 僅保留規劃 |
| Phase 4 ROS 2 / 導航 / LoRa / 雲端 | 代碼大幅完成，待整合與實機驗證 | 2026-07-01：已整併孫同學 ROS2 專案，smartnav_audio（語音）、smartnav_brain（地圖/導航）、smartnav_llm（LangChain Agent）都已有實作，但彼此尚未串接、尚未在實機測試 |

> 目前最重要的待驗證項目：
>
> 1. 照片註冊 VIP 後，能否在 RPi4 即時偵測流程穩定辨識。
> 2. RPi4 能否透過區網穩定呼叫筆電上的 /api/chat。
> 3. liveness_detector.py 在 ARM64 上是否可穩定初始化。
> 4. （新增）ROS2 端的 smartnav_audio / smartnav_brain / smartnav_llm 能否在真正的 ROS2 Jazzy 環境下 `colcon build` 成功並實機運作。

## 依專題實務目標對齊

你的最終目標不是單一 demo，而是一個以 ROS2 為主架構的智慧銀行機器人系統。依照目前 repo 的實際內容，這個專案現在處於「單程序流程可展示、ROS2 架構過渡中」的階段。

| 子系統 | 最終目標 | 目前進度 |
|------|----------|----------|
| 人臉辨識 | 單張大頭照註冊後能辨識 VIP / 訪客 / 黑名單並觸發動作 | 已確認以孫同學實際維護的 GitHub 版本（swient/smartnav-bot）為基礎，本專案的 gender/person_type 欄位是正確的銀行場景擴充（非分岔），待補準確率與端對端驗證 |
| 語音辨識 + LLM | 多語音輸入 -> LLM 分析 -> 固定 JSON 給 ROS2 | 已決策：終局為 ROS2 smartnav_llm（LangChain Agent），現階段 Ollama 運算仍在筆電；筆電 FastAPI 保留為非 ROS2 驗證流程的並行橋接 |
| ROS2 主架構 | vision / brain / audio / action / UI 都走 ROS2 | smartnav_msgs、smartnav_vision、smartnav_brain、smartnav_audio、smartnav_llm 都已有實作，但彼此尚未串接、無統一 bringup 總入口、尚未實機驗證 |
| 機器人動作與地圖 | 地圖建立、帶領、跟隨、語音提示 | smartnav_brain 已有 CreateMap（含自動探索）與 Navigate action，但只有到點導航，無帶領/跟隨語意層，且缺移動底盤可實測 |
| 前端 UI | 顯示辨識、LLM、系統狀態與功能頁 | 筆電端已有 Web 監控與聊天頁，機器人本體 UI 尚未完成 |

如果用一句話描述現在的 repo：非 ROS2 的「人臉辨識 + 筆電 LLM 決策 + 基礎 Web UI」主幹已可展示；ROS2 端已確定為終局主架構（smartnav_llm LangChain Agent + smartnav_vision 銀行客製化），但還沒有串接成一條可實機運行的完整 ROS2 主流程。 

## 現行架構

- RPi4 端
  - src/scripts/realtime_detection_insightface.py：即時偵測與辨識 GUI
  - src/scripts/manage_vip_database.py：VIP / 黑名單註冊、查詢、刪除
  - src/ai_vision/face_recognizer.py：512D embedding 比對
  - src/database/schema.py：SQLite schema
- 筆電端
  - src/llm_server/main.py：FastAPI Server
  - src/llm_server/llm_client.py：Ollama 呼叫與 JSON 解析
  - src/llm_server/static/index.html：狀態監控頁
  - src/llm_server/static/chat.html：互動測試頁
- ROS2 工作區（src/smartnav_ros2，2026-07-01 整併孫同學專案後）
  - smartnav_vision：人臉偵測/識別/註冊，已確認以孫同學 GitHub 維護版為基礎，再加上本專案銀行場景擴充（gender/person_type，非分岔）
  - smartnav_audio：語音喚醒、sherpa-onnx ASR、sherpa-onnx TTS（整併自孫同學）
  - smartnav_brain：地圖服務、地點服務、Nav2 導航動作（整併自孫同學）
  - smartnav_llm：**終局 LLM 決策架構**，LangChain + Ollama 對話式 Agent，可呼叫 ROS2 服務（整併自孫同學，新增 launch/llm.launch.py 可指定筆電 Ollama 位址）
  - smartnav_msgs：訊息/服務/動作定義，已合併雙方介面
  - smartnav_bringup：仍為空壳，尚無涵蓋全部套件的總 launch

詳細比較與差異談見 docs/專題計畫.md 的「功能/架構對照表」。

## 快速開始

### 1. 啟動筆電端 LLM API Server

```bash
ollama pull qwen2.5:3b
ollama create qwen-bank -f Modelfile_qwen
python -m pip install fastapi uvicorn pydantic
python src/llm_server/main.py --host 0.0.0.0 --port 8000
```

啟動後可直接開啟：

- http://localhost:8000/
- http://localhost:8000/chat
- http://localhost:8000/health

可選模型：

```bash
ollama create gemma3-bank -f Modelfile_gemma3
ollama create gemma4e2b-bank -f Modelfile_gemma4e2b
ollama create gemma4e4b-bank -f Modelfile_gemma4e4b
```

### 2. 部署 RPi4 環境

```bash
git clone https://github.com/xiange1209/welcoming_robot.git ai_bank_robot
cd ai_bank_robot
bash setup_ubuntu.sh
source ~/ai_bank_robot_env/bin/activate
```

相容入口 setup_rpi4_quick.sh 仍保留，但現在只會轉接到 setup_ubuntu.sh。

### 3. 初始化資料庫與新增測試 VIP

```bash
python3 src/scripts/manage_vip_database.py init
python3 src/scripts/manage_vip_database.py add-vip-image "測試VIP" photo.jpg --gender M --level gold
python3 src/scripts/manage_vip_database.py list-vips
```

目前人臉註冊支援兩種正規方式：

- 單張大頭照註冊：`add-vip-image` / `add-blacklist-image`
- 攝影機即時註冊：`add-vip-camera` / `add-blacklist-camera`

註冊流程已改為只接受真實照片或攝影機樣本，不再允許建立隨機測試 embedding，以免資料庫出現假人臉資料。

### 4. 啟動即時辨識

```bash
export DISPLAY=:1
python3 src/scripts/realtime_detection_insightface.py
```

### 5. 啟用 RPi4 到筆電的 LLM API 整合

即時辨識腳本現在已可在辨識到 VIP / 黑名單 / 訪客時，背景呼叫筆電端的 /api/chat，並把 reply / action 疊到畫面上。

預設為關閉，請先修改 config/inference_config.yaml：

```yaml
llm_api:
  enabled: true
  base_url: "http://<筆電IP>:8000"
```

或在啟動前直接用環境變數覆蓋：

```bash
export BANK_LLM_API_ENABLED=1
export BANK_LLM_API_URL="http://192.168.1.xxx:8000"
python3 src/scripts/realtime_detection_insightface.py
```

如果 API Server 暫時連不到，腳本會自動退回 fallback 回應，並暫時停止重試，不會每一幀都卡在網路請求上。

## 模型選擇摘要

下表屬於目前整理出的規劃數據與使用建議，其中只有 qwen2.5:3b 是目前 repo 預設主模型；其餘模型仍待實測補齊延遲與 JSON 穩定度。

| 模型 | 角色 | 估計 GPU 佔用 | 適合 RTX 3050 4GB | 目前狀態 |
|------|------|---------------|-------------------|----------|
| qwen2.5:3b | 預設主模型 | 約 2.2 GB | 是 | 已配置，建議優先使用 |
| gemma3:4b | 備用比較模型 | 約 3-4 GB | 勉強可用，可能部分 CPU 卸載 | 已補 Modelfile，待實測 |
| gemma4:e2b | 輕量替代模型 | 約 1.5-2.0 GB | 是 | 已有 Modelfile，待實測 |
| gemma4:e4b | 中型替代模型 | 約 2.5-3.5 GB | 接近上限 | 已有 Modelfile，待實測 |

完整比較與驗證建議請看 docs/LLM規劃書.md。

## 文件索引

- docs/部署與重現指南.md：給組員最快速的部署與 smoke test 流程
- docs/專題計畫.md：目前有效的企畫、里程碑、驗證矩陣與下一步
- docs/LLM規劃書.md：Qwen / Gemma 模型比較、估計資源需求與建議測試方式

## 目前限制

- 活體檢測流程仍未在 RPi4 ARM64 完整驗證。
- 筆電 API Server 與 RPi4 的區網整合仍待實機測試。
- ROS 2 套件目前只是骨架，不應視為已可部署功能。
- src/scripts/benchmark.py 已修正為現行模組路徑，但若要做 ONNX 模型基準，仍需先提供 config/inference_config.yaml 指向的模型檔。
- 目前硬體只有 RPi4、鏡頭與 RTX 3050 筆電，因此地圖建立、跟隨、帶領等導航功能暫時只能先規劃或改用模擬環境驗證。

## 建議的下一個驗證順序

1. 先在筆電端用 /chat 頁面驗證 qwen2.5:3b JSON 回應是否穩定。
2. 再在 RPi4 上完成 add-vip-image -> realtime_detection_insightface 的端對端驗證。
3. 最後驗證 RPi4 呼叫筆電 /api/chat，確認 LAN、延遲與 fallback 行為。
