# CLAUDE.md

本文件提供 Claude Code (claude.ai/code) 在此儲存庫工作時的指南。

## ⚡ 每個 session 先讀（30 秒）

**制度手冊在 `.claude/handbook/`**（七份）：harness 診斷 → `01-diagnosis.md`｜派工與模型選擇 → `02-dispatch.md`｜判斷 rubric（何時升級/何時算完成/何時停下問/何時換路）→ `03-judgment.md`｜模板（搜尋/實作/重構/研究/審查/HANDOFF 交接）→ `04-templates.md`｜改手冊的規則 → `05-maintenance.md`｜交接信與未竟事項 → `06-letter.md`｜黃金任務驗收 → `07-benchmark.md`。非 trivial 任務動手前先掃 02 與 03；若存在 `.claude/handbook/HANDOFF.md`，最先讀它。
**派工授權**：02-dispatch.md 第 1 節「指揮官不下場」是使用者的常設授權（2026-07-03 訂立）——列於該表的工作型態預設派 subagent，此指示覆寫 Agent 工具「僅使用者要求時派工」的預設限制。

**環境速查卡**（每條都是實測踩過的坑，違反必炸）：
- Python 用 `python`，絕不用 `python3`（Microsoft Store 空殼，靜默失敗 exit 49）
- PowerShell 5.1 沒有 `&&`/`||`；需要 POSIX 語法改用 Bash 工具
- 路徑含中文：shell 命令一律加雙引號；讀檔/搜尋優先用 Read/Grep/Glob 工具（避開編碼與跳脫亂碼）
- 破壞性操作（刪檔、reset、大規模搬移）前先 commit 快照：先 `git status` 確認範圍，用具名路徑 `git add`（避免 `-A` 誤收機密或大型檔案）
- 狀態的單一事實來源是本檔＋docs/；memory 只放使用者偏好與教訓，不寫進度
- 改完任何制度檔跑 `python scripts/check_handbook.py`

## Project Overview

**智慧銀行 VIP 迎賓與安全通報系統** — 以同學的 ROS 2 專案 [swient/smartnav-bot](https://github.com/swient/smartnav-bot) 為主體，整合人臉辨識（InsightFace）、語音（sherpa-onnx）、LLM 對話 Agent（LangChain + Ollama）與 Nav2 導航的雙機系統。

**關鍵約束**：
- **RPi4（8GB RAM, Ubuntu 24.04, ROS 2 Jazzy）**：跑 vision / audio / navigation / bringup + TurtleBot3
- **Windows 筆電（RTX 3050 4GB VRAM）**：跑 Ollama；RPi4 的 `smartnav_llm` 透過 `ollama_base_url` 連到筆電
- 2026-07-02 專案重組：**舊的單機 Python 原型已全部刪除**（`src/ai_vision/`、`src/database/`、`src/llm_server/`、`src/scripts/`、`config/inference_config.yaml`、`setup_rpi4_quick.sh` 等皆不存在，不可再引用）

## 目前狀態（2026-07-02 起，異動時更新本節）

**已驗證**：
- RPi4 上 InsightFace buffalo_sc 即時人臉檢測 15+ FPS（於舊架構驗證，引擎與 ROS 2 版相同）
- 筆電 LLM 延遲基準（`scripts/benchmark_llm_models.py`，摘要見下；明細與解讀在 `docs/架構參考.md`）

**已實作、待實機驗證**：
- 整個 ROS 2 workspace 尚未在 RPi4 實機 `colcon build` / 運行過
- vision → LLM 橋接（`/user_text`）、bringup 統一啟動、audio、navigation 皆屬此類

**未實作**：
- 銀行場景 LLM 工具（帶 VIP 到貴賓室、通報行員）— 目前工具集全為導航導向
- HMI（`smartnav_hmi` 為空殼）、多模態決策 orchestrator（`smartnav_brain` 為空殼）

## Architecture

**結構總覽**（完整樹狀圖、各套件節點/話題/參數細節 → `docs/架構參考.md`）：
`src/` 下 8 個 ROS 2 套件——`smartnav_msgs`（介面定義）、`smartnav_vision`（人臉辨識）、`smartnav_audio`（語音）、`smartnav_llm`（LLM Agent）、`smartnav_navigation`（地圖/導航）、`smartnav_brain`／`smartnav_hmi`（空殼預留）、`smartnav_bringup`（統一啟動）；根目錄另有 `scripts/`、`docs/`、四個 Modelfile 與 `benchmark_llm_results.json`。

### 雙機通訊

```
┌────────────────────────────┐              ┌────────────────────────┐
│ Raspberry Pi 4             │  HTTP        │ Windows 筆電            │
│ (Ubuntu 24.04, ROS2 Jazzy) │  Ollama API  │ (RTX 3050 4GB VRAM)    │
│ vision / audio / llm /     ├─────────────→│ Ollama :11434          │
│ navigation + TurtleBot3    │←─────────────┤ (qwen2.5:3b /          │
│                            │  串流回覆     │  gemma3:4b 可即時)      │
└────────────────────────────┘              └────────────────────────┘
```

### 事件流（vision → LLM）

```
相機 /image_raw
  → face_recognition_node（InsightFace 512D embedding 比對）
  → /face_recognition/result (smartnav_msgs/RecognitionResult)
  → recognition_text_bridge_node（轉自然語言）
  → /user_text (std_msgs/String)          ← speech_recognizer_node（ASR）也發到同一話題
  → llm_service_node（LangChain Agent，可呼叫導航工具）
  → /llm_response、/llm_stream、/speech_text
  → speech_synthesizer_node（TTS）→ /audio_out → voice_playback_node
```

## LLM 模型選擇（2026-07-02 實測結論）

qwen2.5:3b 最快（TTFT 2.2 秒/總 3.1 秒）、gemma3:4b 次之（2.3/4.7 秒）；gemma4:e2b/e4b 模型檔超出 4GB VRAM 被 CPU 卸載、慢 20~40 倍，**不適合即時互動**。完整表格與測試方法在 `docs/架構參考.md`，原始數據在 `benchmark_llm_results.json`。bringup 預設 `model_name` 仍是 `gemma4:e2b`，實際啟動時應覆寫。

## Common Commands

### 建置（RPi4）

```bash
cd ~/smartnav-bot          # workspace 根目錄（含 src/）
colcon build --symlink-install
source install/setup.bash
```

### 統一啟動（RPi4）

```bash
# 完整啟動，LLM 連到筆電 Ollama
ros2 launch smartnav_bringup smartnav.launch.py ollama_base_url:=http://192.168.1.xxx:11434

# 只跑視覺 + LLM（關閉 audio/navigation）
ros2 launch smartnav_bringup smartnav.launch.py \
  enable_audio:=false enable_navigation:=false \
  ollama_base_url:=http://192.168.1.xxx:11434 model_name:=qwen2.5:3b
```

### 人臉註冊（取代舊的 manage_vip_database.py）

```bash
ros2 service call /face_registration/register smartnav_msgs/srv/RegisterFace \
  "{person_name: '陳佳憲', gender: 'M', person_type: 'VIP', num_samples: 10}"
# 黑名單：person_type: 'BLACKLIST'；手動刷新快取：
ros2 service call /face_recognition/refresh_cache std_srvs/srv/Empty
```

### 除錯觀察

```bash
ros2 topic echo /face_recognition/result
ros2 topic echo /user_text
ros2 topic echo /llm_response
```

個別節點啟動、LLM 基準測試、語音模型下載、scp 傳輸 → 指令大全在 `docs/架構參考.md`。

## Development Workflow

Windows（Claude Code 修改程式）→ git push 或 scp 到 RPi4 → RPi4 `colcon build` 實機測試 → 回報輸出/FPS/錯誤 → 分析修正循環。詳細指令在 `docs/架構參考.md`。

## 已知限制與注意事項

1. **LLM 沒有銀行場景工具**：工具集全為導航導向；`system_prompt.txt` 是導航設定。人臉事件進 `/user_text` 後只會被當一般對話處理。這是後續開發重點。
2. **frontier_exploration_ros2 需手動 clone**：`brain.launch.py` 依賴它；`git clone https://github.com/mertgulerx/frontier_exploration_ros2` 到 `src/`。
3. **gemma4 在 4GB VRAM 不可行**：實測 CPU 卸載、首次輸出 42~95 秒。即時互動用 `qwen2.5:3b` 或 `gemma3:4b`。
4. **Windows 上用 `python` 不要用 `python3`**：後者是 Microsoft Store 空殼。
5. **ROS 2 workspace 未經實機驗證**：改動時不要假設既有行為已通過測試；文件需誠實區分「已驗證」與「已實作待驗證」。
6. **`enable_gpu` 在 RPi4 上應設 false**：RPi4 無 CUDA，InsightFace 需用 CPUExecutionProvider（launch 預設為 true，實機啟動時覆寫 `enable_gpu:=false`）。
7. **RPi4 效能參考**（舊架構實測）：buffalo_sc 檢測 15+ FPS、推理延遲約 100-150ms/幀、記憶體約 350-400MB。

## 參考檔案

各套件節點/話題/服務/參數、Key Files 對照表、LLM 實測明細、指令大全 → `docs/架構參考.md`。部署步驟 → `docs/部署與重現指南.md`。驗證狀態與待辦 → `docs/驗證與實作清單.md`。
