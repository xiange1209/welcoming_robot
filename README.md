# 智慧銀行 VIP 迎賓與安全通報系統

**Raspberry Pi 4 + TurtleBot3 邊緣 AI 迎賓機器人**

---

## 📖 專題簡介

部署於樹莓派 4 的邊緣 AI 系統，實現銀行門前的 VIP 迎賓與安全通報功能。全部推理在本機執行，無需上傳人臉照片，隱私優先。

### ✅ 目前已實現（Phase 2b，已在 RPi4 驗證）

| 功能 | 說明 |
|------|------|
| **實時人臉檢測** | InsightFace buffalo_sc，15+ FPS，512D embedding |
| **VIP 識別** | 餘弦相似度比對，一張照片即可註冊 |
| **黑名單告警** | 安全優先：先檢查黑名單，再比對 VIP |
| **性別記錄** | M/F/Other 手動輸入，顯示在識別標籤 |
| **防誤判過濾** | 連續 2 次偵測才確認 VIP/黑名單，避免短暫誤判 |
| **VIP/黑名單管理** | 新增、刪除、查詢（照片或攝影機即時拍攝） |
| **統一主選單** | `main.py` 一鍵啟動所有功能 |

### 🔄 規劃中（Phase 3+）

| 功能 | 說明 |
|------|------|
| **活體檢測** | 眨眼/搖頭/張嘴（MediaPipe，待 ARM64 驗證） |
| **語音銀行助手** | Whisper STT + 本機 LLM (Qwen/Gemma) + TTS |
| **人臉數據增強** | 單樣本生成多角度樣本，提升識別率 |
| **ROS 2 節點** | 多進程架構，face/voice/brain 分離 |

---

## 📂 專案結構

```
專題/
├── README.md
├── CLAUDE.md                          # Claude Code 開發指南
├── requirements_minimal.txt           # Python 依賴清單
├── setup_ubuntu.sh                    # Ubuntu 24.04 一鍵部署腳本
├── config/
│   └── inference_config.yaml          # 推理配置（平台/模型/閾值）
├── docs/
│   ├── 專題計畫.md                    # 完整計畫書（8 階段）
│   ├── 實作工作流程.md
│   └── vibe_coding流程.md             # Windows ↔ RPi4 開發循環
└── src/
    ├── ai_vision/
    │   ├── face_recognizer.py          # VIP/黑名單 embedding 比對
    │   ├── face_detector.py
    │   ├── liveness_detector.py        # 活體檢測（規劃中）
    │   └── inference_engine.py         # ONNX Runtime 抽象層
    ├── database/
    │   └── schema.py                   # SQLite 表結構
    ├── scripts/
    │   ├── main.py                     # ⭐ 統一主選單入口
    │   ├── realtime_detection_insightface.py  # 實時人臉檢測 GUI
    │   ├── manage_vip_database.py      # VIP/黑名單 CLI 管理工具
    │   ├── test_vip_recognition.py     # 單幀照片識別測試
    │   └── benchmark.py                # 性能基準測試
    └── smartnav_ros2/                  # ROS 2 多進程架構（Phase 3）
        ├── smartnav_vision/
        ├── smartnav_audio/
        └── smartnav_brain/
```

---

## 🚀 快速開始

### 1. 環境部署（首次在 RPi4 執行）

```bash
# 下載代碼
git clone https://github.com/xiange1209/welcoming_robot.git ai_bank_robot
cd ~/ai_bank_robot

# 一鍵安裝（Ubuntu 24.04 aarch64，約 40-60 分鐘）
bash setup_ubuntu.sh

# 啟動虛擬環境
source ~/ai_bank_robot_env/bin/activate
```

> 安裝內容：libcamera 0.3.0（修復 OV5647 bug）、Python venv、InsightFace、OpenCV、ONNXRuntime 等

### 2. 啟動主選單

```bash
source ~/ai_bank_robot_env/bin/activate
export DISPLAY=:1     # VNC 使用
cd ~/ai_bank_robot
python3 src/scripts/main.py
```

選單畫面：

```
╔══════════════════════════════════════════════╗
║      智慧銀行 VIP 迎賓與安全通報系統          ║
║      Phase 2b  |  RPi4 Edge Computing       ║
╚══════════════════════════════════════════════╝

── 人臉識別 ──────────────────────────────────
  [1] 啟動實時人臉檢測 (GUI)            ✅ 已實作
  [2] 單幀照片識別測試                   ✅ 已實作

── VIP 管理 ──────────────────────────────────
  [3] 新增 VIP（照片路徑）               ✅ 已實作
  [4] 新增 VIP（攝影機即時拍攝）         ✅ 已實作
  [5] 查看所有 VIP 名單                  ✅ 已實作

── 黑名單管理 ────────────────────────────────
  [6] 新增黑名單（照片路徑）             ✅ 已實作
  [7] 新增黑名單（攝影機即時拍攝）       ✅ 已實作
  [8] 查看黑名單                         ✅ 已實作

── 系統工具 ──────────────────────────────────
  [9]  性能基準測試                      ✅ 已實作
  [10] 初始化資料庫                      ✅ 已實作
  [11] 刪除 VIP                          ✅ 已實作
  [12] 刪除黑名單                        ✅ 已實作

── 規劃中 (Phase 3+) ─────────────────────────
  [13] 語音銀行助手                      🔄 規劃中
  [14] 人臉數據增強                      🔄 規劃中
  [15] ROS 2 節點管理                    🔄 規劃中

  [0] 退出
```

---

## 👤 VIP / 黑名單管理

### 初始化資料庫（首次執行）

```bash
python3 src/scripts/manage_vip_database.py init
```

### 新增 VIP

```bash
# 方法 1：交互式（提示輸入名字、性別、照片路徑）
python3 src/scripts/manage_vip_database.py add-vip

# 方法 2：CLI 直接輸入
python3 src/scripts/manage_vip_database.py add-vip-image "陳佳憲" photo.jpg \
  --gender M --level platinum --phone "0900123456"

# 方法 3：攝影機即時拍攝 5 張樣本（效果最佳）
export DISPLAY=:1
python3 src/scripts/manage_vip_database.py add-vip-camera
```

### 新增黑名單

```bash
python3 src/scripts/manage_vip_database.py add-blacklist
python3 src/scripts/manage_vip_database.py add-blacklist-image "李三" photo.jpg \
  --gender M --risk high --reason "Known fraud"
python3 src/scripts/manage_vip_database.py add-blacklist-camera
```

### 查詢 / 刪除

```bash
python3 src/scripts/manage_vip_database.py list-vips
python3 src/scripts/manage_vip_database.py list-blacklist
python3 src/scripts/manage_vip_database.py delete-vip       # 交互式，軟刪除
python3 src/scripts/manage_vip_database.py delete-blacklist
```

---

## 🎥 實時人臉檢測

```bash
export DISPLAY=:1
python3 src/scripts/realtime_detection_insightface.py

# 控制鍵：Q 退出 | S 保存當前幀 | 空白鍵 暫停
```

識別標籤格式：
- 🟡 **VIP**：`VIP 陳佳憲 (M) 0.82`（黃框）
- 🔴 **黑名單**：`黑名單 李三 (M) 0.75`（紅框）
- 🟢 **訪客**：`訪客 0.12`（綠框）

---

## ⚙️ 識別參數說明

| 參數 | 預設值 | 說明 |
|------|--------|------|
| `vip_threshold` | 0.25 | 餘弦相似度閾值，超過此值識別為 VIP |
| `blacklist_threshold` | 0.20 | 黑名單閾值（寬鬆以確保安全） |
| `detect_interval` | 5 | 每 5 幀做一次完整偵測（提升 FPS） |
| 確認次數 | 2 | 連續 2 次偵測相同結果才顯示（防誤判） |

> **同一人不同條件下**（戴眼鏡、素顏、不同光線）餘弦相似度約 0.2–0.5；
> 建議使用攝影機多樣本（5 張）註冊以提升識別率。

---

## 📊 性能表現（RPi4 8GB）

| 指標 | 數值 |
|------|------|
| 偵測 FPS | 15+ FPS（detect_interval=5） |
| 單幀推理延遲 | 100–150ms |
| 記憶體用量 | ~350–400MB |
| CPU 使用率 | ~60–70% |
| 模型大小 | ~50MB（buffalo_sc） |

---

## 🔧 開發工作流程（Windows ↔ RPi4）

```
[Windows] Claude Code 修改代碼
        ↓  SCP 傳輸
[RPi4]  測試與驗證
        ↓  回報結果
[Windows] 分析改進
```

```bash
# 傳輸代碼到 RPi4
cd C:\Users\陳佳憲\Desktop\專題
scp -r src/ setup_ubuntu.sh requirements_minimal.txt \
    xiange@192.168.1.125:~/ai_bank_robot/

# SSH 進入 RPi4 測試
ssh xiange@192.168.1.125
source ~/ai_bank_robot_env/bin/activate
cd ~/ai_bank_robot
python3 src/scripts/main.py
```

---

## 🐛 常見問題

### 1. 攝影機視窗出現黑畫面

InsightFace 初始化需要 200–300ms，導致視窗渲染不及。已在程式碼中加入 20 幀預熱解決，重新執行即可。

### 2. ORT GPU 警告訊息

```
[W:onnxruntime:Default, device_discovery.cc:325] GPU device discovery failed
```

正常現象（RPi4 無 GPU），已設定 `set_default_logger_severity(3)` 抑制，不影響功能。

### 3. VIP 識別不出來

- 確認使用的是 **餘弦相似度閾值**（0.25），不是歐氏距離
- 嘗試降低 `vip_threshold` 到 0.20（在 `realtime_detection_insightface.py` 修改）
- 建議用攝影機拍攝 5 張不同角度樣本（`add-vip-camera`）

### 4. OpenCV GUI 錯誤

```
The function is not implemented. Rebuild with GTK+ support
```

```bash
pip uninstall opencv-python-headless -y
pip install --no-cache-dir opencv-python
```

### 5. Picamera2 找不到

確認 venv 用 `--system-site-packages` 建立：

```bash
python3 -m venv --system-site-packages ~/ai_bank_robot_env
```

### 6. FPS 過低（< 10）

```bash
vcgencmd measure_temp    # 確認溫度 < 70°C
# 如過熱：加散熱片/風扇
# 或調高 detect_interval（預設 5，可調到 8）
```

---

## 🏗️ ROS 2 架構規劃（Phase 3+）

```
                 ┌─────────────────────────────┐
                 │      smartnav_brain          │
                 │   (決策與狀態機 orchestrator) │
                 └──────────┬──────────────────┘
                            │ ROS 2 Topics
           ┌────────────────┼────────────────┐
           ▼                ▼                ▼
  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐
  │smartnav_vision│  │smartnav_audio│  │  TurtleBot3  │
  │(人臉識別節點) │  │(語音助手節點) │  │  (導航節點)  │
  └──────────────┘  └──────────────┘  └──────────────┘
```

---

## 👥 分工

| 成員 | 負責模組 |
|------|---------|
| 陳佳憲 | 人臉識別（ai_vision）、資料庫、主選單、系統整合 |
| 孫瑋廷 | ROS 2 架構（smartnav_ros2）、導航 |

---

## 📋 依賴安裝

```bash
# RPi4 推薦使用 setup_ubuntu.sh（自動處理 libcamera + venv）
bash setup_ubuntu.sh

# 或手動安裝 Python 套件
pip install -r requirements_minimal.txt
# 注意：picamera2 需透過 setup_ubuntu.sh 安裝（非 pip 標準版）
```

---

## 📅 Phase 進度

| 階段 | 狀態 | 說明 |
|------|------|------|
| Phase 1 | ✅ 完成 | 環境建置（RPi4 + InsightFace） |
| Phase 2a | ✅ 完成 | 實時人臉檢測，12-13 FPS 驗證 |
| Phase 2b | ✅ 完成 | VIP 識別、黑名單、管理介面，FPS 15+ |
| Phase 3 | 🔄 規劃中 | 語音助手、ROS 2 多進程 |
| Phase 4+ | ⏳ 未開始 | LoRa 告警、雲端同步、HMI |
