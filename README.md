# 智慧銀行 VIP 迎賓與安全通報系統

**樹莓派 4 + TurtleBot3 人工智能邊緣運算平台**

---

## 📖 項目簡介

本項目是一套**邊緣 AI 系統**，部署在樹莓派 4 上，實現銀行門前的 VIP 迎賓和安全通報功能：

- 🎯 **核心功能**：
  - ✅ **實時人臉檢測** (InsightFace, 12-13 FPS)
  - ✅ **單樣本 VIP 識別** (一張大頭照即可註冊)
  - ✅ **性別記錄** (M/F/Other - 手動註冊，不自動檢測)
  - ✅ **黑名單告警** (安全優先檢測邏輯)
  - 🔄 **活體檢測** (眨眼/搖頭/張嘴 - 代碼就緒，待硬件測試)
  - 🔄 **機器人聯動** (遠程通知和動作執行)

- 🔧 **平台特性**：
  - 邊緣計算（無需上傳人臉照片，隱私優先）
  - 可配置（切換 RPi4 ↔ Jetson Orin 只需改配置文件）
  - ROS 2 就緒架構（未來可遷移到多進程模式）

---

## 📂 當前項目結構 (Phase 2b 已清理)

```
ai_bank_robot/
│
├── 📋 文檔與配置
│   ├── CLAUDE.md                    # Claude Code 開發指南
│   ├── README.md                    # 本文件
│   ├── 專題計畫.md                  # 完整項目規劃 (8 階段)
│   └── vibe_coding流程.md           # 開發工作流程指南 (中文)
│
├── 🔧 環境與配置
│   ├── requirements_minimal.txt     # 最小依賴 (20 包，~1.5GB)
│   ├── setup_rpi4_quick.sh          # 快速部署腳本 (~7 步驟)
│   └── config/
│       └── inference_config.yaml    # 推理配置 (平台/精度/模型路徑)
│
├── 👁️ 視覺 AI 模組 (vision_ai/)
│   ├── __init__.py
│   ├── inference_engine.py          # ONNX Runtime 抽象層 (RPi4 ↔ Jetson)
│   ├── face_detector.py             # 人臉檢測 (InsightFace buffalo_sc)
│   ├── face_recognizer.py           # ✅ VIP/黑名單識別 (embedding 比對)
│   └── liveness_detector.py         # 活體檢測 (MediaPipe landmarks)
│
├── 💾 數據庫模組
│   ├── database/
│   │   ├── __init__.py
│   │   └── schema.py                # ✅ SQLite 表結構 (含性別字段)
│   └── ai_bank_robot.db             # 運行時生成 (VIP/黑名單存儲)
│
├── 🤖 業務邏輯 (規劃中)
│   └── core_workflow/
│       └── __init__.py              # 未來: VIP → 機器人動作邏輯
│
├── 🚗 機器人導航 (團隊負責)
│   └── robot_navigation/
│       └── __init__.py              # 占位: SLAM/Nav2 集成
│
├── 🧰 工具腳本
│   ├── scripts/
│   │   ├── realtime_detection_insightface.py    # ⭐ 實時檢測 (GUI 顯示)
│   │   ├── manage_vip_database.py               # ⭐ VIP/黑名單管理工具
│   │   ├── test_vip_recognition.py              # 單幀識別測試
│   │   ├── benchmark.py                         # 性能基準測試
│   │   └── demo_phase2b.py                      # Phase 2b 演示
│
├── 🖼️ 模型存儲
│   └── models/
│       └── (InsightFace 自動下載至此)
│
├── 📷 測試照片
│   └── test_photos/
│       └── (用於 VIP 註冊的樣本)
│
└── .git/                            # Git 版本控制
    └── (GitHub: xiange1209/welcoming_robot)
```

**已刪除的過時模組** ❌：
- `backend_server/` - FastAPI（Phase 5 後端才需要）
- `communication/` - LoRa/MQTT（Phase 4 通訊才需要）
- `speech_hmi/` - 語音與 HMI（Phase 3 才實現）
- `tests/`、`docs/` - 文檔整合到 CLAUDE.md
- `setup_rpi4_dev.sh` - 舊腳本（已被 setup_rpi4_quick.sh 替代）

---

## 🎯 Phase 2b: 單樣本人臉識別 (當前階段)

### ✅ 已完成的功能

| 功能 | 狀態 | 說明 |
|------|------|------|
| **人臉檢測** | ✅ 驗證完成 | InsightFace buffalo_sc, 12-13 FPS, 512D embedding |
| **性別字段** | ✅ 完成 | M/F/Other，手動註冊，存儲在 SQLite |
| **VIP 識別** | ✅ 代碼完成 | Euclidean 距離 embedding 比對，閾值 0.65 |
| **黑名單檢測** | ✅ 代碼完成 | 安全優先邏輯，先檢黑名單，閾值 0.60 |
| **GUI 標籤** | ✅ 完成 | `VIP_<name>_<gender> (confidence)` 彩色框顯示 |
| **資料庫** | ✅ 完成 | SQLite + gender 字段 |
| **管理工具** | ✅ 完成 | 交互式 CLI 註冊、查詢、列表 |

### 🔄 待硬件測試的功能

| 功能 | 狀態 | 說明 |
|------|------|------|
| **實時識別整合** | 🟨 Code Ready | realtime_detection_insightface.py 已集成識別邏輯 |
| **活體檢測** | 🟨 Code Ready | 眨眼/搖頭/張嘴 檢測（需在 RPi4 驗證 MediaPipe ARM64） |
| **VIP 動作** | 🟨 待實現 | 識別 VIP 時觸發機器人說話/動作 |

### 🚀 識別流程

```
📹 實時攝像頭輸入
        ↓
🎯 InsightFace 檢測 (RetinaFace)
        ↓
🧮 提取 512D embedding 向量
        ↓
⚡ 與資料庫比對 (安全優先)
        ├─→ 黑名單? (距離 < 0.60)  →  🔴 警報: 黑名單_<name>_<gender>
        │
        ├─→ VIP? (距離 < 0.65)      →  🟨 歡迎: VIP_<name>_<gender>
        │                             (觸發機器人問候動作)
        │
        └─→ 陌生人                   →  🟢 訪客 (信心度)
```

---

## 🚀 快速開始

### 1. 環境部署 (首次在 RPi4 執行)

```bash
# 下載代碼到 RPi4
cd ~
git clone https://github.com/xiange1209/welcoming_robot.git ai_bank_robot

# 進入項目
cd ~/ai_bank_robot

# 執行快速部署 (自動化 7 步驟)
bash setup_rpi4_quick.sh

# 驗證環境
source ~/ai_bank_robot_env/bin/activate
python3 -c "from insightface.app import FaceAnalysis; print('✓ InsightFace ready')"
```

### 2. 初始化數據庫 (首次執行)

```bash
source ~/ai_bank_robot_env/bin/activate
cd ~/ai_bank_robot

# 初始化 SQLite schema
python3 scripts/manage_vip_database.py init
# 輸出: ✓ 數據庫已初始化
```

### 3. 註冊 VIP (使用照片)

#### 📌 方法 1: 交互式註冊

```bash
python3 scripts/manage_vip_database.py add-vip-interactive

# 交互提示:
# VIP 名字: 陳佳憲
# 性別 [M/F/Other]: M
# 電話 (選填): 0900123456
# 郵箱 (選填): user@bank.com
# 等級 [standard/gold/platinum]: platinum
# 人臉照片路徑: /path/to/photo.jpg

# 輸出: ✓ VIP '陳佳憲' (M) 已添加
```

#### 📌 方法 2: 命令行批量註冊

```bash
python3 scripts/manage_vip_database.py add-vip \
  --name "李美琪" \
  --gender F \
  --level gold \
  --phone "0900654321" \
  --email "lee@bank.com" \
  /path/to/lee_photo.jpg

# 輸出: ✓ VIP '李美琪' (F) 已添加
```

#### 📌 方法 3: 新增黑名單

```bash
python3 scripts/manage_vip_database.py add-blacklist \
  --name "王二" \
  --gender M \
  --risk high \
  --reason "Failed background check" \
  /path/to/blacklist_photo.jpg

# 輸出: ✓ 黑名單 '王二' (M) 已添加
```

### 4. 查看已註冊的 VIP 和黑名單

```bash
# 列出所有 VIP
python3 scripts/manage_vip_database.py list-vips

# 列出所有黑名單
python3 scripts/manage_vip_database.py list-blacklist
```

### 5. 測試單幀識別

```bash
# 用測試照片驗證識別算法
python3 scripts/test_vip_recognition.py /path/to/test/photo.jpg

# 輸出示例:
# ✓ 檢測到 1 張人臉
# VIP 匹配: 陳佳憲 (M) - 信心度 0.78
```

### 6. 🎬 運行實時檢測 (最重要！)

```bash
export DISPLAY=:1  # 設置顯示 (VNC/HDMI)
python3 scripts/realtime_detection_insightface.py

# GUI 窗口會顯示:
# - 實時攝像頭影像
# - 人臉邊界框 (黃=VIP, 紅=黑名單, 綠=訪客)
# - 标籤: VIP_陳佳憲_男 (0.78) 等

# 快捷鍵:
# q          - 退出
# s          - 保存當前幀到 /tmp/
# 空格       - 暫停/恢復
```

---

## 📊 核心模組說明

### 🎫 資料庫架構 (database/schema.py)

**表結構示例 (SQLite)**：

```python
# vip_members 表
id              INTEGER PRIMARY KEY
name            TEXT NOT NULL UNIQUE       # "陳佳憲"
gender          TEXT                       # "M", "F", "Other"
phone           TEXT
email           TEXT
vip_level       TEXT                       # "standard", "gold", "platinum"
face_embedding  BLOB (512D float32)        # embedding 向量
registered_at   TIMESTAMP
visit_count     INTEGER

# blacklist 表 (結構相同，多 reason + risk_level)
id              INTEGER PRIMARY KEY
name            TEXT NOT NULL UNIQUE
gender          TEXT                       # "M", "F", "Other"
face_embedding  BLOB (512D float32)
reason          TEXT                       # "Failed background check"
risk_level      TEXT                       # "low", "medium", "high"
added_at        TIMESTAMP
```

**Python 使用方法**：

```python
from database.schema import DatabaseSchema
from vision_ai.face_recognizer import FaceRecognizer

# 初始化
recognizer = FaceRecognizer()

# 添加 VIP
recognizer.add_vip(
    name="陳佳憲",
    face_embedding=embedding_array,  # 512D numpy array
    gender="M",
    phone="0900123456",
    email="user@bank.com",
    vip_level="platinum"
)

# 識別人臉
result = recognizer.recognize_face(face_embedding)
# 返回:
# {
#   'person_type': 'vip' | 'blacklist' | 'visitor',
#   'name': '陳佳憲',
#   'gender': 'M',
#   'confidence': 0.78,
#   'vip_level': 'platinum',
#   ...
# }

recognizer.close()
```

### 👁️ 視覺 AI 模組 (vision_ai/)

**InferenceEngine** (inference_engine.py):
- 抽象層，支持 ONNX Runtime + TensorFlow Lite
- 自動適配 RPi4 vs Jetson Orin（通過 config 切換）

```python
from vision_ai.inference_engine import InferenceEngine

engine = InferenceEngine("config/inference_config.yaml")
output = engine.predict("face_detector", frame)
```

**FaceDetector** (face_detector.py):
- InsightFace buffalo_sc 模型
- 返回人臉邊界框 + 512D embedding

```python
from vision_ai.face_detector import FaceDetector

detector = FaceDetector()
detections = detector.detect(frame)
# 返回: [{'box': [x1, y1, x2, y2], 'confidence': 0.95, 'embedding': array(...)}]
```

**FaceRecognizer** (face_recognizer.py):
- VIP/黑名單比對
- Euclidean 距離相似度
- 緩存優化

```python
from vision_ai.face_recognizer import FaceRecognizer

recognizer = FaceRecognizer()
result = recognizer.recognize_face(embedding)
# 返回識別結果 (VIP/黑名單/訪客)
```

---

## ⚙️ 配置系統

### config/inference_config.yaml

```yaml
# 平台選擇 (無需修改代碼)
platform: "rpi4"                # "rpi4" | "jetson_orin"
model_precision: "int8"         # "int8" | "fp16"
batch_size: 1                   # 單張推理
num_threads: 4                  # ARM 4 核

# 模型路徑
model_paths:
  face_detector: "./models/buffalo_sc"  # InsightFace 自動下載

# 推理參數
inference_params:
  face_detect_threshold: 0.7
  face_vip_threshold: 0.65        # VIP 匹配閾值
  face_blacklist_threshold: 0.60  # 黑名單閾值
  frame_rate: 10                  # FPS 目標
  detect_interval: 3              # 每 3 幀檢測一次 (優化性能)
```

**切換到 Jetson Orin** (只需改 3 行):

```yaml
platform: "jetson_orin"
model_precision: "fp16"
batch_size: 4                       # 可提高批量
use_gpu: true
```

---

## 🔄 開發工作流程 (Windows ↔ RPi4)

詳見 **`vibe_coding流程.md`** (中文完整指南)

### 快速循環

```
1. 編輯代碼 (Windows: C:/Users/陳佳憲/Desktop/專題/)
              ↓ (SCP 傳輸)
2. 同步代碼 (RPi4: ~/ai_bank_robot/)
              ↓ (SSH 遠程執行)
3. 測試驗證 (python3 scripts/realtime_detection_insightface.py)
              ↓ (複製輸出)
4. 反饋結果 (性能指標、錯誤信息)
              ↓ (回到第 1 步)
```

### 命令快速參考

```bash
# ━━━ Windows (Code Editor) ━━━
cd C:\Users\陳佳憲\Desktop\專題

# 同步所有代碼到 RPi4
scp -r . xiange@192.168.1.125:~/ai_bank_robot/

# ━━━ RPi4 (SSH) ━━━
ssh xiange@192.168.1.125

# 激活虛擬環境
source ~/ai_bank_robot_env/bin/activate
cd ~/ai_bank_robot

# 運行實時檢測
export DISPLAY=:1
python3 scripts/realtime_detection_insightface.py

# 查看實時資源使用
watch -n 1 'free -h; vcgencmd measure_temp; top -b -n 1 | head -10'
```

---

## 🚧 ROS 2 遷移計畫 (Phase 3+)

雖然當前使用 Python 單進程，架構已準備好遷移到 ROS 2 多進程模式：

### 當前架構 (Python 單進程)

```
realtime_detection_insightface.py
├─ FaceDetector (vision_ai/)
├─ FaceRecognizer (vision_ai/)
├─ LivenessDetector (vision_ai/)
└─ DatabaseSchema (database/)
```

### 目標架構 (ROS 2 多進程)

```
ai_bank_robot_ws/src/
├── vision_pkg/
│   ├── nodes/
│   │   ├── detection_node.py      # 人臉檢測發布 topic
│   │   └── recognition_node.py    # VIP 識別發布 topic
│   └── package.xml
│
├── speech_pkg/
│   ├── nodes/
│   │   ├── speech_recognizer_node.py  # STT
│   │   └── speech_synthesizer_node.py # TTS
│   └── package.xml
│
├── robot_control_pkg/
│   ├── nodes/
│   │   └── turtlebot_node.py      # 機器人控制
│   └── package.xml
│
└── core_orchestrator_pkg/
    ├── nodes/
    │   └── orchestrator_node.py    # 業務邏輯協調
    └── package.xml
```

### ROS 2 通訊拓撲 (未來)

```
vision_node 發布:
  /robot/face_detected
    ├─ person_id: "VIP_陳佳憲"
    ├─ gender: "M"
    └─ confidence: 0.78
           ↓
core_orchestrator_node 訂閱並決策
           ↓
speech_node 發布:
  /robot/say => "歡迎光臨，陳佳憲"
           ↓
robot_control_node 執行動作:
  /cmd_vel => 轉向客戶
```

---

## 📈 性能目標與現狀

| 指標 | 目標 | Phase 2a | Phase 2b(目標) |
|------|------|---------|--------------|
| **人臉檢測延遲** | < 200ms | ✅ 100-150ms | ✅ 100-150ms |
| **人臉識別延遲** | < 300ms | - | 🟨 待測 |
| **活體檢測延遲** | < 100ms | - | 🟨 待測 |
| **整體管線延遲** | < 500ms @ 10 FPS | ✅ 12-13 FPS | 🟨 待測 |
| **內存峰值** | < 2GB | ✅ 350-400MB | 🟨 待測 |
| **CPU 使用** | 60-80% | ✅ 60-70% | 🟨 待測 |
| **GPU** | N/A (RPi4) | N/A | N/A |

---

## 🔧 常見問題與解決方案

### 問題 1: OpenCV GUI 無法顯示

**症狀**: `The function is not implemented. Rebuild with GTK+ support`

**解決方案**:
```bash
pip uninstall opencv-python-headless -y
pip install --no-cache-dir opencv-python
```

### 問題 2: InsightFace 模型下載失敗

**症狀**: HTTP 404 errors

**解決方案**:
```bash
# 首次運行時會自動下載到 ~/.insightface/
python3 scripts/realtime_detection_insightface.py
# 模型自動下載到 insightface_models/
```

### 問題 3: Picamera2 找不到

**症狀**: `ModuleNotFoundError: No module named 'picamera2'`

**解決方案**:
```bash
# 虛擬環境需要 --system-site-packages
python3 -m venv --system-site-packages ~/ai_bank_robot_env
source ~/ai_bank_robot_env/bin/activate
sudo apt-get install -y libcap-dev python3-libcamera python3-kms++
```

### 問題 4: FPS 過低 (< 5 FPS)

**症狀**: 人臉檢測太慢

**解決方案**:
```bash
# 檢查溫度 (應該 < 70°C)
vcgencmd measure_temp

# 增加 detect_interval (每 4-5 幀檢測)
# 編輯 scripts/realtime_detection_insightface.py
# detect_interval = 5  # 而不是 3

# 檢查 CPU 使用
top -b -n 1 | head -15
```

### 問題 5: 磁盤空間不足

**症狀**: `[Errno 28] No space left on device`

**解決方案**:
```bash
# 清理 pip 緩存
pip clean --all
rm -rf ~/.cache/pip/*

# 清理系統日誌
sudo journalctl --vacuum-size=50M

# 驗證剩余空間
df -h  # 需要 > 5GB 可用

# 完整清理 (如果上述不夠)
sudo apt-get clean
sudo apt-get autoremove
```

### 問題 6: VIP 識別不準確 (信心度太低)

**症狀**: 識別最高信心度只有 0.55，閾值是 0.65，無法識別

**原因可能**:
1. 照片質量差 (模糊、光線不足、角度不正)
2. 閾值過高
3. 運行時攝像頭角度與註冊照片差異大

**解決方案**:

```bash
# 方案 1: 調整閾值 (短期)
# 編輯 vision_ai/face_recognizer.py
self.vip_threshold = 0.60  # 從 0.65 降到 0.60

# 方案 2: 使用高質量照片重新註冊 (推薦)
# 要求: 正面、清晰、充足光線、640x480+ 解析度
python3 scripts/manage_vip_database.py add-vip \
  --name "陳佳憲" --gender M \
  /path/to/better_photo.jpg

# 方案 3: 檢查 embedding 質量
python3 << 'EOF'
from scripts.manage_vip_database import extract_embedding_from_image
import numpy as np

emb = extract_embedding_from_image("photo.jpg")
print(f"Embedding shape: {emb.shape}")
print(f"Norm: {np.linalg.norm(emb):.4f}")
print(f"Range: [{emb.min():.4f}, {emb.max():.4f}]")
EOF
```

---

## 🧪 測試与驗證

### 單元測試 (單個模組)

```bash
# 測試人臉檢測
python3 << 'EOF'
from vision_ai.face_detector import FaceDetector
import cv2

frame = cv2.imread("test_photo.jpg")
detector = FaceDetector()
detections = detector.detect(frame)
print(f"✓ 檢測到 {len(detections)} 張人臉")
EOF

# 測試人臉識別
python3 scripts/test_vip_recognition.py test_photo.jpg

# 測試數據庫
python3 << 'EOF'
from database.schema import DatabaseSchema
schema = DatabaseSchema()
schema.initialize()
print("✓ 數據庫就緒")
EOF
```

### 集成測試 (完整流程)

```bash
# 步驟 1: 初始化
python3 scripts/manage_vip_database.py init

# 步驟 2: 註冊 VIP
python3 scripts/manage_vip_database.py add-vip \
  --name "李美琪" --gender F \
  test_photos/lee.jpg

# 步驟 3: 測試單幀
python3 scripts/test_vip_recognition.py test_photos/lee.jpg
# 預期: ✓ VIP 匹配: 李美琪 (F) - 信心度 > 0.65

# 步驟 4: 實時檢測
export DISPLAY=:1
python3 scripts/realtime_detection_insightface.py
# 預期: 看到黃色框 + 標籤 "VIP_李美琪_女 (0.78)"
```

### 性能基準測試

```bash
# 完整性能報告 (推理延遲、內存、CPU、溫度)
python3 scripts/benchmark.py

# 輸出示例:
# ━━━ InsightFace Benchmark ━━━
# Platform: rpi4
# Model: buffalo_sc
#
# Latency (10 frames average):
#   Min: 95ms  | Avg: 108ms | Max: 145ms
#
# Memory:
#   Peak: 412 MB
#
# CPU:
#   Load: 68%
#
# Temperature:
#   CPU: 52°C
```

---

## 📚 核心命令速查表

```bash
# ━━━ 環境管理 ━━━
source ~/ai_bank_robot_env/bin/activate      # 激活虛擬環境
pip list                                      # 查看已安裝包
df -h                                         # 磁盤空間
free -h                                       # 內存使用
vcgencmd measure_temp                         # RPi4 溫度

# ━━━ 數據庫管理 ━━━
python3 scripts/manage_vip_database.py init                    # 初始化
python3 scripts/manage_vip_database.py add-vip-interactive    # 交互添加 VIP
python3 scripts/manage_vip_database.py list-vips              # 列表
python3 scripts/manage_vip_database.py list-blacklist         # 黑名單

# ━━━ 測試與驗證 ━━━
python3 scripts/test_vip_recognition.py photo.jpg             # 單幀識別
python3 scripts/benchmark.py                                   # 性能測試
python3 -c "from insightface.app import FaceAnalysis; print('✓')"  # 檢查模型

# ━━━ 實時運行 ━━━
export DISPLAY=:1
python3 scripts/realtime_detection_insightface.py             # 實時檢測

# ━━━ 調試工具 ━━━
watch -n 1 'free -h; top -b -n 1 | head -10'     # 實時監控資源
ssh xiange@192.168.1.125                         # SSH 連接
scp -r . xiange@192.168.1.125:~/ai_bank_robot/   # SCP 傳輸代碼
```

---

## 📖 相關文檔

- **CLAUDE.md** - 完整開發指南 (架構、Debugging、已知問題)
- **專題計畫.md** - 完整項目規劃 (8 階段、硬件規格、風險評估)
- **vibe_coding流程.md** - 開發工作流程 (Windows ↔ RPi4 循環)

---

## 👥 團隊分工

| 角色 | 負責人 | 任務 |
|-----|-------|------|
| **Vision AI + Speech** | 陳佳憲 (你) | 本項目所有模組 |
| **Robot Control / SLAM** | 同學 | ROS 2, SLAM (SLAM 建圖), Nav2 (自主導航) |
| **Backend + Communication** | 團隊協力 | FastAPI 後端、LoRa/MQTT 通訊 (Phase 3+) |

---

## 🔗 有用的資源

### 官方文檔
- [InsightFace GitHub](https://github.com/deepinsight/insightface)
- [OpenCV Python](https://docs.opencv.org/4.x/)
- [ONNX Runtime](https://onnxruntime.ai/)
- [Raspberry Pi OS](https://www.raspberrypi.com/software/)
- [ROS 2 文檔](https://docs.ros.org/)

### 開發工具
- **SSH 工具**: PuTTY / Windows Terminal
- **文件傳輸**: WinSCP / scp 命令
- **遠程編輯**: VS Code Remote SSH

---

## 📊 當前進度 (2026-04-07)

- ✅ **Phase 1**: 環境搭建完成
- ✅ **Phase 2a**: 人臉檢測驗證完成 (12-13 FPS)
- ✅ **Phase 2b**: 代碼完成 + 性別字段支持 ← **當前，待硬件測試**
- 🔄 **Phase 3**: ROS 2 遷移計畫
- 🔄 **Phase 4+**: 語音、HMI、LoRa 通訊

---

**版本**: 2.0 (2026-04-07)
**狀態**: Phase 2b Code Ready - 等待硬件測試驗證
**最後更新**: 2026-04-07

---

**有問題？** 查看 CLAUDE.md 中的「Debugging Tips」或 GitHub Issues 提出。
