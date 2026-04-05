# 智慧銀行 VIP 迎賓與安全通報系統

樹莓派 4 + TurtleBot3 人工智能邊緣運算平台

## 📂 專案結構

```
.
├── vision_ai/                    # 視覺 AI 模組（人臉檢測/識別）
│   ├── __init__.py
│   ├── inference_engine.py       # ONNX 推理引擎抽象層
│   ├── face_detector.py          # 人臉檢測 (RetinaFace INT8)
│   ├── liveness_checker.py       # 活體防偽 (待實現)
│   └── face_recognizer.py        # 人臉識別 (ArcFace INT8，待實現)
│
├── speech_hmi/                   # 語音與人機界面模組
│   ├── __init__.py
│   ├── speech_recognizer.py      # 語音識別 (Whisper，待實現)
│   ├── intent_classifier.py      # 意圖分類 (待實現)
│   └── speech_synthesizer.py     # 語音合成 (pyttsx3，待實現)
│
├── database/                     # 資料庫管理
│   ├── __init__.py
│   ├── schema.py                 # SQLite 表結構
│   └── sync_manager.py           # 雲端同步 (待實現)
│
├── config/                       # 配置檔
│   └── inference_config.yaml     # 推理引擎配置（平台+精度可切換）
│
├── scripts/                      # 工具腳本
│   ├── benchmark.py              # 性能基準測試
│   └── download_models.py        # 模型下載 (待實現)
│
├── models/                       # 預訓練模型存儲
│   ├── retinaface_int8.onnx      # (需下載)
│   ├── arcface_int8.onnx         # (需下載)
│   └── efficientnet_b0_emotion.onnx  # (需下載)
│
├── robot_navigation/             # 機器人導航模組 (團隊負責)
├── communication/                # 通訊模組 (LoRa/MQTT)
├── core_workflow/                # 核心工作流程邏輯
├── backend_server/               # FastAPI 後端服務
├── tests/                        # 測試套件
├── docs/                         # 文檔
│
├── setup_rpi4_dev.sh             # 樹莓派環境搭建腳本
├── CLAUDE.md                     # Claude Code 開發指南
├── 專題計畫.md                   # 完整項目規劃文檔
├── .gitignore
└── README.md                     # 本文件
```

## 🚀 快速開始

### 1. 環境設置

在樹莓派上運行一次（已完成）：
```bash
bash setup_rpi4_dev.sh
```

### 2. 初始化資料庫

```bash
source ~/ai_bank_robot_env/bin/activate
cd ~/ai_bank_robot
python3 -c "from database.schema import DatabaseSchema; DatabaseSchema().initialize()"
```

### 3. 下載量化模型

需要下載以下模型到 `models/` 目錄：
- **RetinaFace INT8**: < 10MB
- **ArcFace INT8**: < 20MB
- **EfficientNet-B0**: < 15MB

### 4. 運行基準測試

```bash
python3 scripts/benchmark.py
```

輸出預期結果：
- 人臉檢測延遲：~200ms @ 10 FPS
- 整體推理延遲：< 500ms
- 內存使用：< 2GB

## 📊 核心模組說明

### InferenceEngine (vision_ai/inference_engine.py)
推理引擎抽象層，自動適配平台（RPi4 vs Jetson Orin）

**使用方法**：
```python
from vision_ai import InferenceEngine

engine = InferenceEngine("config/inference_config.yaml")
engine.load_all_models()

# 推理
output = engine.predict("face_detector", input_data)

# 基準測試
results = engine.benchmark("face_detector", input_shape, num_runs=10)
```

### FaceDetector (vision_ai/face_detector.py)
RetinaFace 人臉檢測

**使用方法**：
```python
from vision_ai import FaceDetector
import cv2

detector = FaceDetector()
frame = cv2.imread("image.jpg")

# 檢測人臉
detections = detector.detect(frame)
# → [{'box': [x1, y1, x2, y2], 'confidence': 0.95, 'landmarks': [...]}]

# 繪製結果
result = detector.draw_detections(frame, detections)
cv2.imshow("Result", result)
```

### DatabaseSchema (database/schema.py)
SQLite 資料庫管理

**使用方法**：
```python
from database import DatabaseSchema

schema = DatabaseSchema()
schema.initialize()

conn = schema.connect()

# 記錄訪問
schema.log_visit(conn, person_type='vip', person_name='Alice', confidence=0.98)

# 記錄告警
schema.log_security_alert(conn, alert_type='blacklist_detected', person_name='Bob', severity='high')

conn.close()
```

## 📈 性能目標

| 指標 | 目標 | 現狀 |
|-----|------|------|
| 人臉檢測延遲 | < 200ms | 測試中 |
| 人臉識別延遲 | < 300ms | 待實現 |
| 整體管線延遲 | < 500ms @ 10 FPS | 待測試 |
| 內存峰值 | < 2GB | 待測試 |
| CPU 使用 | 60-80% | 待監控 |

## 🔧 配置系統說明

`config/inference_config.yaml` 控制所有推理參數：

**平台切換**（無需修改代碼）：
```yaml
# RPi4
platform: "rpi4"
model_precision: "int8"
use_gpu: false

# 升級到 Jetson 時，只需改這三行
platform: "jetson_orin"
model_precision: "fp16"
use_gpu: true
```

## 📝 下一步任務

### Phase 2 - 視覺 AI 核心 (進行中)

- [x] 建立專案結構
- [x] 配置系統 (inference_config.yaml)
- [x] 資料庫 schema
- [x] 推理引擎框架
- [x] 人臉檢測模組
- [x] 基準測試工具
- [ ] 下載量化模型
- [ ] 實現活體檢測 (liveness_checker.py)
- [ ] 實現人臉識別 (face_recognizer.py)
- [ ] 在樹莓派上進行性能測試

### Phase 3 - 語音與 HMI

- [ ] Whisper 語音識別
- [ ] TTS 語音合成 (pyttsx3)
- [ ] 意圖分類器
- [ ] 7吋觸控屏 UI

## 🔗 有用的命令

```bash
# 啟動虛擬環境
source ~/ai_bank_robot_env/bin/activate

# 查看已安裝套件
pip list | grep -E 'opencv|onnx|whisper|fastapi'

# 測試推理引擎
python3 -c "from vision_ai import InferenceEngine; print(InferenceEngine().platform)"

# 初始化資料庫
python3 -c "from database import DatabaseSchema; DatabaseSchema().initialize()"

# 運行基準測試
python3 scripts/benchmark.py

# 查看資源使用
top -b -n 1 | head -15
free -h
df -h

# 樹莓派溫度
vcgencmd measure_temp
```

## 📖 相關文檔

- `專題計畫.md` - 完整項目規劃 (8 階段、團隊分工、風險評估)
- `CLAUDE.md` - Claude Code 開發指南
- `scripts/benchmark.py` - 性能測試工具說明

## ⚠️ 已知限制

1. **Whisper 未安裝** - 磁盤空間限制，可稍後安裝：`pip install openai-whisper`
2. **模型未下載** - 需手動下載 RetinaFace、ArcFace、EfficientNet INT8 模型
3. **活體檢測待實現** - 需集成 MediaPipe Face Landmarks
4. **團隊協調** - 等待同學完成 ROS 2 / SLAM 集成

## 👥 團隊分工

| 角色 | 負責人 | 任務 |
|-----|-------|------|
| 視覺 AI + 語音 | 陳佳憲（你） | 本項目相關模組 |
| 機器人控制/導航 | 同學 | ROS 2, SLAM, Nav2 |
| 後台系統 + 通訊 | 團隊協力 | FastAPI, LoRa, MQTT |

## 📞 支持

遇到問題？檢查以下步驟：

1. 查看 `CLAUDE.md` 中的「Debugging Tips」
2. 檢查 `setup_rpi4_dev.sh` 步驟 [1/7] - [7/7] 是否全部通過
3. 驗證 Python 版本 >= 3.9：`python3 --version`
4. 驗證核心套件安裝：`pip list | grep opencv`

---

**版本**: 1.0
**最後更新**: 2026-04-05
**狀態**: ✅ Phase 1 完成，Phase 2 進行中
