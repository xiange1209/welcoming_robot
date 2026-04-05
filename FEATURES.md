# 智慧銀行AI服務機器人 - 功能模塊清單

## Phase 1: ✅ 已完成

### 🎥 實時人臉檢測模塊
**文件**: `scripts/realtime_detection_insightface.py`

**功能**:
- ✅ 高精度人臉檢測 (RetinaFace ONNX)
- ✅ 正臉 + 側臉同時檢測
- ✅ 置信度評分 (0.0-1.0)
- ✅ GUI 顯示 (OpenCV with GTK)
- ✅ 實時 FPS 監控
- ✅ 幀保存功能 (S 鍵)
- ✅ 暫停/繼續 (Space 鍵)
- ✅ 退出 (Q 鍵)

**性能指標**:
- **FPS**: 12-13 (目標 10 ✅)
- **推理延遲**: ~100-150ms per frame
- **內存使用**: ~350-400MB
- **CPU 使用**: ~60-70%
- **模型大小**: ~50MB (buffalo_sc)

**已嵌入功能**:
- InsightFace buffalo_sc 模型自動下載
- 幀跳過優化 (每 3 幀檢測 1 次)
- 人臉 embedding 提取 (512D 向量：用於後續識別)

---

### 🗂️ 環境配置
**文件**: `config/inference_config.yaml`

**功能**:
- ✅ 平台選擇 (RPi4 / Jetson Orin)
- ✅ 模型精度設定 (INT8 / FP16)
- ✅ 推理參數配置
- ✅ 模型路徑管理

---

### 💾 數據庫架構
**文件**: `database/schema.py`

**已定義表結構**:
- `vip_members` - VIP 名單 + face_embedding
- `blacklist` - 黑名單 + face_embedding
- `visit_logs` - 訪客日誌
- `security_alerts` - 安全告警
- `robot_status_log` - 機器人狀態監控

---

## Phase 2: 🔄 進行中和後續

### 🔒 活體防偽 (Liveness Detection) - 待實施
**計劃功能**:
- 眨眼檢測 (MediaPipe Face Landmarks)
- 搖頭檢測 (頭部位置追蹤)
- 張嘴檢測 (嘴部張合動作)
- 防照片/視頻攻擊

**預計難度**: ⭐⭐ (中等)
**預計時間**: 2-3 天

---

### 👤 VIP 人臉識別 (Face Recognition) - 待實施
**計劃功能**:
- 使用 InsightFace embedding (已收集)
- VIP 數據庫匹配
- 黑名單檢測
- 信心度閾值設定 (0.6)
- 陌生人分類

**預計難度**: ⭐⭐⭐ (中等)
**預計時間**: 3-4 天

---

### 😊 表情分析 (Emotion Analysis) - 後續
**計劃功能**:
- 7 種表情分類 (喜、怒、哀、樂、驚、厭、中性)
- 自動調整迎賓語氣
- 情緒趨勢分析

**預計難度**: ⭐⭐ (中等)
**預計時間**: 2-3 天
**模型**: EfficientNet-B0 或 MobileNetV2 (輕量化)

---

### 📝 訪客日誌記錄 - 待實施
**計劃功能**:
- SQLite 本地存儲
- 訪問時間記錄
- 檢測置信度
- MQTT 定期同步

---

### 🔊 語音互動 - 後續
**計劃功能**:
- Whisper 語音識別 (STT)
- pyttsx3 語音合成 (TTS)
- 意圖分類
- 自動迎賓

**已安裝**: FastAPI 框架

---

### 📱 HMI 觸控屏 - 後續
**計劃功能**:
- 7 吋觸控屏集成
- 實時人臉預覽
- VIP 匹配結果顯示
- 互動菜單

---

### 🤖 自主導航 - 隊友負責
**技術棧**:
- ROS 2 Jazzy
- Nav2 路徑規劃
- SLAM (Cartographer / SLAM Toolbox)
- RPLIDAR A1M8 + RealSense D435i

---

### 📡 IoT 通訊 - 後續
**計劃功能**:
- LoRa 雙向通訊
- VIP 通報推送
- 安全告警
- MQTT 雲端同步

---

## 📊 已驗證的模型 & 庫

| 組件 | 版本 | 狀態 | 備註 |
|------|------|------|------|
| InsightFace | 0.7.3 | ✅ | buffalo_sc (輕量，推薦) |
| OpenCV | 4.13.0 | ✅ | with GTK support |
| ONNX Runtime | 1.24.4 | ✅ | CPU optimized |
| Picamera2 | 0.3.34 | ✅ | 樹莫派原生 |
| NumPy | 2.2.4 | ✅ | 基礎計算 |
| FastAPI | 0.135.3 | ✅ | Web 框架 |
| MQTT | 2.1.0 | ✅ | 通訊協議 |
| PyYAML | 6.0.2 | ✅ | 配置管理 |

---

## 🎯 開發路線圖

```
Phase 1: 環境 + 人臉檢測 ✅ DONE
  └─ 推理引擎 + Picamera2 + InsightFace

Phase 2: 活體防偽 + VIP 識別 🔄 NEXT (1-2週)
  └─ Liveness (眨眼/搖頭) + Face Recognition

Phase 3: 表情分析 + 語音交互 (1-2週)
  └─ Emotion + Whisper STT + pyttsx3 TTS

Phase 4: HMI UI + 數據庫 (2週)
  └─ 觸控屏 + SQLite + MQTT

Phase 5: IoT 通訊 + 導航集成 (2週)
  └─ LoRa + 行員 App + ROS2 Nav2

Phase 6: 雲端後台 + 測試 (2-3週)
  └─ Web Dashboard + 異常容錯 + 性能優化

總計: 8-10 週 (4 個月)
```

---

## 🚀 快速遷移清單

移植到新樹莫派時的步驟：

```bash
# 1. 克隆或傳輸項目
git clone <repo> ai_bank_robot

# 2. 進入目錄
cd ai_bank_robot

# 3. 創建虛擬環境（支持系統套件）
python3 -m venv --system-site-packages ~/ai_bank_robot_env

# 4. 激活虛擬環境
source ~/ai_bank_robot_env/bin/activate

# 5. 安裝最小化依賴
pip install --no-cache-dir -r requirements_minimal.txt

# 6. 驗證安裝
python3 -c "from insightface.app import FaceAnalysis; print('✅ Ready')"

# 7. 設置顯示
export DISPLAY=:1

# 8. 運行檢測
python3 scripts/realtime_detection_insightface.py
```

---

## 📈 性能監控命令

```bash
# 實時 FPS 和內存
python3 scripts/realtime_detection_insightface.py

# 保存檢測結果到文件
python3 scripts/realtime_detection_insightface.py 2>&1 | tee detection_log.txt

# 批量查看保存的幀
ls -lah /tmp/face_detection_*.jpg
```

---

## 💡 已知限制 & 優化空間

| 項 | 當前 | 優化 |
|---|------|------|
| FPS | 12-13 | 用 buffalo_L 可達 8-10，但需要更多資源 |
| 內存 | ~350MB | 可減少到 ~250MB（模型量化） |
| 延遲 | 100-150ms | 可優化到 80ms（幀跳過調整） |
| 側臉檢測 | ✅ 支援 | 已達到 |
| 群體檢測 | ✅ 支援 | 最多同時檢測 5+ 人 |

