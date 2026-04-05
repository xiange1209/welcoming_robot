# 快速移植指南 - Raspberry Pi 4 環境部署

## 📋 簡介

本指南用於在**新的 Raspberry Pi 4** 上快速重建智慧銀行AI服務機器人的開發環境。

**預計時間**: 20-30 分鐘（取決於網絡速度）

---

## ✅ 需要做的準備

- ✅ Raspberry Pi 4 (8GB)
- ✅ 32GB SD 卡（已刷 Raspberry Pi OS）
- ✅ 攝像頭（可選，測試時需要）
- ✅ 網絡連接（WiFi 或有線）
- ✅ SSH 或直接登錄

---

## 🚀 快速部署（一行命令）

### 方案 A：本地執行（推薦）

在樹莫派上執行：

```bash
# 1. 下載項目（如果還沒有）
git clone <repo_url> ~/ai_bank_robot
cd ~/ai_bank_robot

# 2. 運行快速部署腳本
bash setup_rpi4_quick.sh
```

**就這麼簡單！** ✨

---

### 方案 B：分步執行（手動控制）

如果想看每一步的詳細輸出，逐步執行：

```bash
# 1. 更新系統
sudo apt-get update && sudo apt-get upgrade -y

# 2. 安裝 GTK+ 和依賴
sudo apt-get install -y \
    libgtk2.0-dev pkg-config \
    libcap-dev python3-dev

# 3. 創建虛擬環境
python3 -m venv --system-site-packages ~/ai_bank_robot_env
source ~/ai_bank_robot_env/bin/activate

# 4. 清理磁盤
rm -rf ~/.cache/pip/*
sudo apt-get clean
sudo journalctl --vacuum-size=50M

# 5. 檢查磁盤空間（需要 >5GB 可用）
df -h

# 6. 安裝 Python 依賴
pip install --no-cache-dir -r requirements_minimal.txt
```

---

## 📊 安裝內容清單

### 核心套件（~1.5 GB）

| 套件 | 版本 | 用途 |
|------|------|------|
| insightface | 0.7.3 | 人臉檢測 + 識別 |
| opencv-python | 4.13.0 | 圖像處理 + GUI |
| onnxruntime | 1.24.4 | ONNX 模型推理 |
| numpy | 2.2.4 | 數值計算 |
| picamera2 | 0.3.34 | 攝像頭驅動 |
| fastapi | 0.135.3 | Web 框架 |
| paho-mqtt | 2.1.0 | 通訊協議 |

### 模型文件（~50-100 MB）

- `insightface_models/buffalo_sc/` - InsightFace 檢測和識別模型
  - 首次運行時自動下載

---

## 🔧 驗證安裝

安裝完成後，驗證所有套件：

```bash
# 激活虛擬環境
source ~/ai_bank_robot_env/bin/activate

# 進入項目目錄
cd ~/ai_bank_robot

# 測試導入
python3 << 'EOF'
print("測試套件導入...")
import cv2
print(f"✓ OpenCV {cv2.__version__}")

import insightface
print("✓ InsightFace 0.7.3")

import onnxruntime
print("✓ ONNX Runtime")

from picamera2 import Picamera2
print("✓ Picamera2")

import numpy
print(f"✓ NumPy {numpy.__version__}")

import fastapi
print("✓ FastAPI")

import paho.mqtt
print("✓ MQTT")

print("")
print("🎉 所有套件已驗證！")
EOF
```

**預期輸出**:
```
✓ OpenCV 4.13.0
✓ InsightFace 0.7.3
✓ ONNX Runtime
✓ Picamera2
✓ NumPy 2.2.4
✓ FastAPI
✓ MQTT

🎉 所有套件已驗證！
```

---

## 🎥 首次運行

```bash
# 設置顯示（如果使用 VNC）
export DISPLAY=:1

# 運行人臉檢測
python3 scripts/realtime_detection_insightface.py
```

**首次運行會自動下載模型** (~50MB，需要 1-2 分鐘）。

**預期結果**:
- ✅ 視頻窗口顯示
- ✅ 檢測到人臉時用綠色框標記
- ✅ FPS 顯示在左上角（目標 10+）
- ✅ 按 S 保存幀到 `/tmp/`
- ✅ 按 Q 退出

---

## 🆘 常見問題和解決方案

### 問題 1：磁盤空間不足

**症狀**: `[Errno 28] No space left on device`

**解決方案**:
```bash
# 清理磁盤
sudo apt-get clean
sudo apt-get autoclean
rm -rf ~/.cache/pip/*
sudo journalctl --vacuum-size=50M

# 檢查可用空間
df -h
# 需要至少 5GB 可用
```

---

### 問題 2：模型下載失敗

**症狀**: InsightFace 自動下載時網絡超時

**解決方案**:
```bash
# 手動指定模型目錄
mkdir -p ./insightface_models

# 運行時會自動重試下載
python3 scripts/realtime_detection_insightface.py
```

---

### 問題 3：OpenCV 沒有 GUI 支援

**症狀**: `cv2.imshow()` 報錯：`The function is not implemented`

**解決方案**:
```bash
# 卸載無 GUI 版本
pip uninstall opencv-python-headless -y

# 重新安裝有 GUI 版本
pip install --no-cache-dir opencv-python

# 驗證
python3 -c "import cv2; cv2.imshow('test', __import__('numpy').zeros((100,100), dtype='uint8'))"
```

---

### 問題 4：Picamera2 找不到

**症狀**: `ModuleNotFoundError: No module named 'picamera2'`

**解決方案**:
```bash
# 確保虛擬環境用了 --system-site-packages
python3 -m venv --system-site-packages ~/ai_bank_robot_env
source ~/ai_bank_robot_env/bin/activate

# 檢查系統 picamera2 是否已安裝
python3 -c "import sys; print(sys.path)"
```

---

### 問題 5：FPS 太低（<5）

**症狀**: 幀率低於預期

**解決方案**:
```bash
# 檢查系統資源
top -b -n 1 | head -15

# 檢查溫度（可能過熱）
vcgencmd measure_temp

# 降低檢測間隔（更多 CPU 使用）
# 編輯 realtime_detection_insightface.py
# 改 detect_interval = 3 → detect_interval = 2
```

---

## 📦 快速備份和轉移

### 備份當前環境

```bash
# 備份虛擬環境
tar -czf ai_bank_robot_env_backup.tar.gz ~/ai_bank_robot_env/

# 備份項目代碼
tar -czf ai_bank_robot_code_backup.tar.gz ~/ai_bank_robot/

# 大小約 500MB + 50MB
```

### 轉移到另一台樹莫派

```bash
# 在目標樹莫派上
mkdir -p ~/

# 從 Windows/Mac 傳輸（使用 scp）
scp -r ./ai_bank_robot xiange@<target_ip>:~/

# 運行快速部署
ssh xiange@<target_ip> "bash ~/ai_bank_robot/setup_rpi4_quick.sh"
```

---

## 📈 性能監控

安裝後，可以監控系統性能：

```bash
# 實時 FPS 和擦偵測結果
python3 scripts/realtime_detection_insightface.py

# 檢查資源使用
watch -n 1 'free -h; echo "---"; vcgencmd measure_temp'

# 查看檢測日誌
ls -lah /tmp/face_detection_*.jpg
```

---

## 🎯 下一步

部署完成後，接下來的開發步驟：

1. **Phase 2 - 活體防偽**
   ```bash
   # 後續實施 liveness_checker.py
   python3 scripts/liveness_checker.py
   ```

2. **Phase 2 - VIP 識別**
   ```bash
   # 後續實施 face_recognizer.py
   python3 scripts/face_recognizer.py
   ```

3. **Phase 3 - 表情分析**
   ```bash
   # 後續實施 emotion_analyzer.py
   python3 scripts/emotion_analyzer.py
   ```

---

## 📝 環境變數設置（可選）

創建 `.env` 文件以簡化配置：

```bash
# 在項目根目錄
cat > .env << 'EOF'
# 攝像頭配置
CAMERA_ID=0
FRAME_WIDTH=640
FRAME_HEIGHT=480
FPS_TARGET=10

# 檢測參數
FACE_CONFIDENCE_THRESHOLD=0.5
VIP_MATCH_THRESHOLD=0.6

# MQTT 配置
MQTT_BROKER=192.168.1.100
MQTT_PORT=1883
MQTT_USER=your_username
MQTT_PASSWORD=your_password

# 顯示配置
DISPLAY=:1
EOF

# 加載環境變數
source .env
```

---

## ✅ 完成檢查清單

- [ ] Git clone 或傳輸項目
- [ ] 運行 `bash setup_rpi4_quick.sh`
- [ ] 驗證所有套件導入成功
- [ ] 測試 `python3 scripts/realtime_detection_insightface.py`
- [ ] 檢查 FPS (應該 >10)
- [ ] 保存檢測結果 (S 鍵)
- [ ] 準備下一階段開發

---

## 💬 有問題？

如果遇到問題，檢查以下文件：

- `FEATURES.md` - 功能清單
- `requirements_minimal.txt` - 依賴版本
- `CLAUDE.md` - 架構和設計指南
- `vibe_coding流程.md` - 開發流程

---

**祝你部署順利！🚀**

