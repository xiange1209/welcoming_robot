# Phase 2b 功能進度報告

## 📊 目前進度

### ✅ 已完成的核心代碼

| 模組 | 功能 | 狀態 |
|-----|------|------|
| **vision_ai/face_recognizer.py** | Embedding 相似度比對 VIP/黑名單 | ✅ 完成 |
| **vision_ai/liveness_detector.py** | 眨眼/搖頭/張嘴活體檢測 | ✅ 完成 |
| **scripts/realtime_detection_insightface.py** | 整合檢測+識別+活體系統 | ✅ 完成 |
| **scripts/manage_vip_database.py** | VIP 管理工具 + **從照片提取 embedding** | ✅ 完成 |

### 📖 已新增的文檔

| 文檔 | 用途 |
|-----|------|
| **PHASE_2B_QUICKSTART.md** | 快速開始指南 |
| **TEST_PLAN.md** | 完整測試計畫（你需要按此進行） |
| **scripts/demo_phase2b.py** | 離線功能演示（可在 Windows 上執行） |

---

## 🎯 你現在需要做什麼

### **第 1 步：準備照片** （重要！）
```
準備 2-3 張清晰的人臉照片
- vip_1.jpg     (你或同事的臉，作為 VIP 樣本)
- vip_2.jpg     (另一個人，驗證多人識別)
- blacklist.jpg (第三個人，作為黑名單樣本)
- visitor.jpg   (驗證陌生人不被誤識別)

要求：
✓ 正面清晰
✓ 光線充足
✓ 640x480 以上解析度
✓ JPG 或 PNG 格式
```

### **第 2 步：在樹莫派上執行測試**

按照 **TEST_PLAN.md** 的步驟執行：

```bash
# SSH 連接樹莓派
ssh xiange@192.168.1.125
source ~/ai_bank_robot_env/bin/activate
cd ~/ai_bank_robot

# 1. 安裝 MediaPipe
pip install --no-cache-dir mediapipe

# 2. 初始化數據庫
python3 scripts/manage_vip_database.py init

# 3. 從照片添加 VIP
python3 scripts/manage_vip_database.py add-vip-image "你的名字" vip_1.jpg

# 4. 運行主程式（10-15 分鐘的實時檢測測試）
export DISPLAY=:1
python3 scripts/realtime_detection_insightface.py
```

### **第 3 步：記錄測試結果**

使用 **TEST_PLAN.md** 中的表格記錄：
- ✓ 識別準確度（VIP/黑名單是否正確識別）
- ✓ FPS（應該 10+ FPS）
- ✓ 活體檢測（眨眼/搖頭是否檢測到）
- ✓ 任何問題或邊界情況

### **第 4 步：告訴我結果**

告訴我：
1. 識別準確度如何？
2. FPS 是多少？
3. 有任何錯誤或問題嗎？
4. 活體檢測效果如何？

基於你的反饋，我會：
- 修復任何 bug
- 調整參數以提高性能
- 然後推上 GitHub

---

## 🆘 如果遇到問題

### 照片無法提取 embedding
```bash
# 檢查照片是否有人臉
python3 -c "
import cv2, sys
from insightface.app import FaceAnalysis
img = cv2.imread('photo.jpg')
if img is None: print('❌ 無法讀取照片')
else:
    face_app = FaceAnalysis(name='buffalo_sc', providers=['CPUExecutionProvider'])
    face_app.prepare(ctx_id=-1)
    faces = face_app.get(img)
    print(f'✓ 檢測到 {len(faces)} 張人臉')
"
```

### MediaPipe 安裝失敗
```bash
# 重新安裝
pip install --no-cache-dir mediapipe
python3 -c "import mediapipe; print('✓')"
```

### FPS 太低
- 檢查 CPU 溫度（vcgencmd measure_temp）
- 確認是否有其他進程占用 CPU
- 可以減少檢測頻率（modify detect_interval）

---

## 本地演示（可選，在 Windows 上執行）

如果想先在 Windows 上看看功能是否正常運行：

```bash
cd C:\Users\陳佳憲\Desktop\專題

# 需要先安裝依賴
pip install mediapipe insightface opencv-python numpy

# 運行演示（不需要攝像頭，只是驗證代碼邏輯）
python3 scripts/demo_phase2b.py

# 應該輸出：
# ✅ 所有離線演示通過
```

---

## 📋 當前代碼架構

```
vision_ai/
├── face_recognizer.py      # Embedding 相似度比對
│   ├── recognize_face()    # VIP/黑名單匹配
│   ├── add_vip()          # 添加 VIP
│   └── add_blacklist()    # 添加黑名單
└── liveness_detector.py    # 活體檢測
    ├── check_liveness()   # 眨眼/搖頭/張嘴
    ├── eye_aspect_ratio() # EAR 計算
    └── mouth_aspect_ratio() # MAR 計算

scripts/
├── realtime_detection_insightface.py  # 主程式（檢測+識別+活體）
├── manage_vip_database.py             # VIP 管理（新增：從照片提取 embedding）
└── demo_phase2b.py                    # 本地演示

database/
└── ai_bank_robot.db                   # SQLite（自動初始化）
    ├── vip_members          # VIP 表（名稱、embedding、電話、郵箱等）
    ├── blacklist            # 黑名單表（名稱、embedding、風險級別等）
    └── visit_logs           # 訪客日誌（可選，用於記錄）
```

---

## ⏱️ 預期時間表

| 階段 | 任務 | 時間 |
|-----|------|------|
| 1 | 準備照片 | 5-10 分鐘 |
| 2 | 樹莫派初始化 + 添加 VIP | 10 分鐘 |
| 3 | 實時檢測測試（10-15 分鐘的現場測試） | 15 分鐘 |
| 4 | 記錄結果 + 報告 | 5 分鐘 |
| **總計** | | **35-40 分鐘** |

---

## ✅ 測試完成後可以推上 GitHub

一旦你完成測試和報告，我會：
1. ✅ 確認所有功能正常
2. ✅ 修復任何發現的問題
3. ✅ 添加測試結果到 git commit
4. ✅ 推上 GitHub

---

## 📌 記住！

**最重要的是照片質量**
- 1 張高質量照片 > 10 張低質量照片
- 正面清晰 = 識別準確度 95%+
- 側臉模糊 = 可能識別失敗

準備好開始了嗎？🚀
