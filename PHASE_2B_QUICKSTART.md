# Phase 2b 快速開始指南

## 📦 安裝依賴

```bash
# 激活虛擬環境
source ~/ai_bank_robot_env/bin/activate
cd ~/ai_bank_robot

# 安裝 MediaPipe（用於活體檢測）
pip install --no-cache-dir mediapipe

# 驗證安裝
python3 -c "import mediapipe; print('✓ MediaPipe ready')"
```

## 🗄️ 初始化 VIP 資料庫

```bash
# 初始化空的數據庫
python3 scripts/manage_vip_database.py init
```

## 📸 從照片添加 VIP（推薦方式）

```bash
# 添加 VIP（從真實人臉照片提取 embedding）
python3 scripts/manage_vip_database.py add-vip-image "李美琪" photo.jpg \
  --phone "0900123456" --email "lee@example.com" --level platinum

# 添加黑名單（從照片）
python3 scripts/manage_vip_database.py add-blacklist-image "詐騙犯" suspect.jpg \
  --reason "Known fraud case" --risk high
```

**照片要求**：
- ✅ 正面清晰人臉
- ✅ 光線充足，無過度遮擋
- ✅ 解析度 640x480 以上

## 🧪 測試識別功能

```bash
# 測試：用已存儲的 VIP embedding 進行識別
python3 scripts/manage_vip_database.py test

# 輸出應顯示：識別結果、信心度等
```

## 📊 查看 VIP 和黑名單

```bash
# 查看 VIP 列表
python3 scripts/manage_vip_database.py list-vips

# 查看黑名單
python3 scripts/manage_vip_database.py list-blacklist
```

## 🎥 執行整合的人臉檢測 + 識別 + 活體檢測

```bash
# 設置顯示
export DISPLAY=:1

# 運行主程式
python3 scripts/realtime_detection_insightface.py

# 控制
# Q - 退出
# S - 保存當前幀
# 空格 - 暫停/繼續
```

## 📊 預期輸出

程式會在邊界框上標註：
- **VIP**: 黃色邊框 + 「VIP: 名字 (信心度)」
- **黑名單**: 紅色邊框 + 「⚠ Blacklist: 名字」
- **訪客**: 綠色邊框 + 「Visitor」
- **活體狀態**: ✓ Alive 或 ✗ Spoofing?

## 🔍 故障排除

### 照片無法提取 embedding
```bash
# 確保照片路徑正確、格式支持（JPG/PNG）
file photo.jpg  # 檢查文件類型

# 確保照片中有清晰的人臉
# 可以用以下命令預覽檢測結果：
python3 -c "
import cv2
from insightface.app import FaceAnalysis
img = cv2.imread('photo.jpg')
face_app = FaceAnalysis(name='buffalo_sc', providers=['CPUExecutionProvider'])
face_app.prepare(ctx_id=-1, det_size=(512,512))
faces = face_app.get(img)
print(f'檢測到 {len(faces)} 張人臉')
for i, face in enumerate(faces):
    print(f'  人臉 {i+1}: 信心度 {face.det_score:.4f}')
"
```

### FPS 太低
- 確認檢測間隔設置（目前為每 3 幀檢測一次）
- 檢查 CPU 溫度（可能需要冷卻）

### 活體檢測失敗
- 確保光線充足
- 確認人臉在攝像頭範圍內並清晰
- MediaPipe 可能需要調整參數

## 📝 完整測試計畫

詳見 **TEST_PLAN.md** 瞭解：
- 完整的測試流程
- 性能指標記錄
- 邊界情況測試

---

## Phase 2b 完成清單

- [x] Face Recognizer（embedding 相似度比對）
- [x] Liveness Detector（眨眼/摇头/张嘴）
- [x] 整合的主程式
- [x] VIP 管理工具（支持從照片添加）
- [ ] 在 Raspberry Pi 4 上實際測試 ← **你來做**
- [ ] 驗證 10+ FPS 性能
- [ ] 驗證識別準確度
- [ ] 驗證活體檢測有效性

---

## 後續 Phase 3（計劃中）

- 表情分類（EfficientNet-B0）
- LoRa 通知（檢測到 VIP → 通知行員）
- 語音交互（Whisper STT + pyttsx3 TTS）

