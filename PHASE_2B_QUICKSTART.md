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
# 完整快速開始：初始化 + 添加測試數據
python3 scripts/manage_vip_database.py create-test-data

# 查看 VIP 列表
python3 scripts/manage_vip_database.py list-vips

# 查看黑名單
python3 scripts/manage_vip_database.py list-blacklist

# 測試識別功能
python3 scripts/manage_vip_database.py test
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
- **活體狀態**: ✓ Alive（通過活體檢測）或 ✗ Spoofing?

## 🔍 故障排除

### MediaPipe 找不到
```bash
pip list | grep mediapipe
# 如果沒有，重新安裝
pip install --no-cache-dir mediapipe
```

### 數據庫不存在
```bash
# 初始化數據庫
python3 scripts/manage_vip_database.py init
```

### 活體檢測失敗
- 確保光線充足
- 確認人臉在攝像頭範圍內
- 可能需要調整 `eye_ar_threshold` 和 `mouth_ar_threshold`

## 📝 自定義：添加特定人員

```bash
# 手動添加 VIP
python3 scripts/manage_vip_database.py add-vip

# 手動添加黑名單
python3 scripts/manage_vip_database.py add-blacklist
```

---

## Phase 2b 完成清單

- [x] Face Recognizer（embedding 相似度比對）
- [x] Liveness Detector（眨眼/摇头/张嘴）
- [x] realtime_detection_insightface.py（集成識別+活體檢測）
- [x] VIP 資料庫管理工具
- [ ] 在 Raspberry Pi 4 上實際測試
- [ ] 性能優化（保持 10+ FPS）
- [ ] 邊界情況測試（多人、側臉、低光）

---

## 後續 Phase 3（計劃中）

- 表情分類（EfficientNet-B0）
- LoRa 通知（檢測到 VIP → 通知行員）
- 語音交互（Whisper STT + pyttsx3 TTS）
