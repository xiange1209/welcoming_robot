# Phase 2b 完整測試計畫

## 📝 測試前準備

### 1️⃣ 準備照片（最重要！）

**需要準備的照片**：

```
photographs/
├── vip_1.jpg          # VIP 樣本 1：正面清晰人臉
├── vip_2.jpg          # VIP 樣本 2：另一個人（測試不同人臉識別）
├── blacklist_1.jpg    # 黑名單樣本（模擬詐騙犯/可疑人士）
├── visitor_1.jpg      # 訪客樣本（其他人，驗證不誤將為 VIP）
├── test_photo.jpg     # 用於活體檢測測試
└── low_quality.jpg    # 低質量照片（邊界情況）
```

**照片要求**：
- ✅ 正面、光線充足
- ✅ 無過度遮擋（口罩、太陽眼鏡可以，但會降低識別準確度）
- ✅ 解析度 640x480 以上
- ✅ JPEG 或 PNG 格式

### 2️⃣ 在樹莫派準備照片

```bash
# 在樹莓派上創建照片目錄
mkdir -p ~/ai_bank_robot/test_photos
cd ~/ai_bank_robot/test_photos

# 從 Windows 複製照片到樹莓派（或上傳圖像文件）
# scp photo.jpg xiange@192.168.1.125:~/ai_bank_robot/test_photos/
```

---

## 🧪 測試流程

### **階段 1：資料庫初始化** （5 分鐘）

```bash
# 1. SSH 連接樹莓派
ssh xiange@192.168.1.125
source ~/ai_bank_robot_env/bin/activate
cd ~/ai_bank_robot

# 2. 初始化數據庫
python3 scripts/manage_vip_database.py init
# 輸出：✓ 數據庫已初始化
```

---

### **階段 2：從照片添加 VIP** （10 分鐘）

```bash
# 添加 VIP #1：李美琪
python3 scripts/manage_vip_database.py add-vip-image "李美琪" test_photos/vip_1.jpg \
  --phone "0900123456" --email "lee@example.com" --level platinum

# 預期輸出：
# 提取 embedding 中...
#   讀取圖像：test_photos/vip_1.jpg (480, 640, 3)
#   初始化 InsightFace...
#   檢測人臉...
#   ✓ 成功提取 embedding (檢測信心度: 0.9872)
# ✓ VIP '李美琪' 已添加

# 添加 VIP #2：王建民
python3 scripts/manage_vip_database.py add-vip-image "王建民" test_photos/vip_2.jpg \
  --phone "0911234567" --level gold
```

**故障排除**：
- ❌ `無法讀取圖像` → 檢查路徑和文件是否存在
- ❌ `圖像中未檢測到人臉` → 照片品質不佳，換更清晰的
- ❌ `檢測到 2 張人臉` → 照片背景有其他人，重新拍照

---

### **階段 3：添加黑名單** （5 分鐘）

```bash
# 添加黑名單：詐騙犯
python3 scripts/manage_vip_database.py add-blacklist-image "詐騙犯 A" test_photos/blacklist_1.jpg \
  --reason "Known fraud case" --risk high

# 查看已添加的數據
python3 scripts/manage_vip_database.py list-vips
python3 scripts/manage_vip_database.py list-blacklist
```

---

### **階段 4：測試識別功能** （5 分鐘）

```bash
# 測試：用已存儲的 VIP embedding 進行識別
python3 scripts/manage_vip_database.py test

# 預期輸出：
# === 測試識別 ===
# 測試識別 VIP: 李美琪
# 識別結果:
#   類型: vip
#   名字: 李美琪
#   信心度: 1.0000
#   詳情: {'phone': '0900123456', ...}
```

---

### **階段 5：運行整合系統** （10-15 分鐘）

```bash
# 設置顯示
export DISPLAY=:1

# 運行主程式（包含檢測 + 識別 + 活體檢測）
python3 scripts/realtime_detection_insightface.py

# 預期輸出：
# ======================================================================
# 🎥 即時人臉檢測 (Picamera2 + InsightFace)
# ======================================================================
# 控制：
#   Q - 退出
#   S - 保存當前幀
#   空格 - 暫停/繼續
# ======================================================================
# ✓ Picamera2 已初始化 (640x480)
# ✓ InsightFace 已初始化 (RetinaFace + ArcFace)
# ✓ 人臉識別已初始化 (2 VIP, 1 黑名單)
# ✓ 活體檢測已初始化
```

**現場測試項目**：

| 測試項目 | 預期結果 | 實際結果 |
|--------|--------|--------|
| 顯示 VIP 李美琪的臉 | 🟡 黃色邊框 + 「VIP: 李美琪 (0.xx)」 | ✓ / ❌ |
| 顯示 VIP 王建民的臉 | 🟡 黃色邊框 + 「VIP: 王建民 (0.xx)」 | ✓ / ❌ |
| 顯示黑名單的臉 | 🔴 紅色邊框 + 「⚠ Blacklist: 詐騙犯 A (0.xx)」 | ✓ / ❌ |
| 顯示陌生人的臉 | 🟢 綠色邊框 + 「Visitor (0.xx)」 | ✓ / ❌ |
| **活體檢測** | | |
| VIP 人臉 + 眨眼 | 「✓ Alive (detected 3 actions)」 | ✓ / ❌ |
| VIP 人臉 + 靜止不動 | 「✗ Spoofing?」 | ✓ / ❌ |
| **性能指標** | | |
| FPS（偵測 VIP） | 10+ FPS | ___ FPS |
| 平均推理延遲 | < 350ms | ___ ms |
| CPU 使用率 | 60-80% | __% |

---

### **階段 6：性能優化測試** （可選）

```bash
# 測試多人情況（2 個 VIP 同時出現在攝像頭中）
# 記錄：
# - 是否全部正確識別？
# - FPS 是否下降？
# - 延遲是否超過 500ms?

# 測試邊界情況：
# - 低光環境
# - 側臉（45 度角）
# - 口罩遮擋
# - 人臉很小（離攝像頭遠）
```

---

## 📊 **測試結果記錄表**

請填入你的實測結果：

### 系統初始化
```
✓ Picamera2 初始化：____ ms
✓ InsightFace 初始化：____ ms
✓ Face Recognizer 初始化：____ ms
✓ Liveness Detector 初始化：____ ms
```

### 識別準確度
```
✓ VIP 識別準確度（應為 100%）：_____% (正確 __/3 人)
✓ 黑名單檢測準確度：_____% (正確 __/1 人)
✓ 訪客正確分類：_____% (正確 __/1 人)
```

### 性能指標
```
🎬 平均 FPS：________
⏱️  平均推理延遲：_______ ms
📊 CPU 使用率：_____% (目標 60-80%)
💾 內存占用：_____ MB (目標 < 500 MB)
🌡️  CPU 溫度：_____°C
```

### 活體檢測測試
```
✓ 眨眼檢測準確度：_____% (測試 __ 次)
✓ 搖頭檢測準確度：_____% (測試 __ 次)
✓ 張嘴檢測準確度：_____% (測試 __ 次)
✓ 總活體檢測通過率：_____% (應為 > 90%)
```

### 故障和邊界情況
```
- 低光環境：[通過 / 失敗 / 勉強可用]
- 側臉（45 度）：[通過 / 失敗 / 勉強可用]
- 口罩遮擋：[通過 / 失敗 / 勉強可用]
- 人臉很小：[通過 / 失敗 / 勉強可用]
- 多人同框：[通過 / 失敗 / 勉強可用]
```

---

## 💾 **保存測試結果**

測試完成後，請：

1. **記錄完整測試結果表**（上面的表格）
2. **保存偵測幀**（按 S 鍵保存）：
   ```bash
   # 查看保存的幀
   ls -lh /tmp/face_detection_*.jpg
   ```
3. **收集日誌**：
   ```bash
   # 如果需要詳細日誌，可以重定向輸出
   python3 scripts/realtime_detection_insightface.py > test_log.txt 2>&1
   ```

---

## ✅ 測試檢查清單

在推上 GitHub 前，確認：

- [ ] 資料庫初始化成功
- [ ] 至少 2 個 VIP 添加成功（從真實照片）
- [ ] 至少 1 個黑名單添加成功
- [ ] 識別函數測試通過（`test` 命令）
- [ ] 主程式運行 2 分鐘以上無崩潰
- [ ] 至少 1 個 VIP 被正確識別（黃邊框）
- [ ] 至少 1 個黑名單被正確識別（紅邊框）
- [ ] 活體檢測（眨眼/搖頭）正常工作
- [ ] FPS 維持在 10+ 以上
- [ ] 沒有 Python 運行時錯誤

---

## 🚀 測試完成後

1. **填寫上面的測試結果表**
2. **告訴我測試結果** - 準確度、FPS、任何問題
3. **我會根據結果修複問題**（如果有的話）
4. **然後一起推上 GitHub**

準備好開始測試了嗎？ 🎯

