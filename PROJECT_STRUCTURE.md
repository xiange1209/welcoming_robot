# 📁 智慧銀行 AI 機器人 - 項目結構 (已整理)

**清理日期**: 2026-04-06
**狀態**: ✅ 已移除9個舊版本/重複檔案，保留核心模塊

---

## 目錄結構

```
smart-bank-robot/
│
├── 📂 vision_ai/                      ← AI 視覺模塊 (負責人: 你)
│   ├── face_detector.py               ✅ 人臉檢測 (RetinaFace)
│   ├── face_recognizer.py            ✅ VIP 識別 (embedding 比對)
│   ├── liveness_detector.py          ✅ 活體檢測 (眨眼/搖頭/張嘴)
│   ├── inference_engine.py           ✅ 推理引擎抽象層
│   └── __init__.py
│
├── 📂 database/                       ← 數據庫管理
│   ├── schema.py                     ✅ SQLite 表定義
│   └── __init__.py
│
├── 📂 scripts/                        ← 運行腳本
│   ├── realtime_detection_insightface.py  ✅ 實時人臉檢測 (主要腳本)
│   ├── manage_vip_database.py         ✅ VIP 數據庫工具
│   ├── test_vip_recognition.py       ✅ 識別測試腳本
│   ├── benchmark.py                  ✅ 性能測試
│   ├── demo_phase2b.py               📝 Phase 2b 演示 (可選)
│   ├── setup_rpi4_quick.sh           ✅ 環境搭建腳本
│   └── setup_rpi4_dev.sh             ✅ 完整設置腳本
│
├── 📂 config/                         ← 配置文件
│   └── inference_config.yaml         ✅ 推理參數配置
│
├── 📂 core_workflow/ (placeholder)    ← 核心邏輯 (計劃中)
├── 📂 communication/ (placeholder)    ← 通訊模塊 (計劃中)
├── 📂 robot_navigation/ (placeholder) ← 導航模塊 (隊友負責)
├── 📂 speech_hmi/ (placeholder)       ← 語音/HMI (計劃中)
├── 📂 backend_server/ (placeholder)   ← 後端服務 (計劃中)
│
├── 📖 重要文檔
│   ├── 專題計畫.md                  📌 完整規劃書 (8 phases)
│   ├── CLAUDE.md                    📌 Claude 開發指南
│   ├── FEATURES.md                  📝 功能清單
│   ├── README.md                    📝 項目入口
│   ├── vibe_coding流程.md           📝 開發工作流
│   ├── TEST_PLAN.md                 📝 Phase 2b 測試計劃
│   ├── TESTING_READY.md             📝 進度報告
│   ├── PHASE_2B_QUICKSTART.md       📝 快速參考
│   └── INSTALLATION_GUIDE.md        📝 安裝指南
│
└── 📦 其他
    └── requirements_minimal.txt (if exists)

```

---

## ✅ 核心功能模塊 (保留)

| 模塊 | 檔案 | 狀態 | 用途 |
|------|------|------|------|
| **人臉檢測** | `vision_ai/face_detector.py` | ✅ 運行中 | InsightFace RetinaFace 檢測 |
| **VIP 識別** | `vision_ai/face_recognizer.py` | ⏳ 待測試 | Embedding 向量相似度比對 |
| **活體檢測** | `vision_ai/liveness_detector.py` | ⏳ 待測試 | 眨眼/搖頭/張嘴檢測 |
| **推理引擎** | `vision_ai/inference_engine.py` | 📝 框架 | 平台抽象層 (RPi4/Jetson) |
| **數據庫** | `database/schema.py` | 📝 框架 | VIP/黑名單/日誌存儲 |
| **實時檢測** | `scripts/realtime_detection_insightface.py` | ✅ 主腳本 | 攝像頭實時推理 |
| **VIP 工具** | `scripts/manage_vip_database.py` | 📝 工具 | 從照片註冊 VIP |
| **識別測試** | `scripts/test_vip_recognition.py` | ✅ 測試 | 驗證識別邏輯 |

---

## 🗑️ 已刪除的檔案 (2026-04-06)

```
❌ scripts/realtime_detection.py                  (舊版)
❌ scripts/realtime_detection_opencv_dnn.py       (替代方案，未用)
❌ scripts/realtime_detection_picamera2.py        (舊版)
❌ scripts/realtime_detection_yolov8.py           (替代方案，未用)
❌ scripts/realtime_detection_yolov8_onnx.py      (替代方案，未用)
❌ scripts/vip_database_manager.py                (已被 manage_vip_database.py 取代)
❌ scripts/download_models.py                     (重複功能)
❌ scripts/model_guide.py                         (未用)
❌ vision_ai/face_detector_mediapipe.py           (舊版)
```

---

## 📋 使用流程

### 在 Raspberry Pi 4 上運行

**1. 初始化環境**
```bash
source ~/ai_bank_robot_env/bin/activate
cd ~/ai_bank_robot
```

**2. 初始化數據庫**
```bash
python3 scripts/manage_vip_database.py init
```

**3. 註冊 VIP**
```bash
python3 scripts/manage_vip_database.py add-vip-image "李美琪" photo.jpg \
  --phone "0900123456" --email "lee@bank.com" --level platinum
```

**4. 測試識別**
```bash
python3 scripts/test_vip_recognition.py test_photos/lee.jpg
```

**5. 實時檢測**
```bash
export DISPLAY=:1
python3 scripts/realtime_detection_insightface.py
```

---

## 📌 下一步

**明天 (2026-04-07)**: 執行 Phase 2b 功能可行性測試
- 準備測試照片
- 驗證 VIP 識別是否工作
- 驗證數據庫端到端流程
- 整合實時檢測

詳見: `TEST_PLAN.md`

---

## 📞 快速參考

| 需求 | 文檔 |
|------|------|
| 我想了解完整項目 | 📌 `專題計畫.md` |
| 我想快速開始 | `PHASE_2B_QUICKSTART.md` |
| 我想執行測試 | `TEST_PLAN.md` |
| 我想了解開發流程 | `vibe_coding流程.md` |
| 我需要安裝指南 | `INSTALLATION_GUIDE.md` |
| 我需要 Claude 幫助 | `CLAUDE.md` |
