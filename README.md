# 具人臉辨識與語音對話功能之迎賓機器人

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue)
![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu-24.04-orange)
![Python 3.12](https://img.shields.io/badge/Python-3.12-green)
![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%204B-red)

大學部畢業專題（2026 年 12 月發表）。一台 **WHEELTEC 阿克曼實車**擔任迎賓機器人：
用人臉辨識分辨來客身分、以語音與 LLM 對話、能自主導航帶位，平板即時顯示狀態。
應用場景設定為銀行大廳。

> *A Reception Robot with Face Recognition and Voice Interaction —
> Implementation on ROS 2 and Raspberry Pi*

---

## ⚠️ 先看這裡：這個儲存庫有兩個分支，內容完全不同

| 分支 | 內容 | 什麼時候用 |
|---|---|---|
| **`master`**（你現在看的） | **硬體層**：17 個 WHEELTEC 廠商套件、建置與實驗腳本 | 只要底盤／光達／相機／麥克風陣列的驅動 |
| **`車子`** | **實機開發主線**：完整 workspace（9 個自有套件）、導航重寫版、教導-重現路徑、實機驗證數據與交接文件 | **想看目前真正在跑的系統，看這個分支** |

兩個分支的 git 歷史**互相獨立**（沒有共同祖先），是同一個專案的兩個層次，不是新舊版本關係。

```bash
git clone -b 車子 https://github.com/xiange1209/welcoming_robot.git   # 實機主線
git clone -b master https://github.com/xiange1209/welcoming_robot.git # 硬體層（本分支）
```

應用層的上游在同學的 [swient/smartnav-bot](https://github.com/swient/smartnav-bot)。

---

## 📋 目錄

- [系統架構](#系統架構)
- [目前狀態](#目前狀態)
- [實測數據](#實測數據)
- [本分支內容](#本分支內容)
- [環境需求](#環境需求)
- [快速開始](#快速開始)
- [已知限制](#已知限制)
- [授權](#授權)

## 系統架構

```text
┌────────────────────────────┐   HTTP :11434   ┌────────────────────────┐
│ Raspberry Pi 4B (8GB)      │ ───────────────→│ Windows 筆電            │
│ Ubuntu 24.04 / ROS 2 Jazzy │                 │ RTX 3050 4GB           │
│ 跑全部 ROS 2 節點           │←─────────────── │ Ollama (qwen2.5:3b)    │
└──────────┬─────────────────┘    串流回覆      └────────────────────────┘
           │ USB
   ┌───────┴────────┬──────────────┬──────────────┐
   │ senior_akm 底盤 │ N10 光達      │ ASTRA S 相機  │   平板瀏覽器 → HMI
   └────────────────┴──────────────┴──────────────┘
```

**Pi 跑全部節點、筆電只當 Ollama 推理伺服器**——人臉辨識全在本機完成（隱私＋低延遲）。
分散式 ROS 2（Pi 輕量＋筆電重運算）評估後未採用，理由見下方[已知限制](#已知限制)。

### 事件流

```text
image_raw ─→ face_embedding_node ─→ face_embedding ─→ user_auth_node
   │            (InsightFace 512D)      (向量，不含身分)      (比對資料庫)
   │                                                             │
   │                                                             ▼
   │                                                      user_identity
   │                                              (姓名／類型／相似度／是否認出)
   │                                                             │
   │                              ┌──────────────────────────────┤
   │                              ▼                              ▼
   │                     bank_reception_node                 HMI 顯示
   │                （迎賓語音／通報／訪客記錄）
   │
麥克風 ─→ voice_trigger ─→ speech_recognizer ─→ user_text
          (喚醒詞＋VAD)        (ASR)                  │
                                                      ▼
                                        llm_service_node（可呼叫導航／銀行工具）
                                                      │
                              llm_response / llm_stream / speech_text
                                                      │
                              speech_synthesizer ─→ voice_playback ─→ 喇叭
```

## 目前狀態

> 本專案嚴格區分三種狀態：**已驗證**（有可重現的實機證據）、
> **已實作待驗證**（程式存在但沒有實機數據）、**未實作**。
> 每個數字都標來源。

### ✅ 已驗證

| 項目 | 數據 | 日期 |
|---|---|---|
| **教導-重現循跡精度** | 624 筆取樣，平均偏離 **3.42 cm**、最大 8.60 cm | 2026-08-03 |
| **地點停止誤差** | 0.17 m（容差設定 0.25 m） | 2026-08-03 |
| **AMCL 掃描過濾** | 地圖吻合度 71% → **90%** | 2026-08-03 |
| **建圖從不可用修到可用** | `map→odom` 最大單步修正 22.273 → **0.355 m** | 2026-07-31 |
| **IMU 零偏補償（ZUPT）** | 靜止 5 分鐘 EKF yaw 漂移 **0.047 度/分**（原始零偏 +9.56 度/分） | 2026-07-31 |
| 相機 CPU 優化 | 相機路徑 70.6% → **31.6%** | 2026-07-29 |
| 人臉辨識端到端 | 10 樣本註冊，65 次 100% 正確 | 2026-07 |
| 全 workspace 建置 | x86 VM 24/24＋arm64 QEMU 25/25 | 2026-07 |

### 🟡 已實作、待實機驗證

銀行場景劇本（VIP 迎賓／黑名單通報／訪客記錄／Telegram 推播）、LLM 銀行工具組、
HMI 教導路徑介面與全域緊急停止、窄轉角自動脫困。

### ❌ 未實作

巡邏模式整合、活體檢測、HMI 統計頁、地圖點選規劃的前端互動。

**實驗進度 3.5 / 8 組。**

## 實測數據

### 感測器與資源

| 項目 | 結果 |
|---|---|
| N10 光達 | 10.06 Hz、std dev 0.0013 s、360°（±π）、約 450 點 |
| ASTRA S 彩色（只開彩色） | **29.6 Hz**；同時開深度時彩色會完全斷流（USB 2.0 頻寬不足） |
| 關閉深度串流的效益 | 彩色 20.8→29.6 Hz、astra CPU 175%→71%、load average 19.4→4.45 |
| 人臉辨識（10 樣本註冊） | 65 次 100% 正確、置信度 0.671~0.875、平均 0.751 |
| 人臉辨識（單張照片註冊） | 0.602~0.723，**距門檻僅 0.002**，有誤判 |
| Pi4 資源（全節點運行） | load average 19.4（4 核超載 4.8 倍），記憶體僅 40% → **瓶頸在 CPU** |

> **正式展示務必用多樣本註冊**——單張照片的置信度距門檻只有 0.002，隨時會誤判。

### LLM 模型延遲（RTX 3050 4GB）

方法：`scripts/benchmark_llm_models.py`，每模型 1 次暖機＋5 次正式，串流 `/api/chat`，
`temperature 0.1`、`num_ctx 512`。

| 模型 | 首次輸出平均 (ms) | 總生成平均 (ms) | 成功率 |
|---|---:|---:|---:|
| **qwen2.5:3b** | **2218** | **3076** | 100% |
| gemma3:4b | 2303 | 4700 | 100% |
| gemma4:e2b | 42235 | 47909 | 100% |
| gemma4:e4b | 95066 | 107123 | 100% |

gemma4 系列模型檔（7.2 GB／9.6 GB）遠超 4 GB VRAM，被卸載到 CPU 而慢 20~40 倍。
**結論：用 `qwen2.5:3b`。**

### LLM 工具呼叫（qwen2.5:3b，2×2 因子實驗）

| | 3 個工具 | 10 個工具 |
|---|:---:|:---:|
| 無串流回呼 | ✓ | ✗ |
| 有串流回呼 | ✓ | ✗ |

**決定性變因是工具數量，與是否串流無關。** 小模型綁超過約 5 個工具就選不出正確工具。

## 本分支內容

```
.
├── src/            17 個 WHEELTEC 廠商套件（修正為 Jazzy 相容）
│   ├── turn_on_wheeltec_robot/   底盤驅動（senior_akm 阿克曼）
│   ├── wheeltec_robot_urdf/      車體模型
│   ├── ldlidar_ros2/             N10 光達
│   ├── lslidar_driver/           鐳射雷達備援
│   ├── astra_camera/             Orbbec ASTRA S 深度相機
│   ├── wheeltec_mic_ros2/        六麥克風環形陣列
│   ├── wheeltec_robot_nav2/      nav2 參數
│   ├── wheeltec_slam_toolbox/    SLAM 封裝
│   ├── nav2_waypoint_cycle/      巡邏（現成待接）
│   ├── ollama_ros_chat/          單輪對話備援
│   └── …                         serial、msgs 等
├── scripts/
│   ├── benchmark_llm_models.py   LLM 延遲基準
│   ├── build_arm64.sh            QEMU 容器交叉建置 arm64
│   ├── register_from_photo.py    從照片檔註冊人臉
│   ├── exp_face_logger.py        人臉辨識實驗記錄器
│   └── download_audio_models.sh  sherpa-onnx 模型下載
├── .clinerules     開發規範
└── LICENSE         MIT
```

廠商原始包共 97 個套件（Humble 世代），精選 17 個修正後納入——
其餘含會與 Jazzy 衝突的 navigation2-humble，不可直接建置。

## 環境需求

| 項目 | 需求 |
|---|---|
| Raspberry Pi 4B（8 GB） | Ubuntu 24.04.4、ROS 2 Jazzy、`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` |
| Windows 筆電 | Ollama（`qwen2.5:3b`），需設 `OLLAMA_HOST=0.0.0.0` 才能被 Pi 連到 |
| 車體 | WHEELTEC senior_akm 阿克曼底盤＋N10 光達＋Orbbec ASTRA S |
| Python 相依 | `insightface`、`onnxruntime`、`sherpa-onnx`、`langchain`、`fastapi` |

## 快速開始

> **要跑完整系統請切到 `車子` 分支**，那裡有整合好的 workspace 與啟動腳本。
> 本分支只提供硬體層驅動。

```bash
# 建置（本分支）
colcon build --symlink-install
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 相機（只開彩色——同時開深度會超過 USB 2.0 頻寬導致彩色斷流）
ros2 launch astra_camera astra.launch.xml \
  enable_depth:=false enable_point_cloud:=false enable_ir:=false color_fps:=15

# 底盤
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
```

### 底盤操作

阿克曼車：後輪驅動、前輪只打角、**不能原地旋轉**。

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p repeat_rate:=20.0
#   i 前進   , 後退   u 前進+左轉   o 前進+右轉   k 停止
#   ★ j / l 是原地旋轉，阿克曼車做不到，按了沒反應是正確的
#   ★ repeat_rate 必加——韌體指令逾時剛好 1.0 秒，不持續送車子會停
```

### 人臉註冊

```bash
ros2 service call /register_face smartnav_msgs/srv/RegisterFace \
  "{user_name: '測試人員', user_type: {type: 1}, description: '', num_samples: 10}"
```

呼叫後**立刻站到相機前並持續站著**（約 20~30 秒），稍微轉頭讓樣本多樣化。
`user_type`：0=GUEST、1=VIP、2=ADMIN、3=BLACKLIST。

## 已知限制

1. **numpy 版本衝突**：`insightface`／`onnxruntime` 會裝進 numpy 2.x，而 apt 的 `cv_bridge`
   是對 numpy 1.x 編譯的 C 擴充，衝突時節點一收到影像就 segfault。
   **驗證相容性必須跑到真正的 C 執行路徑，只 `import` 成功沒有意義。**
2. **USB 2.0 頻寬**：ASTRA S 同時開深度＋彩色會超過頻寬導致彩色斷流；只開彩色反而更快。
3. **阿克曼運動限制**：最小迴轉半徑（韌體值）**0.750 m**，掉頭需要 1.6 m，
   而測試走廊淨寬僅 0.99 m。解法是**教導-重現路徑**——錄下人開過的位姿再重播，
   不讓控制器自己規劃。
4. **MPPI 在窄空間規劃不出來**：預測視野 `time_steps(30) × model_dt(0.1) × vx_max(0.25) = 0.75 m`，
   而完成 90° 轉彎需要 `(π/2) × 0.80 = 1.26 m`——**控制器看得到的距離只有所需的 60%**。
   要補足需 `time_steps ≈ 50`（前向模擬 15000 次），Pi4 上不划算。這是架構限制，不是調參問題。
5. **低電壓 20 V 只切馬達不切舵機**（韌體行為）：症狀是「方向舵會轉但車不走」。
   遇到先量電池，不要查 ROS。
6. **開機後 1.0~2.0 秒車子會自己前進 0.02 m/s**：廠商自檢動作，不是故障。
   2.5 秒後才接受 ROS 指令。**不要放在桌上開機。**
7. **slam_toolbox 在 Jazzy 是 lifecycle 節點**，廠商 Humble 世代 launch 不會自動 activate，
   症狀是 `/map` 話題不存在。
8. **兩條導航鏈不可混用**：WHEELTEC 鏈全程用 `odom_combined` frame（EKF 輸出），
   模擬用的 `smartnav_navigation` 用 `odom`。
9. **`--symlink-install` 後不可改名或搬移 workspace**，否則 install 內連結全懸空
   （`ls` 看得到但 `test -f` 失敗）。
10. **分散式 ROS 2 未採用**：VMware NAT 下跨機 DDS 不通、需 bridged 網路；
    已驗證的組態是「Pi 跑全部節點＋筆電只供 Ollama」。

## 授權

本專案採用 [MIT License](LICENSE)。開發規範見 [`.clinerules`](.clinerules)。
廠商套件（`src/` 下的 WHEELTEC 程式碼）依其原始授權條款。

外部參考：
[swient/smartnav-bot](https://github.com/swient/smartnav-bot)（應用層上游）·
[ROS 2 Jazzy](https://docs.ros.org/en/jazzy/) ·
[Nav2](https://navigation.ros.org/) ·
[InsightFace](https://github.com/deepinsight/insightface) ·
[sherpa-onnx](https://github.com/k2-fsa/sherpa-onnx) ·
[Ollama](https://ollama.com/)
