# SmartNav - 智慧導航機器人系統

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue)
![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu-24.04-orange)
![Python 3.12](https://img.shields.io/badge/Python-3.12-green)

**SmartNav** 是一個基於 ROS 2 Jazzy 的智慧導航機器人系統，整合了視覺辨識、語音介面與自主導航功能，實現高度智能化的人機互動與環境感知能力。

## 📋 目錄

- [主要功能](#-主要功能)
- [系統架構](#️-系統架構)
- [套件說明](#-套件說明)
- [環境需求](#-環境需求)
- [安裝與建置](#-安裝與建置)
- [快速開始](#-快速開始)
- [配置說明](#️-配置說明)
- [主題與服務](#-主題與服務)
- [開發指南](#-開發指南)
- [故障排除](#️-故障排除)

## 🎯 主要功能

### 視覺模組 (`smartnav_vision`)

- **人臉檢測與識別**：使用 InsightFace 模型進行實時人臉偵測與身份識別
- **特徵提取**：高維度人臉特徵向量萃取與儲存
- **動態資料庫**：支援新人員動態註冊與特徵向量管理
- **GPU 加速**：支援 CUDA/GPU 加速推理以提升性能

### 語音模組 (`smartnav_audio`)

- 語音捕捉與處理
- 語音指令識別與執行基礎框架

### 決策模組 (`smartnav_brain`)

- 融合多模態感測資訊進行決策
- 與 Nav2 整合支援自主導航
- 高層次任務規劃與執行

### 啟動管理 (`smartnav_bringup`)

- 統一的系統啟動與配置管理
- 整合 Nav2 導航棧
- 參數與組態集中管理

### 資訊介面 (`smartnav_msgs`)

- 自定義 ROS 2 訊息與服務格式
- 人臉註冊服務 (`RegisterFace.srv`)
- 系統間通訊協定規範

## 🏗️ 系統架構

```text
SmartNav System Architecture
│
├─ 感測層 (Perception Layer)
│  ├─ 相機驅動 → Image Stream
│  ├─ 麥克風驅動 → Audio Stream
│  └─ IMU/里程計 → Odometry
│
├─ 處理層 (Processing Layer)
│  ├─ smartnav_vision: 視覺處理
│  │  ├─ face_recognition_node: 人臉識別
│  │  └─ face_registration_node: 人臉註冊
│  ├─ smartnav_audio: 語音處理
│  └─ smartnav_brain: 決策引擎
│
├─ 導航層 (Navigation Layer)
│  └─ Nav2 計畫規劃器、控制器
│
└─ 執行層 (Execution Layer)
   └─ 機器人底盤驅動與執行機構
```

## 📦 套件說明

### [`smartnav_vision`](src/smartnav_vision/) - 視覺核心模組

**功能**：實時人臉檢測、特徵提取與身份識別

**主要節點**：

- [`face_recognition_node`](src/smartnav_vision/smartnav_vision/face_recognition_node.py)：訂閱相機影像，執行實時人臉識別
- [`face_registration_node`](src/smartnav_vision/smartnav_vision/face_registration_node.py)：管理新人員人臉特徵註冊

**核心模組**：

- [`face_engine.py`](src/smartnav_vision/smartnav_vision/face_engine.py)：InsightFace 引擎封裝
- [`database_manager.py`](src/smartnav_vision/smartnav_vision/database_manager.py)：人臉特徵向量資料庫管理
- [`face_utils.py`](src/smartnav_vision/smartnav_vision/face_utils.py)：通用人臉處理工具函數

**依賴**：

- `rclpy`：ROS 2 Python 用戶端庫
- `opencv-python`：影像處理
- `insightface`：人臉檢測與識別
- `sensor_msgs`, `vision_msgs`：ROS 2 訊息類型

### [`smartnav_audio`](src/smartnav_audio/) - 語音介面模組

**功能**：語音捕捉、處理與基礎語音指令識別框架

**依賴**：

- `rclpy`：ROS 2 Python 用戶端庫
- `std_msgs`：標準訊息類型

### [`smartnav_brain`](src/smartnav_brain/) - 決策引擎模組

**功能**：高層次任務規劃、多模態融合決策與導航控制

**依賴**：

- `rclpy`：ROS 2 Python 用戶端庫
- `nav2_msgs`：Nav2 導航訊息
- `geometry_msgs`：幾何訊息（位置、姿態）
- `std_msgs`：標準訊息類型

### [`smartnav_bringup`](src/smartnav_bringup/) - 系統啟動配置

**功能**：統一的系統啟動管理與參數配置

**內容**：

- Launch 檔案：系統級別的節點啟動與參數配置
- 參數配置檔案：各模組的初始化參數

**依賴**：

- `rclpy`：ROS 2 Python 用戶端庫
- `nav2_bringup`：Nav2 啟動依賴

### [`smartnav_msgs`](src/smartnav_msgs/) - 資訊介面定義

**功能**：系統級自定義訊息與服務規範

**定義的服務**：

- [`RegisterFace.srv`](src/smartnav_msgs/srv/RegisterFace.srv)：人臉註冊服務
  - **請求**：人員名稱 (`string`)、採樣數量 (`int32`)
  - **回應**：成功標誌 (`bool`)、訊息 (`string`)、人員 UUID (`string`)

## 🔧 環境需求

### 系統需求

- **作業系統**：Ubuntu 24.04 LTS (Noble Numbat)
- **ROS 2**：Jazzy Jalisco
- **Python**：3.12+
- **CUDA**（可選）：11.8+ （用於 GPU 加速）
- **cuDNN**（可選）：用於深度學習加速

### 依賴軟體包

```text
# 核心 ROS 2 依賴
- rclpy
- sensor_msgs
- geometry_msgs
- std_msgs
- vision_msgs
- nav2_msgs
- cv_bridge
- rcl_interfaces

# 視覺處理
- opencv-python
- insightface
- onnxruntime（或 onnxruntime-gpu）
- numpy

# Nav2 導航棧
- nav2_bringup
- nav2_core
```

## 📥 安裝與建置

### 1. 預備工作

確保已安裝 ROS 2 Jazzy 與必要的工具：

```bash
# 新增 ROS 2 GPG 金鑰
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# 新增 ROS 2 Ubuntu 24.04 套件庫
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

# 更新套件列表並安裝 ROS 2
sudo apt update
sudo apt install ros-jazzy-desktop
```

### 2. 複製專案倉庫

```bash
cd ~
git clone https://github.com/swient/smartnav-bot.git smartnav_ws
cd smartnav_ws
```

### 3. 安裝依賴

```bash
# 安裝 rosdep 依賴
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

### 4. 建置專案

```bash
# 進入工作區目錄
cd ~/smartnav_ws

# 建置所有套件
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 或針對特定套件建置
colcon build --packages-select smartnav_msgs smartnav_vision --symlink-install
```

### 5. 設定環境

```bash
# 在每個新終端視窗中執行
source ~/smartnav_ws/install/setup.bash
```

建議使用以下幾行以自動載入：

```bash
echo "source ~/smartnav_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 🚀 快速開始

### 啟動人臉識別系統

#### 方式 1：使用 bringup 啟動（待實作）

```bash
# 終端 1：啟動整個系統
source /opt/ros/jazzy/setup.bash
ros2 launch smartnav_bringup smartnav_system.launch.py
```

#### 方式 2：個別啟動各模組

```bash
# 終端 1：啟動人臉識別節點
source /opt/ros/jazzy/setup.bash
ros2 run smartnav_vision face_recognition_node --ros-args -p image_topic:=/camera/image_raw -p enable_gpu:=true
```

### 註冊新人員人臉

```bash
# 呼叫人臉註冊服務
ros2 service call /face_registration/register smartnav_msgs/srv/RegisterFace "{person_name: 'John Doe', num_samples: 10}"
```

### 查看識別結果

```bash
# 訂閱除錯影像（若已啟用）
ros2 topic echo /face_recognition/debug_image
```

### 運行測試

```bash
# 執行單元測試
colcon test --packages-select smartnav_vision

# 顯示測試結果
colcon test-result --verbose
```

## ⚙️ 配置說明

### 人臉識別節點參數 (`face_recognition_node`)

| 參數名稱 | 型態 | 預設值 | 說明 |
| --- | --- | --- | --- |
| `image_topic` | `string` | `/image_raw` | 輸入相機影像的 ROS 2 主題名稱 |
| `model_name` | `string` | `buffalo_sc` | InsightFace 模型名稱（支援：`buffalo_sc`, `buffalo_l`） |
| `confidence_threshold` | `double` | `0.5` | 人臉檢測信心閾值（0.0-1.0） |
| `enable_gpu` | `bool` | `true` | 是否啟用 GPU 加速推理 |
| `recognition_threshold` | `double` | `0.6` | 人臉識別的相似度閾值（0.0-1.0） |
| `publish_debug_image` | `bool` | `false` | 是否發佈帶有檢測框的除錯影像 |

### 使用 YAML 參數檔案配置

建立 `face_recognition_config.yaml`：

```yaml
face_recognition_node:
  ros__parameters:
    image_topic: /camera/image_raw
    model_name: buffalo_sc
    confidence_threshold: 0.5
    enable_gpu: true
    recognition_threshold: 0.6
    publish_debug_image: true
```

啟動時指定配置檔：

```bash
ros2 run smartnav_vision face_recognition_node --ros-args \
  --params-file face_recognition_config.yaml
```

## 📡 主題與服務

### 訊息主題

#### 人臉識別節點

| 主題名稱 | 訊息型態 | 方向 | 說明 |
| --- | --- | --- | --- |
| `/image_raw` | `sensor_msgs/Image` | 訂閱 | 輸入相機影像串流 |
| `/face_recognition/debug_image` | `sensor_msgs/Image` | 發佈 | 帶有人臉檢測框與身份標籤的除錯影像（可選） |

### 服務

| 服務名稱 | 服務型態 | 說明 |
| --- | --- | --- |
| `/face_recognition/refresh_cache` | `std_srvs/Empty` | 重新載入資料庫並刷新特徵向量快取 |
| `/face_registration/register` | `smartnav_msgs/srv/RegisterFace` | 註冊新人員人臉特徵 |

### 服務使用範例

```bash
# 刷新人臉識別快取
ros2 service call /face_recognition/refresh_cache std_srvs/srv/Empty

# 註冊新人員（10 個臉部樣本）
ros2 service call /face_registration/register smartnav_msgs/srv/RegisterFace \
  "{person_name: 'Alice', num_samples: 10}"
```

## 👨‍💻 開發指南

### 代碼風格與文檔化

本專案遵循嚴格的開發規範，詳見 [`.clinerules`](.clinerules)：

#### Python 風格

- 遵循 **PEP 8** 編碼風格
- 使用 **Google Style Python Docstrings** 文檔化
- 盡可能使用 type hints

**示例**：

```python
def recognize_face(self, embedding: np.ndarray) -> tuple[Optional[str], float]:
    """識別臉部身份
    
    將輸入特徵向量與快取中所有已註冊人員比較。
    
    Args:
        embedding: 臉部特徵向量 (512維或其他維度)
    
    Returns:
        tuple[Optional[str], float]: (人員UUID, 相似度分數)
        若相似度低於閾值則返回 (None, 相似度)
    
    Raises:
        ValueError: 當嵌入向量維度不符時
    """
```

### 新增功能步驟

1. **規劃設計**：設計新功能的 ROS 2 介面（主題、服務、參數）
2. **定義訊息**：若需自定義訊息/服務，在 `smartnav_msgs` 中定義
3. **實現功能**：在相應套件中編寫節點或模組代碼
4. **測試驗證**：編寫單元測試並驗證功能
5. **文檔更新**：更新 README 與代碼文檔

### 專案結構佈局

```text
smartnav_ws/
├── src/
│   ├── smartnav_msgs/              # 訊息介面定義
│   │   ├── srv/                    # 服務定義
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── smartnav_vision/            # 視覺模組（Python）
│   │   ├── smartnav_vision/        # 套件代碼
│   │   │   ├── face_engine.py      # InsightFace 引擎
│   │   │   ├── database_manager.py # 特徵向量資料庫管理
│   │   │   ├── face_recognition_node.py
│   │   │   ├── face_registration_node.py
│   │   │   └── __init__.py
│   │   ├── test/                   # 單元測試
│   │   ├── setup.py
│   │   ├── setup.cfg
│   │   └── package.xml
│   ├── smartnav_audio/             # 語音模組（Python）
│   ├── smartnav_brain/             # 決策模組（Python）
│   └── smartnav_bringup/           # 系統啟動套件（目前為基礎套件結構）
│       ├── CMakeLists.txt
│       └── package.xml
├── build/                          # 建置輸出（自動產生）
├── install/                        # 安裝輸出（自動產生）
├── log/                            # 建置日誌（自動產生）
├── .clinerules                     # 開發規範
├── LICENSE                         # MIT 許可證
└── README.md                       # 本檔案
```

### 偵錯與日誌

查看節點日誌：

```bash
# 即時檢視日誌
ros2 run smartnav_vision face_recognition --ros-args --log-level debug

# 即時查看 ROS 2 日誌輸出
ros2 topic echo /rosout

# 查看 ROS 2 日誌目錄
ls ~/.ros/log
```

使用 `colcon` 偵錯：

```bash
# 編譯時產生偵錯符號
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

## 🛠️ 故障排除

### 問題：人臉識別性能低下

**解決方案**：

1. 檢查 GPU 是否正確啟用：`ros2 param list` 檢查 `enable_gpu` 參數
2. 調整 `confidence_threshold` 與 `recognition_threshold` 參數
3. 確保輸入影像品質良好（光照充足、清晰度高）

### 問題：識別錯誤率高

**解決方案**：

1. 增加人臉註冊時的樣本數量（`num_samples`）
2. 調整 `recognition_threshold` 閾值
3. 使用更高級的模型（如 `buffalo_l`）

### 問題：找不到依賴套件

**解決方案**：

```bash
rosdep install --from-paths src --ignore-src -r -y
pip install insightface opencv-python onnxruntime
```

## 📚 相關資源

- [ROS 2 官方文檔](https://docs.ros.org/en/jazzy/)
- [Nav2 文檔](https://navigation.ros.org/)
- [InsightFace GitHub](https://github.com/deepinsight/insightface)
- [OpenCV 文檔](https://docs.opencv.org/)

## 🤝 貢獻指南

歡迎貢獻代碼、報告問題與提出改進建議！

1. Fork 本倉庫
2. 建立功能分支：`git checkout -b feature/amazing-feature`
3. 提交更改：`git commit -m 'Add some amazing feature'`
4. 推送分支：`git push origin feature/amazing-feature`
5. 開啟 Pull Request
