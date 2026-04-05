#!/bin/bash
# 樹莓派 4 AI 銀行機器人 - 開發環境搭建腳本（分步驟安裝版本）
# 適用於：Raspberry Pi OS Trixie (Python 3.13+) + 32GB 記憶卡
# 用途：人臉識別 + 語音辨識 + HMI 開發

set -e

echo "========================================="
echo "樹莓派 4 開發環境搭建（磁盤優化版）"
echo "========================================="

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# 確認 Python 版本
echo -e "${YELLOW}[1/7] 檢查 Python 版本...${NC}"
PYTHON_VERSION=$(python3 --version | cut -d' ' -f2)
echo "Python 版本：$PYTHON_VERSION"

# 提取主版本號和次版本號進行數值比較
PYTHON_MAJOR=$(echo $PYTHON_VERSION | cut -d'.' -f1)
PYTHON_MINOR=$(echo $PYTHON_VERSION | cut -d'.' -f2)

# 檢查是否 >= 3.9
if [[ $PYTHON_MAJOR -lt 3 ]] || [[ $PYTHON_MAJOR -eq 3 && $PYTHON_MINOR -lt 9 ]]; then
    echo -e "${RED}❌ Python 版本過舊，需要 3.9+${NC}"
    exit 1
fi
echo -e "${GREEN}✓ Python 3.9+ 已安裝${NC}"

# 清理磁盤空間
echo -e "\n${YELLOW}[2/7] 清理磁盤空間...${NC}"
echo "  清理系統日誌..."
sudo journalctl --vacuum-size=50M 2>/dev/null || sudo journalctl --vacuum-time=1d 2>/dev/null || true
echo "  清理 apt 緩存..."
sudo apt-get clean
sudo apt-get autoclean -y
sudo apt-get autoremove -y
sudo rm -rf /var/cache/apt/* 2>/dev/null || true
echo "  清理臨時文件..."
sudo rm -rf /tmp/* 2>/dev/null || true
sudo rm -rf /var/tmp/* 2>/dev/null || true
echo "  清理 pip 緩存..."
rm -rf ~/.cache/pip/* 2>/dev/null || true
echo -e "${GREEN}✓ 磁盤空間已清理${NC}"

# 清理 apt 鎖定
echo -e "\n${YELLOW}[3/7] 清理 apt 鎖定...${NC}"
sudo rm -f /var/lib/apt/lists/lock
sudo rm -f /var/cache/apt/archives/lock
sudo rm -f /var/lib/dpkg/lock*
sudo rm -f /var/lib/dpkg/lock-frontend
echo -e "${GREEN}✓ 鎖定已清理${NC}"

# 更新系統包
echo -e "\n${YELLOW}[4/7] 更新系統包...${NC}"
sudo apt-get update -y 2>&1 | tail -1
echo -e "${GREEN}✓ 系統包已更新${NC}"

# 安裝最少的系統依賴
echo -e "\n${YELLOW}[5/7] 安裝基礎工具...${NC}"
sudo apt-get install -y \
    build-essential \
    python3-dev \
    python3-pip \
    python3-venv \
    git
echo -e "${GREEN}✓ 基礎工具已安裝${NC}"

# 建立虛擬環境
echo -e "\n${YELLOW}[6/7] 建立虛擬環境...${NC}"
if [ ! -d "$HOME/ai_bank_robot_env" ]; then
    python3 -m venv $HOME/ai_bank_robot_env
    echo "虛擬環境已建立"
else
    echo "虛擬環境已存在"
fi

# 啟動虛擬環境
source $HOME/ai_bank_robot_env/bin/activate
echo -e "${GREEN}✓ 虛擬環境已啟動${NC}"

# 升級 pip
echo -e "\n${YELLOW}[7/7] 安裝 Python 套件（分步驟）...${NC}"
pip install --upgrade pip --no-cache-dir -q

# 分步驟安裝各個套件
echo ""
echo "  [1/8] 安裝 numpy..."
pip install --no-cache-dir numpy -q
echo "  ✓ numpy 安裝完成"

echo "  [2/8] 安裝 pillow..."
pip install --no-cache-dir pillow -q
echo "  ✓ pillow 安裝完成"

echo "  [3/8] 安裝 OpenCV..."
pip install --no-cache-dir opencv-python-headless -q
echo "  ✓ OpenCV 安裝完成"

echo "  [4/8] 安裝 ONNX Runtime..."
pip install --no-cache-dir onnxruntime -q
echo "  ✓ ONNX Runtime 安裝完成"

echo "  [5/8] 安裝 pyttsx3..."
pip install --no-cache-dir pyttsx3 -q
echo "  ✓ pyttsx3 安裝完成"

echo "  [6/8] 安裝 sounddevice..."
pip install --no-cache-dir sounddevice -q
echo "  ✓ sounddevice 安裝完成"

echo "  [7/8] 安裝後端工具及配置依賴..."
pip install --no-cache-dir fastapi uvicorn pydantic paho-mqtt requests flask pyyaml psutil -q
echo "  ✓ 後端工具安裝完成"

echo "  [8/8] 安裝 Whisper（可選，如果失敗會跳過）..."
if pip install --no-cache-dir openai-whisper -q 2>/dev/null; then
    echo "  ✓ Whisper 安裝完成"
else
    echo "  ⚠ Whisper 安裝失敗（磁盤空間限制），可稍後手動安裝："
    echo "    pip install openai-whisper"
fi

echo -e "${GREEN}✓ 所有 Python 套件已安裝${NC}"

# 驗證安裝
echo -e "\n${YELLOW}驗證環境...${NC}"
python3 << 'PYEOF'
import sys
checks = []

try:
    import cv2
    checks.append(f"✓ OpenCV {cv2.__version__}")
except Exception as e:
    checks.append(f"✗ OpenCV")

try:
    import onnxruntime
    checks.append(f"✓ ONNX Runtime")
except Exception as e:
    checks.append(f"✗ ONNX")

try:
    import whisper
    checks.append(f"✓ Whisper")
except Exception as e:
    checks.append(f"✗ Whisper")

try:
    import pyttsx3
    checks.append(f"✓ pyttsx3")
except Exception as e:
    checks.append(f"✗ pyttsx3")

try:
    import fastapi
    checks.append(f"✓ FastAPI")
except Exception as e:
    checks.append(f"✗ FastAPI")

for check in checks:
    print(check)
PYEOF

echo ""
echo -e "${GREEN}=========================================${NC}"
echo -e "${GREEN}✓ 開發環境搭建完成！${NC}"
echo -e "${GREEN}=========================================${NC}"

echo -e "\n${YELLOW}下一步：${NC}"
echo ""
echo "1️⃣  啟動虛擬環境："
echo "   source ~/ai_bank_robot_env/bin/activate"
echo ""
echo "2️⃣  驗證環境："
echo "   pip list | grep -E 'opencv|whisper|fastapi'"
echo ""
echo "3️⃣  測試 Whisper："
echo "   python3 -m whisper --version"
echo ""
echo "4️⃣  開始開發："
echo "   cd ~/ai_bank_robot"
echo ""
echo -e "${GREEN}祝開發順利！🚀${NC}"


