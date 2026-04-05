#!/bin/bash

# ============================================================
# 智慧銀行AI服務機器人 - 快速環境部署腳本 (Raspberry Pi 4)
# 用途：在新的樹莫派上快速重建開發環境
# 使用：bash setup_rpi4_quick.sh
# ============================================================

set -e

echo "=========================================================="
echo "🚀 智慧銀行AI樓服機器人 - 快速環境部署"
echo "=========================================================="
echo "此腳本將在樹莫派 4 上快速部署開發環境"
echo "預計時間：15-20 分鐘"
echo ""

# ===== Step 1: 檢查 Python 版本 =====
echo "[1/7] 檢查 Python 版本..."
PYTHON_VERSION=$(python3 --version | cut -d' ' -f2)
echo "✓ Python $PYTHON_VERSION"

# ===== Step 2: 系統更新 =====
echo "[2/7] 更新系統套件列表..."
sudo apt-get update -qq

# ===== Step 3: 安裝系統依賴 =====
echo "[3/7] 安裝 GTK+ 和編譯工具..."
sudo apt-get install -y -qq \
    libgtk2.0-dev \
    pkg-config \
    libatlas-base-dev \
    libjasper-dev \
    libtiff5 \
    libharfbuzz0b \
    libwebp6 \
    libqtgui4 \
    libqt4-test \
    libhdf5-dev \
    libcap-dev

echo "✓ 系統依賴已安裝"

# ===== Step 4: 創建虛擬環境 =====
echo "[4/7] 創建虛擬環境..."
if [ ! -d "$HOME/ai_bank_robot_env" ]; then
    python3 -m venv --system-site-packages ~/ai_bank_robot_env
    echo "✓ 虛擬環境已創建"
else
    echo "⚠ 虛擬環境已存在，跳過創建"
fi

# ===== Step 5: 激活虛擬環境 =====
echo "[5/7] 激活虛擬環境..."
source ~/ai_bank_robot_env/bin/activate

# ===== Step 6: 清理磁盤 =====
echo "[6/7] 清理磁盤空間..."
rm -rf ~/.cache/pip/* 2>/dev/null || true
sudo apt-get clean -y -qq
sudo journalctl --vacuum-size=50M 2>/dev/null || true
df -h | grep -E "^/dev|Filesystem"

# ===== Step 7: 安裝 Python 依賴 =====
echo "[7/7] 安裝 Python 依賴 (this may take 5-10 minutes)..."
pip install --upgrade pip -q
pip install --no-cache-dir -q -r requirements_minimal.txt

# ===== 驗證安裝 =====
echo ""
echo "=========================================================="
echo "✅ 環境部署完成！"
echo "=========================================================="
echo ""
echo "後續步驟："
echo ""
echo "1️⃣  激活虛擬環境："
echo "   source ~/ai_bank_robot_env/bin/activate"
echo ""
echo "2️⃣  進入項目目錄："
echo "   cd ~/ai_bank_robot"
echo ""
echo "3️⃣  設置顯示："
echo "   export DISPLAY=:1"
echo ""
echo "4️⃣  運行人臉檢測："
echo "   python3 scripts/realtime_detection_insightface.py"
echo ""
echo "5️⃣  驗證安裝："
python3 -c "
import cv2
import insightface
import onnxruntime
from picamera2 import Picamera2
print('✓ OpenCV', cv2.__version__)
print('✓ InsightFace 0.7.3')
print('✓ ONNX Runtime installed')
print('✓ Picamera2 installed')
print('')
print('🎉 所有依賴已安裝！')
"

echo ""
echo "=========================================================="
echo "磁盤使用情況："
df -h | grep -E "^/dev|Filesystem"
echo "=========================================================="
