#!/bin/bash
# 智慧銀行 AI 環境安裝腳本
# 適用：Ubuntu 24.04 Server (aarch64) + Raspberry Pi 4 + Camera Rev 1.3 (OV5647)
# 使用方式：bash setup_ubuntu.sh
#
# 安裝內容：
#   - 修復 apt noble-updates（解決 -dev 套件版本衝突）
#   - 系統編譯依賴
#   - libcamera 0.3.0（從原始碼建置，修復 OV5647 + kernel 6.8 FATAL bug）
#   - Python venv + kms stub（修復 picamera2 pip 版）
#   - InsightFace / OpenCV / ONNXRuntime 等 AI 套件

set -uo pipefail

# ── 顏色 ──────────────────────────────────────────────
GREEN='\033[0;32m'; YELLOW='\033[1;33m'
RED='\033[0;31m';   BLUE='\033[0;34m'; NC='\033[0m'

FAILED=()
VENV="$HOME/ai_bank_robot_env"
LIBCAM_SRC="$HOME/libcamera/libcamera"
LIBCAM_VER="v0.3.0"

step()  { echo -e "\n${BLUE}[$1]${NC} $2"; }
ok()    { echo -e "  ${GREEN}✓${NC} $1"; }
warn()  { echo -e "  ${YELLOW}⚠${NC} $1"; }
fail()  { echo -e "  ${RED}✗${NC} $1"; FAILED+=("$1"); }

# ── 標題 ──────────────────────────────────────────────
echo "╔══════════════════════════════════════════════╗"
echo "║  智慧銀行 AI 環境安裝                        ║"
echo "║  Ubuntu 24.04 + RPi4 + Camera OV5647        ║"
echo "╚══════════════════════════════════════════════╝"

# ── Step 0：系統檢查 ──────────────────────────────────
step "0/7" "系統檢查"
ARCH=$(uname -m)
UBUNTU_VER=$(lsb_release -rs 2>/dev/null || echo "unknown")
ok "Ubuntu $UBUNTU_VER ($ARCH)"
[[ "$ARCH" != "aarch64" ]] && warn "非 aarch64，libcamera RPi 流程可能不適用"
[[ "$EUID" -eq 0 ]] && { echo "請勿用 root 執行本腳本"; exit 1; }

# ── Step 1：修復 apt sources ──────────────────────────
step "1/7" "修復 apt sources（加入 noble-updates）"
SOURCES="/etc/apt/sources.list.d/ubuntu.sources"
if grep -q "noble-updates" "$SOURCES" 2>/dev/null; then
    ok "noble-updates 已存在，跳過"
else
    sudo tee -a "$SOURCES" > /dev/null << 'EOF'

Types: deb
URIs: http://ports.ubuntu.com/ubuntu-ports/
Suites: noble-updates
Components: main restricted universe multiverse
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg
EOF
    ok "noble-updates 已加入"
    sudo apt update -q && ok "apt update 完成" || fail "apt update"
fi

# ── Step 2：系統編譯依賴 ──────────────────────────────
step "2/7" "安裝系統編譯依賴"
PKGS=(
    python3-pip git cmake
    meson ninja-build pkg-config
    libgnutls28-dev openssl
    libyaml-dev python3-yaml python3-ply python3-jinja2
    libdw-dev libunwind-dev libudev-dev libdrm-dev
    pybind11-dev python3-pybind11 libpython3-dev
    libcap-dev g++ python3.12-dev
)
if sudo apt install -y "${PKGS[@]}" -q; then
    ok "系統依賴安裝完成"
else
    warn "批次安裝失敗，逐一嘗試..."
    for pkg in "${PKGS[@]}"; do
        sudo apt install -y "$pkg" -q && ok "$pkg" || fail "$pkg"
    done
fi

# ── Step 3：建置並安裝 libcamera 0.3.0 ───────────────
step "3/7" "libcamera 0.3.0（修復 OV5647 + kernel 6.8 FATAL bug）"

if ldconfig -p 2>/dev/null | grep -q "libcamera.so.0.3"; then
    ok "libcamera 0.3.0 已安裝，跳過（如需重建請刪除 $LIBCAM_SRC/build）"
else
    echo "  預計時間：30~60 分鐘（RPi4 -j2 模式）"

    mkdir -p "$(dirname "$LIBCAM_SRC")"

    # Clone（若尚未 clone）
    if [ ! -d "$LIBCAM_SRC/.git" ]; then
        echo "  cloning libcamera..."
        git clone https://git.libcamera.org/libcamera/libcamera.git "$LIBCAM_SRC" \
            && ok "clone 完成" || { fail "git clone libcamera"; }
    else
        ok "libcamera repo 已存在"
    fi

    (
        cd "$LIBCAM_SRC"
        git checkout "$LIBCAM_VER" 2>/dev/null && ok "checkout $LIBCAM_VER" \
            || warn "checkout $LIBCAM_VER 失敗，使用目前分支"

        # meson configure（若 build/ 不存在）
        if [ ! -f "build/build.ninja" ]; then
            meson setup build \
                -Dpipelines=rpi/vc4 \
                -Dipas=rpi/vc4 \
                -Dpycamera=enabled \
                -Dlc-compliance=disabled \
                -Dcam=disabled \
                -Dtest=false \
                --buildtype=release \
                && ok "meson 配置完成" || { fail "meson setup"; exit 1; }
        else
            ok "build/ 已存在，跳過 meson setup"
        fi

        echo "  編譯中（-j2 避免 OOM 當機）..."
        ninja -C build -j2 \
            && ok "編譯完成" || { fail "ninja 編譯失敗"; exit 1; }

        sudo ninja -C build install \
            && ok "安裝完成" || { fail "ninja install"; exit 1; }
    ) || FAILED+=("libcamera-0.3.0")

    sudo ldconfig && ok "ldconfig 更新完成"
fi

# ── Step 4：建立 Python venv ──────────────────────────
step "4/7" "建立 Python 虛擬環境"
if [ -d "$VENV" ]; then
    ok "venv 已存在：$VENV"
else
    python3 -m venv --system-site-packages "$VENV" \
        && ok "venv 建立完成：$VENV" || fail "venv 建立失敗"
fi
# shellcheck disable=SC1090
source "$VENV/bin/activate"

# ── Step 5：修復 libcamera Python 路徑 ───────────────
step "5/7" "設定 libcamera Python 路徑 + kms stub"
SITE="$VENV/lib/python3.12/site-packages"

# libcamera 0.3.0 路徑（優先）
echo '/usr/local/lib/python3.12/site-packages' > "$SITE/libcamera030.pth" \
    && ok "libcamera 0.3.0 Python 路徑設定完成"

# dpkg libcamera 路徑（system python 備用）
SYS_PTH="/usr/lib/python3/dist-packages/aarch64_libcamera.pth"
if [ ! -f "$SYS_PTH" ]; then
    sudo bash -c "echo '/usr/lib/aarch64-linux-gnu/python3.12/site-packages' > $SYS_PTH" \
        && ok "system Python libcamera 路徑設定完成"
else
    ok "system Python libcamera 路徑已存在"
fi

# kms stub（解決 picamera2 pip 版 ModuleNotFoundError: No module named 'kms'）
KMS_DIR="$SITE/kms"
if [ ! -d "$KMS_DIR" ]; then
    mkdir -p "$KMS_DIR"
    cat > "$KMS_DIR/__init__.py" << 'EOF'
class _Stub:
    def __getattr__(self, name):
        return name
PixelFormats = _Stub()
class Card:
    pass
class ResourceManager:
    def __init__(self, *a): pass
EOF
    mkdir -p "$SITE/pykms"
    touch "$SITE/pykms/__init__.py"
    ok "kms/pykms stub 建立完成"
else
    ok "kms stub 已存在"
fi

# ── Step 6：安裝 Python 套件 ──────────────────────────
step "6/7" "安裝 Python 套件"
pip install --upgrade pip -q

install_pkg() {
    echo -n "  安裝 $1 ... "
    # shellcheck disable=SC2086
    pip install $1 -q && echo -e "${GREEN}✓${NC}" || { echo -e "${RED}✗${NC}"; FAILED+=("pip:$1"); }
}

install_pkg "numpy"
install_pkg "pillow"
install_pkg "opencv-python"
install_pkg "onnxruntime"
install_pkg "insightface"
install_pkg "psutil"
install_pkg "PyYAML"
install_pkg "pydantic"
install_pkg "python-dotenv"
install_pkg "requests"
install_pkg "fastapi"
install_pkg "uvicorn"
install_pkg "picamera2 --no-deps"

# ── Step 7：初始化目錄 + 驗證 ─────────────────────────
step "7/7" "初始化目錄 + 驗證安裝"
mkdir -p ~/ai_bank_robot/database ~/ai_bank_robot/insightface_models
ok "目錄初始化完成"

python3 - << 'PYEOF'
import sys
sys.path.insert(0, '/usr/local/lib/python3.12/site-packages')

results = []
checks = [
    ("cv2",           "OpenCV"),
    ("numpy",         "NumPy"),
    ("onnxruntime",   "ONNXRuntime"),
    ("insightface",   "InsightFace"),
    ("psutil",        "psutil"),
    ("yaml",          "PyYAML"),
    ("fastapi",       "FastAPI"),
]
for mod, name in checks:
    try:
        __import__(mod)
        print(f"  \033[32m✓\033[0m {name}")
        results.append(True)
    except ImportError:
        print(f"  \033[31m✗\033[0m {name} — 未安裝")
        results.append(False)

try:
    import libcamera
    print(f"  \033[32m✓\033[0m libcamera 0.3.0")
    results.append(True)
except ImportError as e:
    print(f"  \033[31m✗\033[0m libcamera — {e}")
    results.append(False)

try:
    from picamera2 import Picamera2
    print(f"  \033[32m✓\033[0m picamera2")
    results.append(True)
except ImportError as e:
    print(f"  \033[31m✗\033[0m picamera2 — {e}")
    results.append(False)

print()
if all(results):
    print("\033[32m  ✅ 所有套件驗證通過\033[0m")
else:
    print("\033[31m  ❌ 有套件未安裝，請檢查上方錯誤\033[0m")
    sys.exit(1)
PYEOF

# ── 結果摘要 ──────────────────────────────────────────
echo ""
echo "╔══════════════════════════════════════════════╗"
if [ ${#FAILED[@]} -eq 0 ]; then
    echo -e "║  ${GREEN}✅ 安裝完成，無錯誤${NC}"
else
    echo -e "║  ${YELLOW}⚠  以下項目安裝失敗：${NC}"
    for f in "${FAILED[@]}"; do echo "║    - $f"; done
fi
echo "║"
echo "║  啟動方式："
echo "║    source ~/ai_bank_robot_env/bin/activate"
echo "║    export DISPLAY=:1"
echo "║    cd ~/ai_bank_robot"
echo "║    python3 src/scripts/main.py"
echo "╚══════════════════════════════════════════════╝"
