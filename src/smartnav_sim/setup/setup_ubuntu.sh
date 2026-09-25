#!/usr/bin/env bash
# 在 WSL 的 Ubuntu 24.04 裡建立「自動建圖模擬」環境：ROS 2 Jazzy + Gazebo Harmonic + nav2。
#
# 冪等：重跑不會壞，已經做過的步驟會跳過。
# 預估：下載約 0.7 GB、裝完新增約 3.3 GB（在 D 槽的 VHDX 裡，不佔 C 槽），30~60 分鐘。
#
# 用法（在 WSL 的 Ubuntu 終端機裡）：
#   bash "/mnt/c/Users/<Windows 使用者名稱>/Desktop/專題/車子/src/smartnav_sim/setup/setup_ubuntu.sh"
#   （setup_wsl.ps1 跑完最後會印出這台電腦的完整指令，直接複製即可）
#
# 做的事：
#   1. locale UTF-8、universe
#   2. ROS 2 的 apt 來源（官方 ros2-apt-source 套件，2025 年起的做法）
#   3. 裝 ROS 2 Jazzy base + Gazebo Harmonic（ros-gz）+ nav2 + slam_toolbox 等（不裝 recommends 省空間）
#   4. rosdep
#   5. ~/sim_ws/src 用 symlink 指向 Windows 那份 repo（在 Windows 改檔立刻生效）
#   6. colcon build
#   7. ~/.bashrc 加環境變數（不重複加）

set -euo pipefail

# 本檔在 <repo>/src/smartnav_sim/setup/，往上三層就是 repo。
# 不寫死路徑：Windows 路徑裡有使用者名稱，這個檔案會進 GitHub
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
WS="$HOME/sim_ws"
# 模擬只需要這幾個套件；其他（人臉、語音、LLM、HMI）跟建圖無關，不 build
PKGS=(smartnav_msgs smartnav_navigation_cc frontier_exploration_ros2 smartnav_sim)

say()  { printf '\n\033[1;36m▶ %s\033[0m\n' "$*"; }
ok()   { printf '  \033[32m✓\033[0m %s\n' "$*"; }
warn() { printf '  \033[33m⚠\033[0m %s\n' "$*"; }
die()  { printf '  \033[31m✗\033[0m %s\n' "$*"; exit 1; }

# ── 0. 前置檢查 ─────────────────────────────────────────────
say "0/7 前置檢查"
. /etc/os-release
[ "${VERSION_CODENAME:-}" = "noble" ] || die "這支要在 Ubuntu 24.04（noble）跑，現在是 ${PRETTY_NAME:-未知}"
ok "Ubuntu ${VERSION_ID}（${VERSION_CODENAME}）"
[ -d "$REPO/src" ] || die "找不到 $REPO/src —— Windows 那份 repo 的路徑不對？（WSL 裡 C 槽在 /mnt/c）"
ok "看得到 Windows 的 repo：$REPO"
# frontier 是 git 子模組；Windows 端沒 init 的話這裡是空的
[ -f "$REPO/src/frontier_exploration_ros2/package.xml" ] \
  || die "frontier 子模組是空的。先在 Windows 的 車子/ 目錄跑：git submodule update --init src/frontier_exploration_ros2"
ok "frontier 子模組在"

# ── 1. locale 與 universe ───────────────────────────────────
say "1/7 locale 與 universe"
if ! locale | grep -q "UTF-8"; then
  sudo apt-get update
  sudo apt-get install -y locales
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8
fi
ok "locale：$(locale | grep '^LANG=' || echo LANG 未設)"
sudo apt-get install -y software-properties-common curl
if ! grep -rqs "^deb.* universe" /etc/apt/sources.list /etc/apt/sources.list.d/ \
   && ! grep -rqs "Components:.*universe" /etc/apt/sources.list.d/; then
  sudo add-apt-repository -y universe
fi
ok "universe 已啟用"

# ── 2. ROS 2 apt 來源（官方 ros2-apt-source）────────────────
# 取自 ros2/ros2_documentation jazzy 分支 source/Installation/_Apt-Repositories.rst
say "2/7 ROS 2 apt 來源"
if dpkg -s ros2-apt-source >/dev/null 2>&1; then
  ok "ros2-apt-source 已安裝，跳過"
else
  ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
                           | grep -F "tag_name" | awk -F'"' '{print $4}')
  [ -n "$ROS_APT_SOURCE_VERSION" ] || die "查不到 ros-apt-source 的最新版本（GitHub API 限流或沒網路？）"
  curl -L -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
  rm -f /tmp/ros2-apt-source.deb
  ok "ros2-apt-source ${ROS_APT_SOURCE_VERSION}"
fi
sudo apt-get update

# ── 3. 套件 ─────────────────────────────────────────────────
# --no-install-recommends：只少裝約 0.6 GB，但不影響功能。
# 用 ros-base 而不是 desktop：desktop 多出來的主要是 PCL、VTK、OpenJDK，這裡用不到。
say "3/7 安裝 ROS 2 Jazzy + Gazebo Harmonic + nav2（最久的一步）"
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
  ros-jazzy-ros-base \
  ros-jazzy-ros-gz \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox ros-jazzy-robot-localization \
  ros-jazzy-rviz2 ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-rmw-cyclonedds-cpp \
  ros-dev-tools build-essential
ok "套件安裝完成"

# ── 4. rosdep ───────────────────────────────────────────────
say "4/7 rosdep"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update --rosdistro jazzy
ok "rosdep 已更新"

# ── 5. 工作區：symlink 到 Windows 那份 repo ─────────────────
# 用 symlink 而不是複製：在 Windows 用 VS Code 改檔，WSL 裡立刻看得到。
# 代價：/mnt/c 的檔案 I/O 比 WSL 自己的 ext4 慢，colcon build 會慢一些。
say "5/7 工作區 $WS"
mkdir -p "$WS/src"
LINKED=()   # 實際連結到的套件；colcon 指定不存在的套件會直接報錯
for p in "${PKGS[@]}"; do
  src="$REPO/src/$p"
  dst="$WS/src/$p"
  # 看 package.xml 不看目錄：smartnav_sim/ 目前只有 setup/，目錄在但還不是套件，
  # 連進 --packages-up-to 會讓 colcon 直接報「找不到套件」
  if [ ! -f "$src/package.xml" ]; then
    warn "$p 還不是 ROS 套件（沒有 package.xml），跳過（smartnav_sim 還沒寫好的話是正常的）"
    continue
  fi
  if [ -L "$dst" ]; then
    LINKED+=("$p")
    ok "$p 已連結"
  elif [ -e "$dst" ]; then
    warn "$dst 已存在而且不是 symlink —— 不動它，請自己確認"
  else
    ln -s "$src" "$dst"
    LINKED+=("$p")
    ok "$p → $src"
  fi
done

[ "${#LINKED[@]}" -gt 0 ] || die "一個套件都沒連結到，檢查 $REPO/src"

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# rosdep 補其他依賴。解析不到的鍵只警告，不中止（例如 ament_* 測試依賴）
rosdep install --from-paths "$WS/src" --ignore-src --rosdistro jazzy -y \
  --skip-keys "frontier_exploration_ros2_rviz" || warn "rosdep 有部分依賴沒解析到，看上面的訊息；通常不影響 build"

# ── 6. build ────────────────────────────────────────────────
# frontier 的 RViz 面板要 Qt 開發檔，探索本身用不到，跳過
say "6/7 colcon build"
cd "$WS"
colcon build --symlink-install \
  --packages-up-to "${LINKED[@]}" \
  --packages-skip frontier_exploration_ros2_rviz \
  --event-handlers console_cohesion+ \
  || die "build 失敗，看上面的錯誤。常見原因：rosdep 漏裝依賴、Windows 端檔案有 CRLF"
ok "build 完成"

# ── 7. ~/.bashrc ────────────────────────────────────────────
say "7/7 環境變數"
add_line() { grep -qxF "$1" "$HOME/.bashrc" || echo "$1" >> "$HOME/.bashrc"; }
add_line "# ── smartnav 模擬（setup_ubuntu.sh 加的）──"
add_line "source /opt/ros/jazzy/setup.bash"
add_line "[ -f $WS/install/setup.bash ] && source $WS/install/setup.bash"
# 不要跟車上的 Pi 互相發現（Pi 用同一個 CycloneDDS），只在本機通訊
add_line "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST"
add_line "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
# 讓 WSLg 的 Mesa 用 NVIDIA 顯卡算 Gazebo 畫面；沒生效時會退回 llvmpipe（CPU 軟算，慢但能跑）
add_line "export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA"
ok "~/.bashrc 已更新（開新終端機或 source ~/.bashrc 生效）"

sudo apt-get clean
say "完成"
echo "  ROS 2 + Gazebo 佔用：$(du -sh /opt/ros/jazzy 2>/dev/null | cut -f1)（/opt/ros/jazzy）"
echo "  整個 Ubuntu 根目錄：$(sudo du -sxh / 2>/dev/null | cut -f1)"
cat <<'EOF'

下一步（開新終端機）：
  1. 冒煙測試 Gazebo：   gz sim -v4 -r shapes.sdf
     畫面起不來 → gz sim -v4 -r shapes.sdf --render-engine ogre   （WSL 上 ogre2 可能崩潰）
     完全沒畫面 → gz sim -v4 -s -r --headless-rendering shapes.sdf（不開視窗，只跑模擬）
  2. 看顯卡有沒有用上：  sudo apt install -y mesa-utils && glxinfo -B | grep -i renderer   （應該看到 D3D12 (NVIDIA...)；llvmpipe 代表退回 CPU）
  3. 跑自動建圖模擬：    ros2 launch smartnav_sim sim_explore.launch.py
EOF
