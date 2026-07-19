#!/bin/bash
# 在 ros:jazzy arm64 容器內執行：裝依賴 -> colcon build -> 打包 install/
# 用法見 docs/部署與重現指南.md 路線 D。
# 產出 /ws/smartnav_arm64_install.tar.gz，RPi4 解壓即可執行（不需編譯）。
set -e
set -o pipefail
echo "=== [1/4] apt 依賴 ==="
apt-get update -qq
DEBIAN_FRONTEND=noninteractive apt-get install -y -qq \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-cartographer-ros ros-jazzy-robot-localization ros-jazzy-topic-tools \
  ros-jazzy-usb-cam ros-jazzy-joint-state-publisher \
  libuvc-dev libgoogle-glog-dev python3-soundfile >/dev/null
echo "=== [2/4] rosdep ==="
# 網路波動時 update 失敗不致命（快取仍可用），不讓 set -e 把整個編譯殺掉
rosdep update --rosdistro jazzy 2>/dev/null | tail -1 || echo "rosdep update 失敗，改用既有快取繼續"
cd /ws
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy 2>&1 | tail -3
echo "=== [3/4] colcon build（arm64, 不用 symlink-install 以便搬移）==="
source /opt/ros/jazzy/setup.bash
export MAKEFLAGS="-j4"
colcon build --parallel-workers 4 2>&1 | tail -20
echo "=== [4/4] 打包 ==="
# 原子替換：先打包到 .partial，成功才覆蓋正式檔——重編/中斷過程中舊產物永遠可用
tar czf /ws/smartnav_arm64_install.tar.gz.partial install
mv -f /ws/smartnav_arm64_install.tar.gz.partial /ws/smartnav_arm64_install.tar.gz
ls -lh /ws/smartnav_arm64_install.tar.gz
# 清容器內 apt 垃圾，抑制容器可寫層膨脹（實測曾累積到 4.3GB 吃光 VM 磁碟）
apt-get clean && rm -rf /var/lib/apt/lists/*
echo "ARM64_BUILD_DONE"
