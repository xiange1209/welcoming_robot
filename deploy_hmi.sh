#!/usr/bin/env bash
# 把 HMI 的兩個改動檔傳到 Pi，並在傳送前先做備份。
#
# 用法（在 Windows 的 Git Bash 執行）：
#     ./deploy_hmi.sh 192.168.1.100
#     ./deploy_hmi.sh 192.168.1.100 user
#
# 只傳兩個檔案，不碰 smartnav_navigation_cc、不碰任何 launch/config。

set -eo pipefail

PI_IP="${1:?請給 Pi 的 IP，例如 ./deploy_hmi.sh 192.168.1.100}"
PI_USER="${2:-user}"
WS="~/welcoming_robot_ws/src/smartnav_ws/src/smartnav_hmi"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="$HERE/welcoming_robot_ws/src/smartnav_ws/src/smartnav_hmi"

echo "=== 目標 ${PI_USER}@${PI_IP} ==="

# 1. 先備份 —— 出事要能退回去
echo "--- 備份現有版本 ---"
ssh "${PI_USER}@${PI_IP}" "
  set -e
  cd ${WS}
  STAMP=\$(date +%Y%m%d_%H%M%S)
  mkdir -p ~/hmi_backup/\$STAMP
  cp smartnav_hmi/hmi_server_node.py ~/hmi_backup/\$STAMP/
  cp web/index.html ~/hmi_backup/\$STAMP/
  echo \"已備份到 ~/hmi_backup/\$STAMP\"
"

# 2. 傳檔
echo "--- 傳送 ---"
scp "$SRC/smartnav_hmi/hmi_server_node.py" "${PI_USER}@${PI_IP}:${WS}/smartnav_hmi/hmi_server_node.py"
scp "$SRC/web/index.html"                  "${PI_USER}@${PI_IP}:${WS}/web/index.html"

# 3. 建置並驗證
echo "--- 建置 ---"
ssh "${PI_USER}@${PI_IP}" "
  set -e
  source /opt/ros/jazzy/setup.bash
  cd ~/welcoming_robot_ws
  colcon build --packages-select smartnav_hmi
  source install/setup.bash
  echo ''
  echo '=== 驗證新版是否真的進到 install ==='
  N1=\$(grep -c 'Authorization' \$(ros2 pkg prefix smartnav_hmi)/share/smartnav_hmi/web/index.html || true)
  N2=\$(grep -c '_publish_teleop' \$(ros2 pkg prefix smartnav_hmi)/lib/python3*/site-packages/smartnav_hmi/hmi_server_node.py || true)
  echo \"  index.html 的 Authorization: \$N1 處（應 >= 3）\"
  echo \"  hmi_server 的 _publish_teleop: \$N2 處（應 = 5）\"
  if [ \"\$N1\" -ge 3 ] && [ \"\$N2\" -ge 5 ]; then echo '  結果: 新版已生效'; else echo '  結果: 失敗，install 底下還是舊的'; exit 1; fi
"

echo
echo "════════════════════════════════════════════"
echo " 部署完成。接下來："
echo "   1. 重啟 hmi_server 節點（其他節點不用動）"
echo
echo " 退回舊版： ls ~/hmi_backup/ 找最新那個時間戳"
echo "════════════════════════════════════════════"
