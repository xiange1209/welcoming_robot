#!/bin/bash
# 假機器人 (無底座測試用) —— 取代 run_sensors_cc.sh
source /home/user/maprun/env.sh
CYCLONEDDS_FILE="${CYCLONEDDS_URI#file://}"
if [ -n "$CYCLONEDDS_FILE" ] && [ ! -f "$CYCLONEDDS_FILE" ]; then
  echo "[run_fake_robot_cc] 警告：找不到 $CYCLONEDDS_FILE，改用預設設定" >&2
  unset CYCLONEDDS_URI
fi
exec ros2 launch smartnav_navigation_cc fake_robot_cc.launch.py "$@" \
  > "$LOGDIR/fake_robot_cc.log" 2>&1
