#!/bin/bash
# 底盤 + 雷達 + IMU 零偏補償 (相機預設不啟動，省 110% CPU)
#
# ★ 啟動後前幾秒會校準 IMU 零偏，車子必須完全靜止 ★
#   校準期間移動會把真實旋轉當成零偏記下來，之後每次轉彎都會被錯誤補償。
#   要重新校準： ros2 service call /recalibrate_imu_bias std_srvs/srv/Trigger
#
# 用法：
#   ./run_sensors_cc.sh              # 不含相機
#   ./run_sensors_cc.sh true         # 含相機 (人臉辨識用)
source /home/user/maprun/env.sh
CYCLONEDDS_FILE="${CYCLONEDDS_URI#file://}"
if [ -n "$CYCLONEDDS_FILE" ] && [ ! -f "$CYCLONEDDS_FILE" ]; then
  echo "[run_sensors_cc] 警告：找不到 $CYCLONEDDS_FILE，改用 CycloneDDS 預設設定" >&2
  unset CYCLONEDDS_URI
fi
WITH_CAMERA="${1:-false}"
exec ros2 launch smartnav_navigation_cc sensors_cc.launch.py \
  with_camera:="$WITH_CAMERA" > "$LOGDIR/sensors_cc.log" 2>&1
