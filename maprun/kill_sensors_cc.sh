#!/bin/bash
# 安全地停掉底盤/雷達/EKF/IMU 補償。
#
# 和 kill_camera_cc.sh 同樣的理由寫成獨立腳本：pgrep -f 的 pattern 若出現在
# 呼叫端的指令列裡，會把自己的 shell 一起殺掉（exit 144，已經發生過三次）。

SELF=$$
PARENT=$PPID

kill_pattern() {
    local pat="$1"
    local sig="$2"
    for pid in $(pgrep -f "$pat" 2>/dev/null); do
        [ "$pid" = "$SELF" ] && continue
        [ "$pid" = "$PARENT" ] && continue
        kill "$sig" "$pid" 2>/dev/null && echo "  已送 $sig 給 $pid"
    done
}

echo "停止感測器..."
for p in "sensors_cc.launch" "imu_bias_corrector_cc" "lslidar_driver_node" \
         "wheeltec_robot_node" "ekf_node" "robot_state_publisher"; do
    kill_pattern "$p" -TERM
done
sleep 5
for p in "imu_bias_corrector_cc" "lslidar_driver_node" "wheeltec_robot_node" "ekf_node"; do
    kill_pattern "$p" -KILL
done
sleep 2

left=$(pgrep -f "lslidar_driver_node|wheeltec_robot_node" 2>/dev/null | wc -l)
echo "剩餘底盤/雷達行程: $left"
exit 0
