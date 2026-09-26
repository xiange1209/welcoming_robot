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

# ★★ 2026-08-27 晚間新增：**用行程名精確比對**，不看指令列 ★★
#
# 給 `static_transform_publisher` 這種**通用名字**用的。
# 上面的 kill_pattern 走 `pgrep -f`（比對整條指令列），只排除了自己與父行程 ——
# 擋不住**第三方 shell**：別的終端機裡跑
#     ros2 node list | grep static_transform_publisher
# 那條管線的 cmdline 就含這個字串，會被一起 TERM 掉。
#
# `pgrep -x` 比對的是 comm 欄位（真正的執行檔名），shell 的 comm 是 `bash`，
# 所以絕對不會誤中。
#
# ★ 但 comm 欄位**上限 15 字元** —— `static_transform_publisher` 有 26 字元，
#   核心只留前 15 個字：`static_transfor`。用完整名字會比不中而得到 0 個。
#   （同一個坑 stop_nav_cc.sh 的 cmd_vel_floor_c 也踩過。）
kill_exact() {
    local comm="$1"
    local sig="$2"
    for pid in $(pgrep -x "$comm" 2>/dev/null); do
        [ "$pid" = "$SELF" ] && continue
        [ "$pid" = "$PARENT" ] && continue
        kill "$sig" "$pid" 2>/dev/null && echo "  已送 $sig 給 $pid ($comm)"
    done
}

echo "停止感測器..."
# ★ 2026-08-27 補 static_transform_publisher。
#   sensors_cc.launch.py 會起 5 個（base_to_laser/gyro/camera/link/radar），
#   而它們不在這個清單裡 -> 每重啟一次感測器就多 5 個。
#   8/27 實測累積到 **10 個**（存活 6011 秒的那 5 個是第一次啟動留下的）。
#   後果比 cmd_vel_floor 輕（同樣的靜態 TF 重複發布不會衝突），
#   但會白吃 CPU，而且 `ros2 node list | uniq -d` 會一直有雜訊，
#   蓋掉真正危險的重複節點。
for p in "sensors_cc.launch" "imu_bias_corrector_cc" "lslidar_driver_node" \
         "wheeltec_robot_node" "ekf_node" "robot_state_publisher"; do
    kill_pattern "$p" -TERM
done
# ★ 2026-08-27 晚間修正：static_transform_publisher 改走 kill_exact。
#   原本放在上面那個 pgrep -f 迴圈裡，會誤殺 cmdline 含這個字串的第三方 shell。
kill_exact "static_transfor" -TERM
sleep 5
for p in "imu_bias_corrector_cc" "lslidar_driver_node" "wheeltec_robot_node" "ekf_node"; do
    kill_pattern "$p" -KILL
done
# ★ 2026-08-27 晚間補：TERM 不一定收得掉，補一次 KILL。
kill_exact "static_transfor" -KILL
sleep 2

left=$(pgrep -f "lslidar_driver_node|wheeltec_robot_node" 2>/dev/null | wc -l)
echo "剩餘底盤/雷達行程: $left"
# ★ 2026-08-27 晚間補：靜態 TF 也要進存活檢查。
#   8/27 實測累積到 10 個而沒有任何一行提示 —— 沒印出來就等於沒在管。
stf=$(pgrep -x "static_transfor" 2>/dev/null | wc -l)
[ "$stf" != "0" ] && echo "⚠ static_transform_publisher 仍有 $stf 個存活（正常應為 0）"
exit 0
