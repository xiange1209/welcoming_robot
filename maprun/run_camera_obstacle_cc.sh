#!/bin/bash
# 深度相機避障模式。
#
# 驅動只出深度圖，點雲由 depth_obstacle_cc 降頻降採樣後自己生 ——
# 因為驅動的點雲生成在 Pi4 上要吃 34 個百分點的 CPU（61.8% -> 28.2%），
# 而且幀率降不下來（Astra 驅動收到不支援的 fps 會靜默回退成 30）。
#
# 用法： run_camera_obstacle_cc.sh [with_color] [depth_width] [depth_height] [cloud_hz] [pixel_step]
# 例：   run_camera_obstacle_cc.sh false 160 120 5.0 2

set +u
source /home/user/maprun/env.sh
CYCLONEDDS_FILE="${CYCLONEDDS_URI#file://}"
if [ -n "$CYCLONEDDS_FILE" ] && [ ! -f "$CYCLONEDDS_FILE" ]; then unset CYCLONEDDS_URI; fi

WITH_COLOR="${1:-false}"
DW="${2:-320}"
DH="${3:-240}"
HZ="${4:-5.0}"
STEP="${5:-4}"

exec ros2 launch smartnav_navigation_cc camera_obstacle_cc.launch.py \
    with_color:="$WITH_COLOR" \
    depth_width:="$DW" depth_height:="$DH" \
    cloud_hz:="$HZ" pixel_step:="$STEP" \
    > "$LOGDIR/camera_obstacle.log" 2>&1
