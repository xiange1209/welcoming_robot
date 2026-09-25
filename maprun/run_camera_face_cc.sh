#!/bin/bash
# 人臉模式相機：只開彩色，不開深度／IR／點雲。
#
# ★★ 2026-09-18 新增：這支腳本以前**不存在** ★★
#
# `camera_face_only.launch.py` 從 8/14 就寫好了，`kill_camera_cc.sh:46` 也一直知道
# 怎麼殺它 —— 但整個 maprun 裡**沒有任何腳本會啟動它**，HMI 的 SYSTEM_UNITS 裡
# 也沒有對應單元。結果是「只有 kill 那一半，沒有 start 那一半」：
#
#     想在平板上測人臉辨識 -> 按得到「人臉辨識」節點 -> 但沒有東西餵它影像
#     -> 得 ssh 進來手打 ros2 launch，而那正是 HMI 要消滅的事
#
# 用法： run_camera_face_cc.sh [color_fps] [jpeg_quality]
# 例：   run_camera_face_cc.sh 15 85
#
# ⚠ 與 run_camera_obstacle_cc.sh **互斥** —— 兩者都會起 astra_camera_node，
#   而相機只有一台。要換模式先 kill_camera_cc.sh。
#   HMI 上兩者是同一個單元的兩個 variant，就是為了讓這件事在介面上是明確的。
#
# ⚠ color_fps 在這台相機上**不保證生效**：9/17 實驗室實測 ros2 param get 回 15、
#   驅動啟動 log 也印「640x480@15Hz」，但 `ros2 topic hz` 量到 **29.64 Hz**。
#   要確認實際幀率只能用 ros2 topic hz，不要相信參數與 log。

set +u
source /home/user/maprun/env.sh
CYCLONEDDS_FILE="${CYCLONEDDS_URI#file://}"
if [ -n "$CYCLONEDDS_FILE" ] && [ ! -f "$CYCLONEDDS_FILE" ]; then unset CYCLONEDDS_URI; fi

FPS="${1:-15}"
QUALITY="${2:-85}"

exec ros2 launch /home/user/maprun/camera_face_only.launch.py \
    color_fps:="$FPS" \
    jpeg_quality:="$QUALITY" \
    > "$LOGDIR/camera_face.log" 2>&1
