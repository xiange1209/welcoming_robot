#!/bin/bash
# ★ 舊版 brain.launch.py 入口，已停用（2026-08-25 加上護欄）。
#   原檔在 舊版_勿用/run_brain.sh。
#
# 舊版把生命週期管理拆成 run_brain.sh + run_nav2.sh 兩支各管一半，
# 這正是「建完圖之後導航就壞掉」的來源（沒有人負責 deactivate slam_toolbox）。
# 現行 _cc 鏈把服務節點放在同一個 launch 裡，模式仲裁統一由 map_service_cc 負責。

cat >&2 <<'EOF'

  ✗ run_brain.sh 已停用 —— 這是舊版 smartnav_navigation 的 brain 入口。

    舊版把生命週期拆成 run_brain.sh + run_nav2.sh 兩支各管一半，
    沒有人負責 deactivate slam_toolbox，建完圖之後導航就壞掉。

    現行 _cc 鏈的服務節點都在同一個 launch 裡，統一由 map_service_cc 仲裁：
      ~/maprun/run_nav_cc.sh

    要單獨跑迎賓劇本節點（smartnav_brain 套件，與這支無關）：
      ~/maprun/run_node_cc.sh smartnav_brain bank_reception

    真的要跑舊版：
      bash ~/maprun/舊版_勿用/run_brain.sh

EOF
exit 1
