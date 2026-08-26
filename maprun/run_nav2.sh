#!/bin/bash
# ★ 這支是舊版 smartnav_navigation 的啟動腳本，已停用（2026-08-25 加上護欄）。
#
# 為什麼需要護欄：舊版_勿用/README.txt 寫「移到這裡是為了避免誤用」，
# 但當時是**複製**不是搬移 —— 頂層這份原封不動地留著，內容與
# 舊版_勿用/run_nav2.sh 逐位元組相同，沒有任何警告。
# 而 `run_nav<TAB>` 在 run_nav2.sh 與 run_nav_cc.sh 之間是有歧義的。
#
# 舊版跑起來的症狀（2026-07-31 實測）正好就是「導航非常不穩定」：
#   - 建完圖之後導航就壞掉：slam_toolbox 從來沒有被 deactivate，
#     /map 與 map->odom_combined 都有兩個發布者
#   - planner_server 連噴 542 次 "Start occupied"
#   - 恢復行為裡有 <Spin>，阿克曼車不能原地轉，每次都走到逾時才失敗
#   - 全域定位時車子完全不動（發 TwistStamped 但底盤訂 Twist，型別不符）
#
# 而且舊鏈用的是 odom frame、現行 _cc 鏈用 odom_combined，兩條不可混用。
# 舊鏈也沒有 cmd_vel_floor_cc（速度下限）與 path_teach 的車身外緣硬停。

cat >&2 <<'EOF'

  ✗ run_nav2.sh 已停用 —— 這是舊版 smartnav_navigation 的入口。

    舊版的已知症狀就是「導航不穩定」：slam_toolbox 沒 deactivate 造成
    /map 與 map->odom_combined 雙發布者、planner 連噴 Start occupied、
    恢復行為含 <Spin>（阿克曼不能原地轉，只會走到逾時）、
    全域定位時 TwistStamped/Twist 型別不符導致車子完全不動。

    現在該用的是：
      ~/maprun/run_sensors_cc.sh      底盤 + 雷達 + IMU 補償
      ~/maprun/run_nav_cc.sh          導航（auto / mapping）
      ~/maprun/create_map_cc.sh       建圖

    真的要跑舊版（只為了對照實驗）：
      bash ~/maprun/舊版_勿用/run_nav2.sh

EOF
exit 1
