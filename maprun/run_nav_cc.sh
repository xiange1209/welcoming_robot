#!/bin/bash
# 新版導航/建圖總啟動 (smartnav_navigation_cc)
#
# 舊版要分別跑 run_brain.sh + run_nav2.sh 兩支，兩邊各自管一部分生命週期。
# 新版只有這一支：nav2、地圖來源 (amcl/map_server/slam_toolbox) 與 SmartNav
# 服務節點都在同一個 launch 裡，模式仲裁統一由 map_service_cc 負責。
#
# 用法：
#   ./run_nav_cc.sh                 # auto：有地圖就定位，沒地圖就進建圖待命
#   ./run_nav_cc.sh mapping         # 強制建圖模式
#   ./run_nav_cc.sh localization    # 強制定位模式

source /home/user/maprun/env.sh

# env.sh 把 CYCLONEDDS_URI 指向 ~/cyclonedds.xml。那個檔案一旦不見，
# CycloneDDS 會在建立 domain 時直接失敗，整串節點全部 SIGABRT：
#   can't open configuration file file:///home/user/cyclonedds.xml
#   rmw_create_node: failed to create domain, error Error
# 檔案不在就退回預設設定 (節點多的時候可能會撞到 participant index 上限，
# 但至少跑得起來，而且錯誤訊息會很明確)。
CYCLONEDDS_FILE="${CYCLONEDDS_URI#file://}"
if [ -n "$CYCLONEDDS_FILE" ] && [ ! -f "$CYCLONEDDS_FILE" ]; then
  echo "[run_nav_cc] 警告：找不到 $CYCLONEDDS_FILE，改用 CycloneDDS 預設設定" >&2
  unset CYCLONEDDS_URI
fi

START_MODE="${1:-auto}"
# 第二個參數：自動探索開關。false = 建圖時不自動跑，改用遙控走完再 /finish_map
USE_EXPLORATION="${2:-true}"

# launch 那邊其實有**兩個**參數，而且只設一個會產生矛盾的組合：
#
#   use_exploration        要不要「啟動」frontier_explorer 節點
#   auto_start_exploration create_map 時要不要「立刻自動探索」
#                          （這個才是餵給 map_service_cc 的那一個）
#
# 只給 use_exploration:=false 的話，explorer 節點不會啟動，
# 但 map_service_cc 仍然以為要自動探索，於是去呼叫根本不存在的
# /control_exploration，等 15 秒後 create_map 失敗
# —— 而且錯誤只進 log，畫面上什麼都看不到。
# 2026-07-31 實測：使用者連按三次「開始建圖」全部失敗還以為成功了。
#
# 這兩個要一起設。想要「explorer 在線但待命、由 /start_exploration 手動觸發」
# 的話，請直接呼叫 ros2 launch 並分別指定，不要走這支腳本。
exec ros2 launch smartnav_navigation_cc nav_bringup_cc.launch.py \
  use_sim_time:=false use_rviz:=false \
  start_mode:="$START_MODE" \
  use_exploration:="$USE_EXPLORATION" \
  auto_start_exploration:="$USE_EXPLORATION" \
  > "$LOGDIR/nav_cc.log" 2>&1
