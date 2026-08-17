#!/bin/bash
# 停掉導航堆疊 (不動底盤/雷達)
#
# 注意：不要在互動 shell 裡直接打 `pkill -f nav_bringup_cc`。
# pkill -f 比對的是完整命令列，而你打的那行命令本身就含有這個字串，
# 於是 shell 會先把自己殺掉，指令只執行到一半。
# 把樣式放進腳本檔就沒這問題：這支腳本的命令列只有它自己的路徑。

# ★ 2026-08-14 補上 path_teach_cc 與 steering_trim_cc。
#   在那之前它們不在清單裡，所以 stop_nav_cc.sh 收不掉，而 run_nav_cc.sh 會再開一份
#   —— 重啟一次導航就變成**兩個 path_teach_cc 同時提供同名服務**，
#   呼叫隨機落到其中一個。當天的症狀是「plan_taught_path 回傳失敗，
#   log 卻顯示路徑存檔成功」，看起來像程式自相矛盾，實際是在跟兩個節點講話。
#   ★ 做任何 A/B 對照之前，先確認節點只有一個。
# ★ 2026-08-17 再補 stuck_detector_cc 與 scan_filter_cc。
#   實測後果：一天之內重啟幾次導航之後，機器上同時有
#   **四份 stuck_detector_cc、三份 scan_filter_cc**（cpu_report.py 抓到的）。
#   除了白吃約 26% CPU、365 MB 記憶體，更嚴重的是
#   **多份 scan_filter_cc 會同時發布過濾後的雷達**，下游收到交錯的訊息，
#   定位吻合度掉到 88% 且車頭淨空對不上很可能就是這個造成的。
PATTERNS='nav2_|map_service_cc|waypoint_service_cc|navigation_action_cc|path_teach_cc|steering_trim_cc|stuck_detector_cc|scan_filter_cc|async_slam_toolbox|frontier_explorer|smartnav_navigation_cc nav_bringup'

MYPID=$$
MYPPID=$PPID

# 絕對不能殺到呼叫端。踩過兩次：
#   1. 互動 shell 裡打 `pkill -f nav_bringup_cc` -> 那行指令自己就含這個字串，
#      shell 先把自己殺掉，後面的指令都沒執行。
#   2. 在同一行裡寫 `./stop_nav_cc.sh; ps | grep -c 'nav2_|map_service_cc'`
#      -> 呼叫端 bash 的命令列含有 pattern，被這支腳本殺掉 (exit 144)，
#         接在後面的啟動指令整個沒跑，卻看起來像「啟動失敗」。
# 所以這裡明確排除自己、父行程，以及任何 pgrep 撈到的 shell。
should_skip() {
  local pid=$1
  [ "$pid" = "$MYPID" ] && return 0
  [ "$pid" = "$MYPPID" ] && return 0
  # 排除 shell 本身 (bash/sh)，它們只是命令列剛好含有 pattern
  case "$(ps -p "$pid" -o comm= 2>/dev/null)" in
    bash|sh|dash|zsh) return 0 ;;
  esac
  return 1
}

kill_matching() {
  local sig=$1
  for p in $(pgrep -f "$PATTERNS" 2>/dev/null); do
    should_skip "$p" && continue
    kill "-$sig" "$p" 2>/dev/null
  done
}

kill_matching TERM
sleep 4
kill_matching 9
sleep 1

# pgrep -c 找不到東西時會印 0 並回傳 1，用 || echo 0 會多印一行，所以吞掉回傳值
REMAIN=$(pgrep -fc "$PATTERNS" 2>/dev/null); REMAIN=${REMAIN:-0}
SENSORS=$(pgrep -fc 'lslidar_driver_node|wheeltec_robot_node|ekf_node' 2>/dev/null); SENSORS=${SENSORS:-0}
echo "導航堆疊剩餘行程: $REMAIN"
echo "底盤/雷達行程: $SENSORS"
exit 0
