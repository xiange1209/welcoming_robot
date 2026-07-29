#!/bin/bash
# 停掉導航堆疊 (不動底盤/雷達)
#
# 注意：不要在互動 shell 裡直接打 `pkill -f nav_bringup_cc`。
# pkill -f 比對的是完整命令列，而你打的那行命令本身就含有這個字串，
# 於是 shell 會先把自己殺掉，指令只執行到一半。
# 把樣式放進腳本檔就沒這問題：這支腳本的命令列只有它自己的路徑。

PATTERNS='nav2_|map_service_cc|waypoint_service_cc|navigation_action_cc|async_slam_toolbox|frontier_explorer|smartnav_navigation_cc nav_bringup'

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
