#!/bin/bash
# =============================================================================
# smartnav_navigation_cc 分階段測試腳本
# =============================================================================
# 設計原則：由穩到不穩。前一階段沒過就不要往下走，否則問題會疊在一起難以定位。
#
# 用法：
#   ./test_nav_cc.sh clean      階段0  清掉模擬地圖 (上真車前必做)
#   ./test_nav_cc.sh sensors    階段1  啟動底盤/雷達並驗證
#   ./test_nav_cc.sh mapping    階段2  啟動建圖模式並驗證
#   ./test_nav_cc.sh drive      階段3  引導手動遙控建圖 (互動)
#   ./test_nav_cc.sh finish     階段4  結束建圖、存檔、切定位模式
#   ./test_nav_cc.sh navigate   階段5  規劃 + 導航驗證
#   ./test_nav_cc.sh diagnose   隨時可跑：收集完整診斷
#   ./test_nav_cc.sh stop       停掉所有東西
#
#   ./test_nav_cc.sh fake       用假機器人取代真實底盤 (沒接車時)
#
# 環境變數：
#   MAP_NAME=xxx    指定地圖名稱 (預設 test_<時間>)
#   GOAL_DIST=1.0   navigate 階段的目標距車頭幾公尺 (預設 1.0)
#   AUTO_YES=1      跳過所有確認 (包含讓車子實際移動)
# =============================================================================

set -uo pipefail

MAPRUN=/home/user/maprun
LOGDIR="$MAPRUN/logs"
CHECK="$MAPRUN/nav_cc_check.py"
mkdir -p "$LOGDIR"

# 顏色
R='\033[31m'; G='\033[32m'; Y='\033[33m'; B='\033[36m'; N='\033[0m'

step() { echo -e "\n${B}=== $* ===${N}"; }
ok()   { echo -e "${G}✓${N} $*"; }
bad()  { echo -e "${R}✗${N} $*"; }
warn() { echo -e "${Y}!${N} $*"; }

load_env() {
  # ROS 的 setup.bash 會讀 AMENT_TRACE_SETUP_FILES 等未定義變數，
  # 在 set -u 之下會直接中止 ("unbound variable")。source 期間先關掉。
  set +u
  source /home/user/maprun/env.sh
  set -u
  # cyclonedds.xml 不見的話所有節點會 SIGABRT，退回預設比較好除錯
  local f="${CYCLONEDDS_URI#file://}"
  if [ -n "$f" ] && [ ! -f "$f" ]; then
    warn "找不到 $f，改用 CycloneDDS 預設設定"
    unset CYCLONEDDS_URI
  fi
}

# 等某個 log 出現指定樣式，逾時回 1
wait_for_log() {
  local file=$1 pattern=$2 timeout=${3:-180} desc=${4:-$2}
  local t0=$SECONDS
  echo -n "  等待「$desc」"
  while [ $((SECONDS - t0)) -lt "$timeout" ]; do
    if grep -qE "$pattern" "$file" 2>/dev/null; then
      echo " ($((SECONDS - t0))s)"
      return 0
    fi
    sleep 2
    echo -n "."
  done
  echo " 逾時 ${timeout}s"
  return 1
}

# 確認提示。非互動環境 (例如 Claude 用工具執行) 不能卡在 read 上：
#   AUTO_YES=1  一律當成 y
#   非互動且沒設 AUTO_YES -> 用預設值，不阻塞
confirm() {
  local prompt=$1 default=${2:-N}
  if [ "${AUTO_YES:-0}" = "1" ]; then
    echo "$prompt [AUTO_YES=1，自動確認]"
    return 0
  fi
  if [ ! -t 0 ]; then
    echo "$prompt [非互動環境，套用預設值 $default]"
    [[ "$default" =~ ^[Yy]$ ]]
    return
  fi
  local a
  read -rp "$prompt [y/N] " a
  [[ "$a" =~ ^[Yy]$ ]]
}

# -----------------------------------------------------------------------------
stage_clean() {
  step "階段 0：清掉舊地圖"
  echo "現有地圖："
  ls -1 ~/.smartnav/map_database/data/ 2>/dev/null | sed 's/^/    /' || echo "    (無)"
  echo
  if confirm "確定要刪除所有地圖與地點資料庫?"; then
    rm -rf ~/.smartnav/map_database/data/*
    rm -f ~/.smartnav/map_database/maps_db.json ~/.smartnav/nav_state.json
    rm -f ~/.smartnav/waypoint_database/waypoints.json
    ok "已清空 (地圖 + 地點)"
  else
    warn "略過"
  fi
}

stage_sensors() {
  step "階段 1：底盤與雷達"

  echo "USB 序列裝置："
  ls -1 /dev/ttyUSB* 2>/dev/null | sed 's/^/    /' || {
    bad "找不到 /dev/ttyUSB* —— 底盤或雷達沒接上，或 USB 沒列舉成功"
    echo "    提醒：Astra 相機開機列舉常失敗，需要手動插拔一次"
    return 1
  }

  if pgrep -f "lslidar_driver_node" >/dev/null; then
    warn "感測器已在執行，跳過啟動"
  else
    echo "啟動 wheeltec_sensors..."
    setsid nohup "$MAPRUN/run_sensors_cc.sh" </dev/null >/dev/null 2>&1 &
    sleep 15
  fi

  load_env
  python3 "$CHECK" sensors
}

stage_fake() {
  step "階段 1-alt：假機器人 (無底座測試)"
  if pgrep -f "fake_robot_cc" >/dev/null; then
    warn "假機器人已在執行"
  else
    setsid nohup "$MAPRUN/run_fake_robot_cc.sh" </dev/null >/dev/null 2>&1 &
    sleep 8
  fi
  load_env
  python3 "$CHECK" sensors
}

stage_mapping() {
  step "階段 2：啟動建圖模式"

  if ! pgrep -f "lslidar_driver_node|fake_robot_cc" >/dev/null; then
    bad "感測器/假機器人沒在跑，請先執行 sensors 或 fake 階段"
    return 1
  fi

  if pgrep -f "map_service_cc" >/dev/null; then
    warn "導航堆疊已在執行，先停掉再重啟"
    "$MAPRUN/stop_nav_cc.sh" >/dev/null
    sleep 3
  fi

  rm -f "$LOGDIR/nav_cc.log"
  echo "啟動 (start_mode=mapping, use_exploration=false)..."
  setsid nohup "$MAPRUN/run_nav_cc.sh" mapping false </dev/null >/dev/null 2>&1 &
  sleep 5

  if ! wait_for_log "$LOGDIR/nav_cc.log" "已進入建圖模式|等不到 map" 120 "進入建圖模式"; then
    bad "建圖模式切換逾時"
    diagnose_dump
    return 1
  fi
  if grep -q "等不到 map" "$LOGDIR/nav_cc.log"; then
    bad "slam_toolbox active 但沒發 map->base_footprint —— 通常是 /scan 沒資料"
    diagnose_dump
    return 1
  fi
  ok "已進入建圖模式"

  if ! wait_for_log "$LOGDIR/nav_cc.log" "Nav2 已啟動|Nav2 啟動失敗|Failed to bring up" 240 "Nav2 啟動"; then
    bad "Nav2 啟動逾時"
    diagnose_dump
    return 1
  fi
  if grep -qE "Nav2 啟動失敗|Failed to bring up" "$LOGDIR/nav_cc.log"; then
    bad "Nav2 啟動失敗"
    grep -E "Exception when loading BT|Failed to change state|not available" "$LOGDIR/nav_cc.log" | tail -5
    return 1
  fi
  ok "Nav2 已啟動"

  load_env
  python3 "$CHECK" mode
  python3 "$CHECK" nav2
}

stage_drive() {
  step "階段 3：手動遙控建圖"

  local name="${MAP_NAME:-test_$(date +%H%M)}"
  load_env

  echo "送出 create_map (地圖名稱: $name)..."
  setsid nohup bash -c "source $MAPRUN/env.sh; ros2 action send_goal /create_map \
    smartnav_msgs/action/CreateMap \"{map_name: '$name'}\" \
    > $LOGDIR/create_map_cc.log 2>&1" </dev/null >/dev/null 2>&1 &
  # ros2 action send_goal 要先等 action server discovery，5 秒常常不夠，
  # 尤其這個網段還有別台機器的節點在拖慢 discovery。輪詢到出現結果為止。
  if wait_for_log "$LOGDIR/create_map_cc.log" "Goal accepted|Goal was rejected" 45 "create_map 回應"; then
    if grep -q "Goal accepted" "$LOGDIR/create_map_cc.log"; then
      ok "create_map 已接受"
    else
      bad "create_map 被拒絕 (地圖名稱重複?)"
      tail -5 "$LOGDIR/create_map_cc.log"
      return 1
    fi
  else
    warn "45 秒內沒有回應，檢查 create_map_cc.log："
    tail -5 "$LOGDIR/create_map_cc.log" 2>/dev/null
  fi
  echo "$name" > "$LOGDIR/.current_map_name"

  cat <<'EOF'

  ┌─────────────────────────────────────────────────────────┐
  │  現在開始遙控建圖                                        │
  │                                                          │
  │  另開一個終端機執行：                                    │
  │    source /home/user/maprun/env.sh                       │
  │    ros2 run teleop_twist_keyboard teleop_twist_keyboard  │
  │                                                          │
  │  ★ 阿克曼車不能原地轉 ★                                 │
  │    用 i 前進 / , 後退，搭配 j l 轉向                     │
  │    速度壓在 0.3 m/s 以下，建圖品質比較穩                 │
  │                                                          │
  │  把環境走一圈、回到起點附近後，執行：                    │
  │    ./test_nav_cc.sh finish                               │
  └─────────────────────────────────────────────────────────┘

EOF
  echo "建圖過程中可隨時另開終端機檢查地圖成長："
  echo "    $MAPRUN/test_nav_cc.sh diagnose"
}

stage_finish() {
  step "階段 4：結束建圖並存檔"
  load_env

  local name
  name=$(cat "$LOGDIR/.current_map_name" 2>/dev/null || echo "?")
  echo "地圖名稱: $name"

  echo "呼叫 /finish_map..."
  timeout 30 ros2 service call /finish_map std_srvs/srv/Trigger 2>&1 | tail -3

  if ! wait_for_log "$LOGDIR/nav_cc.log" "地圖建立成功|地圖存檔失敗|切回定位模式失敗|系統出現異常" 180 "存檔並切回定位模式"; then
    bad "建圖收尾逾時"
    diagnose_dump
    return 1
  fi

  if grep -q "地圖建立成功" "$LOGDIR/nav_cc.log"; then
    ok "$(grep '地圖建立成功' "$LOGDIR/nav_cc.log" | tail -1 | sed 's/.*: //')"
  else
    bad "建圖收尾失敗："
    grep -E "地圖存檔失敗|切回定位模式失敗|系統出現異常" "$LOGDIR/nav_cc.log" | tail -3
    diagnose_dump
    return 1
  fi

  echo
  echo "存檔結果："
  ls -la ~/.smartnav/map_database/data/ | tail -5
  echo
  python3 "$CHECK" mode
  python3 "$CHECK" map
}

stage_navigate() {
  step "階段 5：導航驗證"
  load_env

  echo "先確認核心不變量 (這是舊版壞掉的地方)..."
  if ! python3 "$CHECK" mode || ! python3 "$CHECK" map; then
    bad "模式或地圖狀態不對，先解決再導航"
    return 1
  fi

  echo
  echo "測試規劃器 (只規劃，車子不會動)..."
  # 規劃到目前位置前方一點
  local coords
  coords=$(python3 - <<'PY'
import math, sys, time, threading, os
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import tf2_ros
rclpy.init(); n=Node("goalcalc"); ex=MultiThreadedExecutor(); ex.add_node(n)
threading.Thread(target=ex.spin,daemon=True).start()
buf=tf2_ros.Buffer(); tf2_ros.TransformListener(buf,n)
# TF buffer 剛建立時是空的，map -> base_footprint 要等 amcl 發第一則。
# 一定要重試，sleep 個兩秒就查會拿到 "target_frame does not exist"。
tf=None
for _ in range(30):
    try:
        tf=buf.lookup_transform("map","base_footprint",rclpy.time.Time()); break
    except Exception: time.sleep(0.5)
if tf is None:
    print("ERR 等不到 map -> base_footprint", file=sys.stderr)
else:
    q=tf.transform.rotation
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    x=tf.transform.translation.x; y=tf.transform.translation.y
    # 目標放在車頭前方 GOAL_DIST 公尺處。
    # 太近 (< 0.8 m 轉彎半徑) 阿克曼規劃不出來；太遠可能頂到走廊末端而 no valid path。
    d = float(os.environ.get("GOAL_DIST", "1.0"))
    print(f"{x+d*math.cos(yaw):.3f} {y+d*math.sin(yaw):.3f}")
# os._exit 會跳過 stdout 的緩衝區清空，不 flush 的話上面印的座標根本傳不出去
sys.stdout.flush(); sys.stderr.flush()
os._exit(0)
PY
)
  if [ -z "$coords" ]; then
    bad "算不出目標座標 (map->base_footprint 讀不到)"
    return 1
  fi
  echo "  目標: $coords"
  python3 "$CHECK" plan $coords || {
    bad "規劃失敗 —— 這是舊版的主要症狀，把 nav_cc.log 給我看"
    grep -E "Start occupied|no valid path" "$LOGDIR/nav_cc.log" | tail -5
    return 1
  }

  echo
  # 車子會實際移動，預設不做。要跑得明確帶 AUTO_YES=1 或在互動終端機確認。
  warn "接下來會讓車子實際移動到 ($coords)"
  if ! confirm "確認周圍淨空、你人在旁邊?" N; then
    warn "略過實際移動測試 (規劃已驗證通過)"
    echo "    要實際跑： AUTO_YES=1 $0 navigate"
    return 0
  fi

  echo "送出 /navigate ..."
  set -- $coords
  timeout 200 ros2 action send_goal /navigate smartnav_msgs/action/Navigate \
    "{use_target_pose: true, target_pose: {position: {x: $1, y: $2}, orientation: {w: 1.0}}, target_name: '測試點'}" \
    2>&1 | tail -12
}

diagnose_dump() {
  step "診斷資訊"
  load_env
  python3 "$CHECK" all
  echo
  echo "--- nav_cc.log 最後的錯誤 ---"
  grep -E "\[(ERROR|FATAL)\]" "$LOGDIR/nav_cc.log" 2>/dev/null | tail -15 || echo "(無)"
  echo
  echo "--- 重複最多的警告 ---"
  grep -oE "\[WARN\] \[[0-9.]+\] \[[^]]*\]: .*" "$LOGDIR/nav_cc.log" 2>/dev/null \
    | sed -E 's/\[[0-9.]+\]//' | sort | uniq -c | sort -rn | head -8 || echo "(無)"
  echo
  echo "--- 行程 ---"
  # pgrep -fc 找不到時會印 0 並回傳 1，用 || echo 0 會變成印兩個 0
  local n
  for pair in "感測器:lslidar_driver_node|wheeltec_robot_node" \
              "假機器人:fake_robot_cc" \
              "導航堆疊:nav2_|map_service_cc"; do
    n=$(pgrep -fc "${pair#*:}" 2>/dev/null); n=${n:-0}
    echo "  ${pair%%:*}: $n"
  done
}

# -----------------------------------------------------------------------------
case "${1:-help}" in
  clean)    stage_clean ;;
  sensors)  stage_sensors ;;
  fake)     stage_fake ;;
  mapping)  stage_mapping ;;
  drive)    stage_drive ;;
  finish)   stage_finish ;;
  navigate) stage_navigate ;;
  diagnose) diagnose_dump ;;
  stop)
    "$MAPRUN/stop_nav_cc.sh"
    for p in $(pgrep -f "fake_robot_cc"); do kill "$p" 2>/dev/null; done
    ok "已停止"
    ;;
  help|*)
    # 從第 2 行印到第一個非註解行為止 (就是檔頭那段說明)
    sed -n '2,${/^[^#]/q;p;}' "$0" | sed 's/^# \?//'
    ;;
esac
