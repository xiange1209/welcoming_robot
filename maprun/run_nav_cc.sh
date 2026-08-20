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

# ★★ 2026-08-17：啟動前先擋兩件事 ★★
#
# (1) 重複堆疊。stop_nav_cc.sh 的註解記著實測結果：一天內重啟幾次導航之後，
#     機器上同時有**四份 stuck_detector_cc、三份 scan_filter_cc**。
#     除了白吃 26% CPU / 365 MB，更嚴重的是多份 scan_filter_cc 會**同時發布
#     過濾後的雷達**，下游 amcl 收到交錯訊息 -> 吻合度掉到 88%。
#     ★ 這是「導航忽然變準／忽然變爛」最可能的隱形變因，做 A/B 前必須排除。
#     偵測用 pgrep -x（比對執行檔名，不是命令列字串）—— 專案禁用 `pgrep -f`，
#     因為字串比對誤殺過七次。
# ★★ 2026-08-20：pgrep -x 對這兩個名字其實**永遠比不中** ★★
#
#   pgrep -x 比對的是 /proc/<pid>/comm，而 Linux 的 TASK_COMM_LEN = 16（含 NUL），
#   也就是**只有 15 個可用字元**。逐一數：
#       controller_server  17 -> comm = "controller_serv"  比不中
#       stuck_detector_cc  17 -> comm = "stuck_detector_"  比不中
#       amcl                4 -> 可用，但**建圖模式下 amcl 根本不跑**
#   所以這道防護等於完全失效，而本檔上面自己記著
#   「實測同時有四份 stuck_detector_cc、三份 scan_filter_cc，吻合度掉到 88%」。
#
#   改用 ros2 node list —— ROS 圖是這件事唯一可靠的事實來源，
#   而且不違反專案禁用 `pgrep -f` 的鐵則（那是字串比對誤殺的來源）。
_dupes=$(ros2 node list 2>/dev/null | grep -cE "controller_server|amcl|scan_filter_cc|stuck_detector_cc|path_teach_cc")
if [ "${_dupes:-0}" -gt 0 ]; then
  echo "[run_nav_cc] ✗ 偵測到導航堆疊已經在跑（controller_server / amcl）。" >&2
  echo "[run_nav_cc]   再啟動一次會產生重複節點，雷達與定位會開始互相干擾。" >&2
  echo "[run_nav_cc]   請先執行：~/maprun/stop_nav_cc.sh" >&2
  echo "[run_nav_cc]   確定要強制啟動請設 FORCE_NAV=1 再跑一次。" >&2
  [ "$FORCE_NAV" = "1" ] || exit 1
  echo "[run_nav_cc] ⚠ FORCE_NAV=1，照使用者要求繼續啟動" >&2
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
# ★★ (2) 日誌不再被覆蓋 ★★
#
# 原本是 `> $LOGDIR/nav_cc.log`，**每次重啟都把上一輪整個蓋掉**。
# 8/17 的代價很具體：當天 8 趟成功導航的 log 全部消失，收工時 nav_cc.log
# 只剩最後一次重啟（19:08:30）之後的內容，而那一段裡只有失敗。
# ★ 12 月報告要引用的實驗數據就是從這裡來的 —— 覆蓋等於銷毀證據。
#
# 改成「每次一個帶時間戳的檔 + nav_cc.log 符號連結指向最新」：
#   既有的 `tail -f $LOGDIR/nav_cc.log` 習慣完全不用改，
#   但歷史留得下來。舊檔只保留最近 20 份，避免把 SD 卡塞爆。
NAV_LOG="$LOGDIR/nav_cc_$(date +%Y%m%d_%H%M%S).log"
ln -sfn "$NAV_LOG" "$LOGDIR/nav_cc.log"
ls -1t "$LOGDIR"/nav_cc_*.log 2>/dev/null | tail -n +21 | xargs -r rm -f
echo "[run_nav_cc] 日誌 -> $NAV_LOG"

exec ros2 launch smartnav_navigation_cc nav_bringup_cc.launch.py \
  use_sim_time:=false use_rviz:=false \
  start_mode:="$START_MODE" \
  use_exploration:="$USE_EXPLORATION" \
  auto_start_exploration:="$USE_EXPLORATION" \
  > "$NAV_LOG" 2>&1
