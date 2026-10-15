#!/usr/bin/env bash
# 上車日誌：每個階段開始／結束、或發生什麼事，就打一行。
# 事後分析工具靠它把 log 切成階段、把「人看到的事」對到「log 的時間」。
#
# 用法：
#   ~/maprun/trip_mark.sh "T0 開始" "電量 24.1V、第 1 輪"
#   ~/maprun/trip_mark.sh "T0 第3輪：車子在門口停住"
#   ~/maprun/trip_mark.sh show                 # 看今天記了什麼
# 檔案：~/maprun/logs/trip_<日期>.csv（欄位 iso_time,phase,note）；collect_data.sh 會收走。
# 建議在 ~/.bashrc 加一行：alias trip='~/maprun/trip_mark.sh'
#
# ★ Pi 沒有 RTC，開機後要等 NTP 同步，時鐘才對。未同步時這裡仍會記錄，但在備註加「（時鐘未同步）」。
set -u
LOGDIR="$HOME/maprun/logs"; mkdir -p "$LOGDIR"
F="$LOGDIR/trip_$(date +%Y%m%d).csv"
case "${1:-}" in
  ""|-h|--help) sed -n '2,10p' "$0"; exit 0 ;;
  show) if [ -f "$F" ]; then cat "$F"; else echo "今天（$(date +%F)）還沒有紀錄"; fi; exit 0 ;;
esac
SYNC=$(timedatectl show -p NTPSynchronized --value 2>/dev/null || echo unknown)
NOTE="${2:-}"
[ "$SYNC" != yes ] && NOTE="${NOTE}（時鐘未同步）"
[ -f "$F" ] || echo "iso_time,phase,note" > "$F"
q() { local s=${1//\"/\"\"}; printf '"%s"' "$s"; }     # CSV：雙引號加倍
printf '%s,%s,%s\n' "$(date +%Y-%m-%dT%H:%M:%S%z)" "$(q "$1")" "$(q "$NOTE")" >> "$F"
echo "✓ 已記錄 $(date +%T)  $1  ${2:-}"
[ "$SYNC" != yes ] && echo "⚠ 時鐘還沒同步：這筆時間可能不準，已在備註標記"
exit 0
