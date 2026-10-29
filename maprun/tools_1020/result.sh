#!/usr/bin/env bash
# 手動測試結果登記：一行一項，PASS／FAIL／WARN／未做 + 數字或備註。
# 結果寫進 ~/welcoming_robot_ws/src/smartnav_ws/maprun/verify_out/manual_results_<日期>.tsv，同時用 trip_mark.sh 在上車日誌留一行（兩邊都會被 collect_data.sh --all 帶走）。
#
# 用法：
#   ~/welcoming_robot_ws/src/smartnav_ws/maprun/tools_1020/result.sh <項目編號> <PASS|FAIL|WARN|未做> "<數字或備註>"
#   ~/welcoming_robot_ws/src/smartnav_ws/maprun/tools_1020/result.sh 1 PASS "V0 全過、throttled 0x0、電池 24.1V"
#   ~/welcoming_robot_ws/src/smartnav_ws/maprun/tools_1020/result.sh 4 FAIL "急停後車子還滑了 2 秒；數字見 estop_timing 輸出"
#   ~/welcoming_robot_ws/src/smartnav_ws/maprun/tools_1020/result.sh show            # 看今天登記了什麼
#
# 項目編號（對應 ~/car_docs/今日手動測試步驟_20261020.md）：
#   0 開機檢查  1 V0  2 標地點  3 自身回波  4 平板急停  5 轉向實驗  6 故事線(每輪一行，備註寫第幾輪)
#   7 LLM 十題  8 人臉語音TTS  9 G5 掉頭觀察
# ★ 沒量到就寫「未做」或 WARN，不要補數字（交接文件的規定）。
set -u
DAY="${RESULT_DAY:-$(date +%Y%m%d)}"
F="${RESULT_FILE:-$HOME/maprun/verify_out/manual_results_${DAY}.tsv}"
mkdir -p "$(dirname "$F")"
if [ "${1:-}" = "show" ]; then
  [ -f "$F" ] && column -t -s $'\t' "$F" || echo "今天（$DAY）還沒有登記"
  exit 0
fi
if [ $# -lt 2 ]; then sed -n '2,15p' "$0"; exit 2; fi
ITEM="$1"; STATUS="$2"; NOTE="${3:-}"
case "$STATUS" in PASS|FAIL|WARN|未做) ;; *) echo "✗ 狀態只能是 PASS／FAIL／WARN／未做，你給的是：$STATUS"; exit 2 ;; esac
SYNC=$(timedatectl show -p NTPSynchronized --value 2>/dev/null || echo unknown)
[ "$SYNC" != yes ] && NOTE="${NOTE}（時鐘未同步）"
[ -f "$F" ] || printf '時間\t項目\t狀態\t備註\n' > "$F"
# 備註裡的 Tab／換行換成空白，避免破壞表格
CLEAN=$(printf '%s' "$NOTE" | tr '\t\n' '  ')
printf '%s\t%s\t%s\t%s\n' "$(date +%Y-%m-%dT%H:%M:%S%z)" "$ITEM" "$STATUS" "$CLEAN" >> "$F"
_TM="$(dirname "$(readlink -f "$0")")/../trip_mark.sh"   # ★ 10/29：repo 的 maprun/trip_mark.sh
[ -x "$_TM" ] && "$_TM" "結果 項目$ITEM $STATUS" "$CLEAN" >/dev/null 2>&1
echo "✓ 已登記：項目 $ITEM  $STATUS  $CLEAN"
[ "$SYNC" != yes ] && echo "⚠ 時鐘還沒同步，這筆時間可能不準，已在備註標記"
exit 0
