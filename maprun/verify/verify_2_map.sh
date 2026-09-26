#!/usr/bin/env bash
# V2 建圖後的記錄 —— 建圖本身要人遙控，這支負責「確認地圖存了」＋「記下三個寬度」。
#
# ★ 為什麼要量寬度：選路門檻是淨寬 >= 1.17 公尺（不是舊文件的 1.02）。
#   門口窄轉角 0.967 公尺已判定過不去，路線要避開最窄處。
#
# 建圖流程（這支之前先做完）：
#   ~/maprun/run_sensors_cc.sh
#   ros2 launch smartnav_navigation_cc mapping_manual_cc.launch.py
#   遙控繞完全場 → 存檔
#
# 用法：~/maprun/verify/verify_2_map.sh

source "$(dirname "$0")/_lib.sh"

echo "V2 建圖結果與場地量測   $(date '+%Y-%m-%d %H:%M:%S')"

# ── 地圖有沒有真的存下來 ────────────────────────────────────
hdr "地圖檔"
MDB="$HOME/.smartnav/map_database"
NEW=$(find "$MDB" -name '*.pgm' -newermt '-6 hours' 2>/dev/null | head -5)
if [ -n "$NEW" ]; then
  echo "$NEW" | while read -r f; do
    rec "新地圖" PASS "$(basename "$f") $(du -h "$f" | cut -f1)" "$(stat -c %y "$f" | cut -d. -f1)"
  done
else
  rec "新地圖" FAIL "6 小時內沒有新的 .pgm" "★ 活躍地圖可能還停在家裡那張，導航實驗全都不算數"
fi

ACTIVE=$(python3 -c "
import json,glob,os
p=os.path.expanduser('~/.smartnav/map_database/maps_db.json')
try:
    d=json.load(open(p,encoding='utf-8'))
    print(d.get('active') or d.get('active_map') or '(欄位不明)')
except Exception as e:
    print('讀不到:',e)
" 2>/dev/null)
rec "活躍地圖" INFO "$ACTIVE" "要是今天建的那張"

# ── 場地量測（人拿捲尺，腳本只負責記錄與判定）────────────────
hdr "場地淨寬（拿捲尺量，這裡只記錄）"
echo "  選路門檻：淨寬 >= 1.17 公尺才排進路線"
echo
ask "門口淨寬（公尺）"       W_DOOR
ask "走廊淨寬（公尺）"       W_HALL
ask "預定路線最窄處（公尺）" W_MIN

verdict() { # verdict <名稱> <值>
  local n="$1" v="$2"
  if [ -z "$v" ]; then rec "$n" WARN "未填" ""; return; fi
  if [ "$(awk -v x="$v" 'BEGIN{print (x>=1.17)?1:0}')" = 1 ]; then
    rec "$n" PASS "${v} m" "達 1.17 m 門檻，可排進路線"
  else
    rec "$n" FAIL "${v} m" "★ 低於 1.17 m —— 這段要避開，不要指望校舵機救得回來"
  fi
}
echo
verdict "門口淨寬"   "$W_DOOR"
verdict "走廊淨寬"   "$W_HALL"
verdict "路線最窄處" "$W_MIN"

banner_done
