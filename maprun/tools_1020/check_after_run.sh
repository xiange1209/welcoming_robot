#!/usr/bin/env bash
# 故事線 ×10 跑完後的客觀檢查（交接文件 §5-3 的四個通過標準）。只讀 log，不動任何東西。
#
# 用法：
#   ~/maprun/tools_1020/check_after_run.sh            # 檢查今天 00:00 之後有改動的 log
#   ~/maprun/tools_1020/check_after_run.sh 14:30      # 只看今天 14:30 之後（建議：故事線開始前打一個 trip_mark，用那個時間）
#
# 四項標準與對應的 log：
#   1. ASR log 沒有 `input overflow`                 ~/.smartnav/logs/system_control/asr_chain_*.log
#   2. 導航 log 沒有「控制迴圈只跑到」警告            ~/.smartnav/logs/system_control/nav_*.log
#   3. 定位模式沒有 `/scan_slam` 斷訊                 同上，只算每份 nav log 裡「已進入定位模式」那行**之後**的
#      （建圖模式一定會有這個警告，9/29 實測 38 次；定位模式 0 次——不要把建圖階段的算進來）
#   4. `vcgencmd get_throttled` 是 0x0               ★ 這個旗標從開機起累積（高位元 = 開機以來發生過），所以**要在跑完時看**
# ★ 這支只檢查客觀四項。每輪成功／失敗與原因要自己記：~/maprun/tools_1020/result.sh 6 PASS|FAIL "第N輪 …"
set -u
SINCE_HM="${1:-00:00}"
SINCE="$(date -d "today $SINCE_HM" '+%Y-%m-%d %H:%M:%S' 2>/dev/null)" || { echo "✗ 時間格式要是 HH:MM（例：14:30）"; exit 2; }
LOGS="$HOME/.smartnav/logs/system_control"
echo "檢查起點：$SINCE    現在：$(date '+%F %T')"
echo "日誌目錄：$LOGS"
[ -d "$LOGS" ] || { echo "✗ 找不到日誌目錄"; exit 1; }
pass=0; fail=0; unk=0
verdict() { # verdict <名稱> <值> <通過條件描述> <0=過/1=不過/2=無法判定>
  case "$4" in 0) s="PASS"; pass=$((pass+1));; 1) s="FAIL"; fail=$((fail+1));; *) s="未量測"; unk=$((unk+1));; esac
  printf "  %-7s %-34s %-12s %s\n" "$s" "$1" "$2" "$3"
}

mapfile -t ASR < <(find "$LOGS" -maxdepth 1 -name 'asr_chain_*.log' -newermt "$SINCE" 2>/dev/null | sort)
mapfile -t NAV < <(find "$LOGS" -maxdepth 1 -name 'nav_*.log' -newermt "$SINCE" 2>/dev/null | sort)
echo "範圍內的 log：ASR ${#ASR[@]} 份、導航 ${#NAV[@]} 份"
for f in "${ASR[@]}" "${NAV[@]}"; do echo "    $(basename "$f")  ($(stat -c %y "$f" | cut -c12-19) 最後寫入，$(wc -l < "$f") 行)"; done
echo
echo "── 四項標準 ──"
# 1
if [ ${#ASR[@]} -eq 0 ]; then verdict "1. ASR input overflow" "—" "範圍內沒有 asr_chain log（語音單元沒開？）" 2
else n=0; for f in "${ASR[@]}"; do c=$(grep -c "input overflow" "$f"); n=$((n+c)); [ "$c" -gt 0 ] && echo "      ↳ $(basename "$f"): $c 次"; done
  verdict "1. ASR input overflow" "$n 次" "要求 0 次" $([ "$n" -eq 0 ] && echo 0 || echo 1); fi
# 2
if [ ${#NAV[@]} -eq 0 ]; then verdict "2. 控制迴圈只跑到…警告" "—" "範圍內沒有 nav log" 2
else n=0; for f in "${NAV[@]}"; do c=$(grep -c "控制迴圈只跑到" "$f"); n=$((n+c)); [ "$c" -gt 0 ] && echo "      ↳ $(basename "$f"): $c 次"; done
  verdict "2. 控制迴圈只跑到…警告" "$n 次" "要求 0 次" $([ "$n" -eq 0 ] && echo 0 || echo 1); fi
# 3
if [ ${#NAV[@]} -eq 0 ]; then verdict "3. 定位模式 /scan_slam 斷訊" "—" "範圍內沒有 nav log" 2
else n=0; nolm=0
  for f in "${NAV[@]}"; do
    ln=$(grep -n "已進入定位模式" "$f" | tail -1 | cut -d: -f1)
    if [ -z "$ln" ]; then nolm=$((nolm+1)); continue; fi
    c=$(tail -n +"$ln" "$f" | grep -c "/scan_slam observation buffer has not been updated")
    n=$((n+c)); [ "$c" -gt 0 ] && echo "      ↳ $(basename "$f")（第 $ln 行起算定位模式）: $c 次"
  done
  if [ "$nolm" -eq "${#NAV[@]}" ]; then verdict "3. 定位模式 /scan_slam 斷訊" "—" "這幾份 nav log 都沒有進入定位模式" 2
  else verdict "3. 定位模式 /scan_slam 斷訊" "$n 次" "要求 0 次（$nolm 份 log 沒進定位模式，未算入）" $([ "$n" -eq 0 ] && echo 0 || echo 1); fi
fi
# 4  ★ 這台 Pi 的 /dev/vcio 是 root:video，使用者帳號不在 video 群組 → 不加 sudo 的 vcgencmd 會失敗
#    （Can't open device file: /dev/vcio）。所以依序試：vcgencmd → sudo -n vcgencmd → 韌體 sysfs（同一個旗標）。
thr=""; src=""
t1=$(vcgencmd get_throttled 2>/dev/null | sed -n 's/^throttled=//p')
if [ -n "$t1" ]; then thr="$t1"; src="vcgencmd"; fi
if [ -z "$thr" ]; then t1=$(sudo -n vcgencmd get_throttled 2>/dev/null | sed -n 's/^throttled=//p'); [ -n "$t1" ] && { thr="$t1"; src="sudo vcgencmd"; }; fi
if [ -z "$thr" ] && [ -r /sys/devices/platform/soc/soc:firmware/get_throttled ]; then
  raw=$(cat /sys/devices/platform/soc/soc:firmware/get_throttled); thr="0x${raw}"; src="sysfs(韌體旗標)"; fi
if [ -z "$thr" ]; then verdict "4. get_throttled" "—" "三個來源都讀不到（vcgencmd／sudo／sysfs）→ 記「未量測」，不要猜" 2
else verdict "4. get_throttled" "$thr" "要求 0x0（來源：$src；開機以來累積）" $([ "$thr" = "0x0" ] && echo 0 || echo 1); fi

echo
echo "── 順便記下（不計入通過與否）──"
echo "  時鐘同步：$(timedatectl show -p NTPSynchronized --value 2>/dev/null)"
if [ -f /opt/ros/jazzy/setup.bash ] && [ -f "$HOME/maprun/env.sh" ]; then
  V=$( (set +u; source "$HOME/maprun/env.sh" >/dev/null 2>&1; timeout 8 ros2 topic echo /PowerVoltage --once --field data 2>/dev/null | grep -Eo '^[0-9]+([.][0-9]+)?$' | head -1) )
  echo "  電池電壓（現在）：${V:-讀不到（底盤單元沒開？）} V"
fi
echo "  溫度：$(awk '{printf "%.1f °C",$1/1000}' /sys/class/thermal/thermal_zone0/temp 2>/dev/null || echo 未知)"
echo "  1 分鐘負載：$(cut -d' ' -f1 /proc/loadavg)    可用記憶體：$(awk '/MemAvailable/{printf "%d MB",$2/1024}' /proc/meminfo)"
echo
echo "結果：PASS $pass、FAIL $fail、未量測 $unk"
echo "→ 把這個結果登記：~/maprun/tools_1020/result.sh 6 $([ $fail -eq 0 ] && [ $unk -eq 0 ] && echo PASS || echo WARN) \"故事線客觀四項：PASS $pass／FAIL $fail／未量測 $unk\""
[ "$fail" -eq 0 ]
