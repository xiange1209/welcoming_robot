#!/bin/bash
# B（行動電源供電）的電源紀錄：每 10 秒記一次欠壓／降頻旗標、溫度、負載到 ~/maprun/logs/b_power_<日期>.csv。
# 只記錄、不會關機（要不要做「欠壓自動關機」由筆電端決定，README 第 7 節）。
#   手動：~/maprun/dualboard/b_power_log.sh        開機自動：systemd/smartnav-b-powerlog.service
# 旗標讀 sysfs（不用 sudo；vcgencmd 要 sudo，記憶 vcgencmd-needs-sudo）：
#   bit0＝現在欠壓　bit1＝現在 ARM 降頻　bit2＝現在節流　bit3＝現在溫度軟限；bit16~19＝開機以來發生過
# ★ 2026-10-22 車上 agent 起草，只做過 bash -n 與短跑 3 列。
OUT_DIR=/home/user/maprun/logs
F=/sys/devices/platform/soc/soc:firmware/get_throttled
mkdir -p "$OUT_DIR"
N=${1:-0}        # 給數字＝只記 N 列就結束（測試用）；不給＝一直記
i=0
while :; do
  OUT="$OUT_DIR/b_power_$(date +%Y%m%d).csv"
  [ -f "$OUT" ] || echo "time,throttled_hex,undervolt_now,arm_capped_now,throttled_now,undervolt_since_boot,temp_c,load1" > "$OUT"
  T=$(tr -d '\n' < "$F" 2>/dev/null); T=${T:-0}
  V=$((16#$T))
  TEMP=$(awk '{printf "%.1f", $1/1000}' /sys/class/thermal/thermal_zone0/temp 2>/dev/null)
  echo "$(date +%FT%T),0x$T,$((V & 1)),$(((V >> 1) & 1)),$(((V >> 2) & 1)),$(((V >> 16) & 1)),$TEMP,$(cut -d' ' -f1 /proc/loadavg)" >> "$OUT"
  i=$((i + 1)); [ "$N" -gt 0 ] && [ "$i" -ge "$N" ] && break
  sleep 10
done
