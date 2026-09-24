#!/usr/bin/env bash
# 驗證腳本共用函式。各 verify_*.sh 一律 source 這支。
#
# 設計原則：
#   1. 螢幕上直接印 PASS / FAIL，讓你當場就知道過沒過，不必事後解讀 log
#   2. 同時把每一項結果寫成一行 TSV 到 $RESULT，之後 collect_data.sh 一次打包
#   3. 能用指令量的就自動量；只有人要拿尺量的才提示輸入
#
# ★ 專案鐵則：不准 pkill -f / pgrep -f（誤殺過七次）。要停節點用 kill_node_cc.sh。

set -uo pipefail

STAMP="${STAMP:-$(date +%Y%m%d_%H%M%S)}"
VDIR="${VDIR:-$HOME/maprun/verify_out/$STAMP}"
RESULT="$VDIR/results.tsv"
mkdir -p "$VDIR"
[ -f "$RESULT" ] || printf "項目\t結果\t量到的值\t判定依據\n" > "$RESULT"

C_OK=$'\e[32m'; C_NG=$'\e[31m'; C_WARN=$'\e[33m'; C_DIM=$'\e[2m'; C_0=$'\e[0m'

hdr() { printf '\n%s\n' "══ $* ══"; }

# rec <項目> <PASS|FAIL|WARN|INFO> <量到的值> <判定依據>
rec() {
  local item="$1" verdict="$2" value="$3" basis="$4" color
  case "$verdict" in
    PASS) color="$C_OK" ;;
    FAIL) color="$C_NG" ;;
    WARN) color="$C_WARN" ;;
    *)    color="$C_DIM" ;;
  esac
  printf '  %s%-4s%s %-34s %s\n' "$color" "$verdict" "$C_0" "$item" "$value"
  [ -n "$basis" ] && printf '       %s%s%s\n' "$C_DIM" "$basis" "$C_0"
  printf '%s\t%s\t%s\t%s\n' "$item" "$verdict" "$value" "$basis" >> "$RESULT"
}

# ask <提示> <變數名> —— 人工量測值，存進結果檔
ask() {
  local prompt="$1" var="$2" ans
  read -r -p "  ▸ $prompt: " ans
  printf -v "$var" '%s' "$ans"
  printf '%s\t手動\t%s\t現場量測\n' "$prompt" "$ans" >> "$RESULT"
}

# 量話題實際輸出頻率。★ ros2 param get 不算驗證，只有量輸出才算。
# hz_of <話題> <秒數>  → 回傳平均 Hz（量不到印 0）
hz_of() {
  local topic="$1" secs="${2:-8}" out
  out=$(timeout "$secs" ros2 topic hz "$topic" --window 40 2>/dev/null \
        | grep -o 'average rate: [0-9.]*' | tail -1 | grep -o '[0-9.]*')
  echo "${out:-0}"
}

# 某個執行檔目前吃多少 CPU（%）。用 ps 的完整指令欄比對，不用 pgrep -f。
cpu_of() {
  ps -eo pcpu,args --no-headers 2>/dev/null \
    | awk -v pat="$1" '$0 ~ pat && $0 !~ /awk/ {s += $1} END {printf "%.1f", s+0}'
}

banner_done() {
  printf '\n%s結果已寫入%s %s\n' "$C_DIM" "$C_0" "$RESULT"
  printf '%s全部跑完後執行%s ~/maprun/verify/collect_data.sh %s打包回傳%s\n\n' \
         "$C_DIM" "$C_0" "$C_DIM" "$C_0"
}
