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

# 某個執行檔「現在」吃多少 CPU（%，多核相加，四核全滿 = 400%）。用 ps 的完整指令欄比對，不用 pgrep -f。
# cpu_of <指令關鍵字> [取樣秒數，預設 3]
# ★★ 2026-09-30：原本用 ps 的 pcpu —— 那是「行程啟動以來的平均」，**不是瞬時值**。
#    9/17 的「127%」就是節點啟動約 90 秒時量的（其中約 35 秒在載模型，平均被拉低），
#    車子端 9/22 改用 /proc 差分量到瞬時 203%。現在取兩次 /proc/<pid>/stat 的 utime+stime 差。
#    （欄位從最後一個 ')' 之後算：指令名稱在括號裡、可能含空白，直接數欄位會錯位。）
cpu_of() {
  local pat="$1" secs="${2:-3}" pids hz p v t0=0 t1=0
  # -ww：不截斷指令列（節點的完整路徑很長，被截掉就比對不到）
  pids=$(ps -ww -eo pid,args --no-headers 2>/dev/null | awk -v pat="$pat" '$0 ~ pat && $0 !~ /awk/ {print $1}')
  if [ -z "$pids" ]; then echo "0.0"; return; fi
  hz=$(getconf CLK_TCK 2>/dev/null); hz=${hz:-100}
  for p in $pids; do v=$(sed 's/.*) //' "/proc/$p/stat" 2>/dev/null | awk '{print $12 + $13}'); t0=$((t0 + ${v:-0})); done
  sleep "$secs"
  for p in $pids; do v=$(sed 's/.*) //' "/proc/$p/stat" 2>/dev/null | awk '{print $12 + $13}'); t1=$((t1 + ${v:-0})); done
  awk -v d=$((t1 - t0)) -v hz="$hz" -v s="$secs" 'BEGIN{ if (d < 0) d = 0; printf "%.1f", d / hz / s * 100 }'
}

banner_done() {
  printf '\n%s結果已寫入%s %s\n' "$C_DIM" "$C_0" "$RESULT"
  printf '%s全部跑完後執行%s ~/maprun/verify/collect_data.sh %s打包回傳%s\n\n' \
         "$C_DIM" "$C_0" "$C_DIM" "$C_0"
}
