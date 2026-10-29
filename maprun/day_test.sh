#!/bin/bash
# 一鍵測試（★ 2026-10-26 車上 agent）：上機當天照提示做，結果全部自動記錄。不必開 VS Code。
#
#   筆電 cmd.exe：ssh user@192.168.137.106
#   進去之後打：  ~/welcoming_robot_ws/src/smartnav_ws/maprun/day_test.sh
#
# 流程（對照報告與 下週上車測試清單_20261023.md）：
#   1 開工檢查 V0 → 2 語音錯字率 → 3 舵機確認 → 4 故事線＋CPU＋整體延遲 → 5 導航新舊 A/B＋急停計時
#   → 6 LLM 十題 → 7 人臉×距離 → 8 操作步驟錄影 → 9 收工打包
# 每一步開始前會說明要做什麼：Enter＝開始、s＝跳過、x＝先離開（不收工）、q＝直接收工（會自動打包）。
# 做過的步驟記在 ~/welcoming_robot_ws/src/smartnav_ws/maprun/verify_out/day_<月日>/done.txt；斷線後再打一次同一個指令，會從沒做的地方接。
# 結果：同一個資料夾（每步的輸出、CSV、session.log），重點數字另外用 result.sh 登記（manual_results_<日期>.tsv）。
# ★ 會讓車子動的步驟要打 yes；平板急停隨時可按。轉向量測（第 3 步）例外：平板急停停不了，只能在這個視窗按 Ctrl-C。
# ★ 任何一步跑到一半按 Ctrl-C，只會停掉那一步、回到流程。
# ★ 背景記錄（CPU、整體延遲、急停計時）用 SIGTERM 停：bash 背景工作會忽略 Ctrl-C。
# 不要 set -u（source ROS 會靜默中止，見記憶 set-u-breaks-ros-source）。

# ★ 2026-10-29：程式在 repo 裡（~/welcoming_robot_ws/src/smartnav_ws/maprun，就是這支所在的資料夾），
#   資料照舊寫在 ~/maprun（verify_out、logs；不進 git）。
R="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
source "$R/env.sh" >/dev/null 2>&1
export PYTHONUNBUFFERED=1
M=/home/user/maprun
T=$R/tools_1020
D=${DAY_TEST_DIR:-$M/verify_out/day_$(date +%m%d)}
PACK_DIR=${DAY_TEST_PACK_DIR:-/home/user}
mkdir -p "$D"
LOG=$D/session.log
DONE=$D/done.txt
touch "$DONE"
INTR=0
trap 'INTR=1' INT   # 主選單的 Ctrl-C 只設旗標（訊息由 ask／步驟自己印，免得印兩次）
# ★ 2026-10-27：bash 5.2 的 read 被 Ctrl-C 打斷（有 trap）後會自己繼續等、不會回傳 → 「回到流程」從沒發生，
#   使用者只能 Ctrl-Z，留下一堆暫停的 day_test。改成每秒逾時一次看旗標：
#   ask 變數名 "提示"；Ctrl-C → 回傳 1（tty 是行緩衝，打到一半的字不會掉）。
ask() {
  local __v __rc
  INTR=0
  while read -r -t 0.02 __v; do :; done   # ★ 10/27：丟掉等待中多按的 Enter（只丟整行，打到一半的不會掉）
  printf '%s' "$2"
  while :; do
    read -r -t 1 __v; __rc=$?
    if [ "$__rc" -eq 0 ]; then printf -v "$1" '%s' "$__v"; return 0; fi
    [ "$INTR" = 1 ] && { echo; return 1; }
    [ "$__rc" -le 128 ] && return 1   # EOF
  done
}

STEPS=(v0 asr servo story navab llm face video wrap)
TITLES=("開工檢查 V0" "語音錯字率（報告第 6 節）" "舵機確認（清單 4-S）" "故事線＋CPU＋整體延遲（報告第 14 節）"
        "導航新舊 A/B＋急停計時（報告第 7 節）" "LLM 十題（報告第 8 節）" "人臉×距離（報告第 5 節）"
        "操作步驟錄影（報告第 9～12 節）" "收工：全部關閉＋打包")

log() { echo "$(date '+%H:%M:%S') $*" >> "$LOG"; }
say() { echo "$*"; log "$*"; }
rec() { "$T/result.sh" "$1" "$2" "$3" >/dev/null 2>&1; log "📝 $1 $2 $3"; echo "  📝 已記錄：$1 $2｜$3"; }
# ★ 10/27：以前只印最後一行（| tail -1）→ 開導航要 1～2 分鐘時畫面完全不動，看起來像當掉（使用者以為 V5、V6 卡死）。
#   現在逐行即時印（每行截 170 字）；過濾器不理 Ctrl-C（以前會噴 KeyboardInterrupt traceback），hmi_unit 結束它就結束。
hmi() { python3 -u "$T/hmi_unit.py" "$@" 2>&1 | python3 -u -c "import signal, sys
signal.signal(signal.SIGINT, signal.SIG_IGN)
for l in sys.stdin: print(l.rstrip()[:170], flush=True)"; }
confirm() { local a; ask a "$1 確定請打 yes："; [ "$a" = "yes" ]; }
# ★ 10/27：`| tee` 時按 Ctrl-C 會連 tee 一起殺掉 → 後面印的總表（例如 ASR 的 CER）沒寫進檔案；而且同名檔重跑就被蓋掉。
#   teeq＝不怕 Ctrl-C、附加寫入的 tee。
teeq() { (trap '' INT; tee -a "$1"); }
# ★ 2026-10-27（第二次）：步驟在跑指令時按 Ctrl-C 只會停掉那一個指令、步驟照樣往下走 → 使用者「Ctrl-C 沒辦法回到頁面」；
#   而且選單上 Ctrl-C＝跳過，連按幾次會一路跳到第 9 步自動收工（全部關閉＋打包）。
#   現在：每一步在子 shell 裡跑，Ctrl-C＝整步中斷、回到這一步的選單（不記成做完）；選單上 Ctrl-C＝離開（不收工）。
#   刻意用 Ctrl-C 結束「某個動作」的地方（ASR 提早結束錄音、人臉註冊完回來、故事線結束、轉向量測停車、A/B 中止、LLM 中止）
#   包在 soft_on／soft_off 裡：Ctrl-C 只結束那個指令（SOFT_INT=1），步驟繼續把結果記完。
STEP_TRAP='echo; echo "（已中斷這一步，回到選單）"; exit 130'
soft_on()  { SOFT_INT=0; trap 'SOFT_INT=1' INT; }
soft_off() { trap "$STEP_TRAP" INT; }
run_step() {   # $1＝步驟 key；回 0＝做完、3＝步驟自己說先不做、130＝Ctrl-C 中斷
  (
    trap "$STEP_TRAP" INT
    STEP_ABORT=0
    "step_$1"
    [ "$STEP_ABORT" = 1 ] && exit 3
    exit 0
  )
}

# ── 背景記錄 ────────────────────────────────────────────────────
bg_start() {   # $1＝標籤
  python3 "$T/load_monitor.py" --label "$1" --secs 7200 --interval 10 > "$D/load_$1.txt" 2>&1 &
  echo $! > "$D/.load.pid"
  python3 "$T/latency_logger.py" --out "$D/latency_$1" > "$D/latency_$1.txt" 2>&1 &
  echo $! > "$D/.latency.pid"
  log "背景記錄開始：CPU（$D/load_$1.txt）、整體延遲（$D/latency_$1）"
}
bg_stop() {
  local f
  for f in "$D/.load.pid" "$D/.latency.pid" "$D/.estop.pid"; do
    [ -f "$f" ] && kill -TERM "$(cat "$f")" 2>/dev/null
    rm -f "$f"
  done
  sleep 4
}

# ── 1 開工檢查 ──────────────────────────────────────────────────
# ★ 2026-10-26：能上網時，開機後一小時內會補跑系統自動更新（apt-daily-upgrade.timer 是 Persistent）。
#   更新中 CPU、語音數據會失真；裝到 HMI 用的函式庫時 needrestart 會重啟 HMI（10/26 那次 55 分鐘重啟 21 次），
#   而 HMI 是 KillMode=control-group：平板／這支腳本開的功能會跟著全部被關掉。只提醒、不擋。
apt_running() {
  systemctl show -p ActiveState --value apt-daily.service apt-daily-upgrade.service 2>/dev/null | grep -qxE 'activating|active|deactivating'
}
apt_pending() {   # 已下載、還沒裝的更新個數（unattended-upgrades 裝完會刪掉 .deb）。apt-get -s upgrade 要 18 秒，太慢
  local f b p v a q n=0
  for f in "${1:-/var/cache/apt/archives}"/*.deb; do
    [ -e "$f" ] || continue
    b=$(basename "$f" .deb); p=${b%%_*}; a=${b##*_}; v=${b#*_}; v=${v%_*}; v=${v//\%3a/:}
    [ "$a" = all ] && q=$p || q=$p:$a
    [ "$(dpkg-query -W -f='${Version}' "$q" 2>/dev/null)" = "$v" ] || n=$((n + 1))
  done
  echo "$n"
}
apt_soon() {   # V0 用：3 小時內排定的自動更新
  local nx now hm n
  nx=$(systemctl show -p NextElapseUSecRealtime --value apt-daily-upgrade.timer 2>/dev/null)
  case "$nx" in [A-Z][a-z][a-z]\ 2*) nx=$(date -d "${nx% *}" +%s 2>/dev/null) ;; *) return ;; esac   # 去掉時區縮寫（CST 會被當成美國中部）
  now=$(date +%s)
  [ -n "$nx" ] && [ "$nx" -gt "$now" ] && [ $((nx - now)) -lt 10800 ] || return
  hm=$(date -d "@$nx" +%H:%M); n=$(apt_pending)
  if [ "$n" -gt 0 ]; then
    say "⚠ 有 $n 個系統更新已下載、約 $hm 會自動安裝，裝完會重啟 HMI（平板斷約 1 分鐘、開著的功能會全部被關掉）"
    say "  建議現在先裝掉再開始：另開一個 SSH 視窗打 sudo unattended-upgrade（2～3 分鐘，平板會斷一下）"
  else
    say "（系統約 $hm 會檢查更新；目前沒有已下載待裝的，有新的才會裝）"
  fi
}
apt_wait() {   # 等更新跑完、HMI 回來（Ctrl-C＝不等了）
  say "等系統更新跑完（每 10 秒看一次；Ctrl-C＝不等了）…"
  while apt_running; do sleep 10 || return; echo -n "."; done
  until curl -sf -m 3 -o /dev/null http://127.0.0.1:8080/; do sleep 5 || return; echo -n "."; done
  echo; say "✓ 系統更新跑完了、HMI 在線"
}

step_v0() {
  if ps -eo args 2>/dev/null | grep -q "[.]vscode-server"; then
    say "⚠ VS Code 還連在這台車上（吃 1.5～3 GB 記憶體、CPU 35～50 點）——量測會失真。請在筆電把 VS Code 關掉再繼續。"
  fi
  timedatectl 2>/dev/null | grep -q "synchronized: yes" || say "⚠ 時鐘還沒同步，log 時間可能跳；等一分鐘再開始比較好"
  apt_soon
  # ★ 10/27：src 的 .py 改了會經 symlink 直接生效，但**已經在跑的 HMI 不會**（平板遙控抖動、倒車方向的修正都在 HMI 裡）。
  #   verify_0 的「HMI 跑的是新程式」只比 colcon build 的時間，看不出這種情況 → 這裡比原始碼的修改時間。
  local hpid hstart newer
  hpid=$(systemctl show -p MainPID --value smartnav-hmi 2>/dev/null)
  if [ -n "$hpid" ] && [ "$hpid" != 0 ] && [ -d "/proc/$hpid" ]; then
    hstart=$(stat -c %Y "/proc/$hpid")
    newer=$(find /home/user/welcoming_robot_ws/src/smartnav_hmi/smartnav_hmi -name '*.py' -newermt "@$hstart" 2>/dev/null | head -3)
    [ -n "$newer" ] && say "⚠ HMI 比程式更新還早啟動（還在跑舊版）：平板「電源」→ 重新開機，或 SSH 打 sudo systemctl restart smartnav-hmi（等 1 分鐘）"
  fi
  if curl -s -m 3 http://192.168.137.1:11434/api/tags >/dev/null; then say "✓ 筆電 Ollama 連得到"
  else say "⚠ 筆電 Ollama 連不到（LLM 那幾步會失敗）：筆電開 Ollama、連同一個熱點"; fi
  # ★ 2026-10-27：電池電壓要底盤節點在跑才讀得到——第一次上機沒開底盤，V0 印「讀不到」（WARN）卻記成全過。
  #   先開「底盤＋雷達＋IMU」（已經開著就不動）；剛開的話 IMU 要靜止校準約 20 秒，這段時間不要碰車。
  # ★ 10/27 12:29：開機後馬上跑 → HMI 還沒起來（服務要等 60 秒）→ 底盤開不了 → 電池讀不到。先等 HMI。
  if ! curl -sf -m 3 -o /dev/null http://127.0.0.1:8080/; then
    say "等 HMI 開好（開機後約 1 分半才會好）…"
    for _ in $(seq 1 40); do sleep 3; curl -sf -m 3 -o /dev/null http://127.0.0.1:8080/ && break; done
  fi
  say "開底盤＋雷達＋IMU（電池電壓要底盤在跑才讀得到）；剛開的話約 20 秒 IMU 校準，這段時間不要碰車……"
  hmi start sensors
  sleep 5   # 行程出現 ≠ 已經在發 /PowerVoltage
  local nf bad vf="$D/v0_$(date +%H%M%S).txt"   # ★ 10/27：每次一個檔（以前 v0.txt 每跑一次就蓋掉上一次）
  "$R/verify/verify_0_preflight.sh" 2>&1 | teeq "$vf"
  cp "$vf" "$D/v0.txt"
  # WARN 也算沒過（電池讀不到、時鐘沒同步、找不到編譯產物）
  bad=$(sed -E 's/\x1b\[[0-9;]*m//g' "$D/v0.txt" | grep -E "^\s+(FAIL|WARN)" | sed -E 's/^\s+//; s/\s{2,}/ /g')
  nf=$(printf '%s' "$bad" | grep -c .)
  if [ "$nf" -eq 0 ]; then rec 1 PASS "V0 全過"
  else
    rec 1 WARN "V0 有 $nf 項沒過：$(printf '%s' "$bad" | paste -sd'/' -)"
    say "★ 有沒過的項目（FAIL／WARN）：先處理（例如 Astra S 拔插一次、檢查電池）再繼續；或照樣往下做。"
  fi
}

# ── 2 語音錯字率 ────────────────────────────────────────────────
step_asr() {
  say "會先關 LLM（不然機器人會回話、甚至開車），再開語音單元。站在麥克風前 60 cm，照稿唸，每句停 3 秒，錄 180 秒。"
  hmi stop llm
  hmi start asr_chain
  sleep 10
  # ★ 10/27：收音緩衝預設改 0.2 秒（"high" 在這個麥克風實際只有 32 ms）——印出實際值，確認有生效
  grep -h -o "收音緩衝.*" "$(ls -t /home/user/.smartnav/logs/system_control/asr_chain_*.log 2>/dev/null | head -1)" 2>/dev/null | tail -1 | sed "s/^/  /"
  # ★ 10/28：印出麥克風實際增益（應該是 60；48＝原廠值、VAD 幾乎不觸發）。10/27 有 4/10 句 VAD 根本沒觸發，要知道當時的增益
  local card; card=$(arecord -l 2>/dev/null | grep -i -m1 -E 'astra|orbbec' | sed -E 's/^card ([0-9]+).*/\1/')
  [ -n "$card" ] && amixer -c "$card" sget 'Mic',0 2>/dev/null | grep -m1 -oE 'Capture [0-9]+ \[[0-9]+%\]' | sed "s/^/  麥克風增益（應為 60）：/"
  say "★ 10/28 修了「句首被吃掉」的一個成因（音訊話題佇列 5→40：觸發時的 0.48 秒預錄以前有六成被丟掉）——這一輪就是要看句首錯有沒有消失。唸的時候正常音量、別離太遠"
  python3 "$T/asr_cer.py" --print-script
  ask __x "準備好就按 Enter 開始錄（Ctrl-C＝這一步先不做）…" || { STEP_ABORT=1; return; }
  local f csv strict loose mark="$D/.asr_start"
  : > "$mark"
  soft_on; python3 "$T/asr_cer.py" 2>&1 | teeq "$D/asr_cer_console_$(date +%H%M).txt"; soft_off   # Ctrl-C＝提早結束錄音
  # ★ 10/27：以前抓「最新的 asr_cer_*.txt」——這次沒寫出 .txt 時抓到 10/20 的舊檔，把 42.1% 記成今天的結果（今天實際 32.9%）。
  #   現在只認這次之後寫的檔；沒有 .txt 就用這次的 CSV 離線重算。
  f=$(find "$M/verify_out" -maxdepth 1 -name 'asr_cer_[0-9]*.txt' -newer "$mark" 2>/dev/null | sort | tail -1)
  if [ -z "$f" ]; then
    csv=$(find "$M/verify_out" -maxdepth 1 -name 'asr_cer_[0-9]*.csv' ! -name '*_recalc*' -newer "$mark" 2>/dev/null | sort | tail -1)
    if [ -n "$csv" ]; then
      say "（這次沒寫出總表，用這次的 $(basename "$csv") 離線重算）"
      python3 "$T/asr_cer.py" --from-csv "$csv" > /dev/null 2>&1 && f="${csv%.csv}_recalc.txt"
    fi
  fi
  [ -n "$f" ] && [ -f "$f" ] && cp -p "$f" "$D/"
  strict=$(grep -oE "整體 CER（嚴格）：[^=]+= [0-9.]+%" "$f" 2>/dev/null | grep -oE "[0-9.]+%$")
  loose=$(grep -oE "整體 CER（寬鬆[^）]*）：[^=]+= [0-9.]+%" "$f" 2>/dev/null | grep -oE "[0-9.]+%$")
  if [ -z "$loose" ]; then rec 2-1 WARN "沒算出 CER，見 $f"; return; fi
  if python3 -c "import sys; sys.exit(0 if float(sys.argv[1].rstrip('%')) < 35 else 1)" "$loose"; then
    rec 2-1 PASS "CER 寬鬆 $loose、嚴格 $strict（$(basename "$f")）"
  else
    rec 2-1 WARN "CER 寬鬆 $loose、嚴格 $strict 偏高（靜態基準 22~26%）（$(basename "$f")）"
  fi
}

# ── 3 舵機確認 ──────────────────────────────────────────────────
trim_set() {
  local J=/home/user/.smartnav/steering_trim.json v
  echo "目前：$(cat "$J" 2>/dev/null || echo '（沒有檔案）')"
  ask v "轉向量測印的「trim_rad_per_m 設 … 可抵消」那個值（-0.30 ~ +0.30；Enter＝不改）："
  [ -z "$v" ] && { echo "沒改"; return; }
  python3 -c "import sys; v=float(sys.argv[1]); sys.exit(0 if -0.30 <= v <= 0.30 else 1)" "$v" 2>/dev/null \
    || { echo "✗ 不是 -0.30~+0.30 的數字，沒改"; return; }
  cp -p "$J" "$J.bak_$(date +%Y%m%d_%H%M%S)" 2>/dev/null
  python3 - "$J" "$v" <<'EOF'
import datetime, json, sys
json.dump({"trim_rad_per_m": float(sys.argv[2]), "saved_at": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
           "updates": 0, "note": "day_test 手動設定"}, open(sys.argv[1], "w"))
EOF
  rec 4-S PASS "trim 改成 $v（舊檔有備份）"
}
steer_run() {   # $1＝before／after
  hmi stop nav
  hmi start sensors
  sleep 12
  python3 "$R/steer_asym_check_cc.py" --dry
  confirm "★ 車子要動了（直走＋左右打滿；地上、四周 1.5 m 淨空；平板急停停不了，只能在這裡 Ctrl-C）。人站在旁邊了嗎？" || return
  soft_on; python3 "$R/steer_asym_check_cc.py" 2>&1 | teeq "$D/steer_$1.txt"; soft_off   # Ctrl-C＝停車（平板急停停不了它）
  local r tr
  r=$(grep -E "左右不對稱：" "$D/steer_$1.txt" | head -1 | sed -E 's/^\s+//')
  tr=$(grep -oE "trim_rad_per_m 設 [-+0-9.]+" "$D/steer_$1.txt" | head -1)
  rec 4-S PASS "轉向量測（$1）：${r:-沒算出半徑}；${tr:-沒算出 trim}"
}
step_servo() {
  # ★ 2026-10-27：第一次上機這一步沒開底盤 → 平板遙控的指令沒人收，舵機一直停在中間。
  #   改成先開底盤，並照使用者要求用鍵盤（teleop_twist_keyboard）直接發 /cmd_vel：慢速前進＋最大轉向。
  #   speed 0.15／turn 1.0 → 轉彎半徑 0.15 m，遠小於車子能做到的，韌體會打到底；0.15 也高於死區 0.085。
  hmi stop nav
  hmi start sensors
  cat <<'EOF'
先只看、不轉旋鈕（約 5 分鐘）：
  1. 整台車架高（後輪也要離地，不然車會跑掉），四周清空，手遠離輪子。
     （底盤如果是剛剛才開的，先等 20 秒讓 IMU 校準完再搬車）
  2. 另開一個 SSH 視窗（跟這個一樣連進來），打這兩行，用鍵盤遙控：
       source ~/welcoming_robot_ws/src/smartnav_ws/maprun/env.sh
       ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.15 -p turn:=1.0
     按住 u＝慢慢前進＋打滿左；按住 o＝慢慢前進＋打滿右；k 或空白鍵＝停；Ctrl-C＝結束（會自動送停止）。
     ★ 車速 0 時韌體會把舵機扳正，所以一定要用 u／o（邊前進邊轉）；j、l 是原地轉，舵機會回正。
     ★ 平板急停停不了鍵盤：要停就在那個視窗按 k／空白鍵或 Ctrl-C。平板不要同時開遙控頁。
  3. 看哪個輪子碰到車身，記下 OLED 第 5 行的 SERVO；再打滿右，同樣記。左右各拍一張照片（輪子＋OLED）。
  4. 做完在鍵盤視窗按 Ctrl-C，再回這個視窗回答下面的問題。
EOF
  local side sl sr a s0
  ask side "哪一邊打滿會碰到車身？（L＝左、R＝右、B＝兩邊、N＝都沒有）："
  ask sl "打滿左時 OLED 的 SERVO（例 1890）："
  ask sr "打滿右時 OLED 的 SERVO（例 1174）："
  rec 4-S PASS "碰車身＝${side:-?}；打滿左 SERVO ${sl:-?}、打滿右 ${sr:-?}"
  case "$side" in
    L|l)
      say "→ 左碰、右打不到底：照手冊 D 節，把靜止 SERVO 從 1666 調到 1595~1610。"
      confirm "先跑一次轉向量測（調前）？" && steer_run before
      ask a "車放回架上，轉右上「微調旋鈕」讓靜止 SERVO 到 1595~1610；調好按 Enter（不調輸入 n）："
      [ "$a" = "n" ] && return
      ask s0 "調完後靜止時的 SERVO："
      rec 4-S PASS "旋鈕調到靜止 SERVO ${s0:-?}"
      confirm "跑轉向量測（調後）？" && steer_run after
      say "★ 把調後量測印的 trim 寫回去（這一步一定要做，否則舊 trim −0.218 會讓車往右偏）："
      trim_set
      say "變差的話：把 SERVO 轉回 1666，再跑一次這一步、trim 填 -0.218。" ;;
    R|r) say "→ 右碰、左打不到底：旋鈕不要動（左邊是被韌體上限 1890 擋住）。已記錄。" ;;
    *)   say "→ 兩邊都碰到或都打不到底：先不要動，照片給筆電。已記錄。" ;;
  esac
}

# ── 4 故事線＋CPU＋整體延遲 ──────────────────────────────────────
# ★ 2026-10-27：第一次上機沒定位就跑故事線 → 導航亂跑。導航起來後、開跑前先定位到一個已存地點（車不會動）。
localize_step() {
  local rc a
  cat <<'EOF'
★ 先定位（車不會動）：車子停哪都可以——出現清單時直接按 Enter，它會拿雷射比整張地圖自己找位置（約 3~10 秒）。
   說「不確定」時：把車移到看得到比較多牆／家具的地方再試；或輸入地點編號（車要停在那個地點、車頭朝存點方向）。
EOF
  while :; do
    python3 "$T/localize_here.py"; rc=$?
    log "定位 rc=$rc（0＝OK、2＝吻合度偏低／方向不對、1＝失敗）"
    [ "$rc" -eq 0 ] && return 0
    ask a "Enter＝再定位一次、s＝不管它繼續、q＝這一步先不做：" || a=q   # Ctrl-C＝這一步先不做
    case "$a" in
      s|S) log "⚠ 定位沒過就繼續"; return 0 ;;
      q|Q) log "定位沒過，這一步先不做"; STEP_ABORT=1; return 1 ;;
    esac
  done
}

step_story() {
  local sc key=demo n=0 t0 a line tag since paused resumed warn st
  # ★ 10/27：預設改省電版——完整版 CPU 閒置 1%、導航控制迴圈掉到 1.7 Hz（設定 20 Hz）、AMCL 丟雷射；省電版人臉沒人時每 4 秒才推論
  #   （帶位途中客人在車後面，鏡頭前沒人 → 正好把 CPU 讓給導航）。
  # ★ 10/28：預設改回完整版。HMI 現在「導航中自動暫停人臉相機、到點恢復」（smartnav_hmi/modules/system_control/nav_camera.py）：
  #   10/27 完整版帶位去櫃台 289 秒逾時，人臉推論其實已經停了，剩下吃 CPU 的是相機＋JPEG（約 80%，以單核計）；
  #   10/20 對照：全開沒人臉 → 閒置 1%，再關相機 → 42%。今天要驗證的就是完整版能不能帶位成功。
  ask sc "哪一種故事線？ 1＝完整（預設：驗證導航中暫停相機）  2＝人臉省電："
  [ "$sc" = "2" ] && key=demo_eco
  tag="${key}_$(date +%H%M)"   # ★ 10/27：重跑不要蓋掉上一次的 CPU／延遲紀錄
  say "開情境 $key（等單元起來，最多約 2 分鐘）…"
  hmi scenario "$key" sensors nav llm
  say "等 LLM 暖機 20 秒…"; sleep 20
  localize_step || return
  since=$(date '+%Y-%m-%d %H:%M:%S')
  bg_start "$tag"
  say "背景開始記錄 CPU 與整體延遲。照平時 demo 跑 3~5 輪：站到鏡頭前 1 m → 對話 → 說「帶我去 X」→ 站到車旁邊、開動後跟上 → 到達。★ 車會動：有人拿平板急停。"
  # ★ 10/27 自動化：每一輪照導航 log 自動切輪、自動登記，不用每輪按 Enter（以前多按的 Enter 會把第 1 輪記成空的）
  say "每一輪會自動偵測、自動記錄；全部做完按一次 Enter。"
  while read -r -t 0.02 __x; do :; done   # 丟掉前面多按的 Enter，免得監看一開始就結束
  soft_on; python3 "$T/story_watch.py" --since "$(date +%s)" --tag "$tag"; soft_off   # Enter 或 Ctrl-C＝結束這幾輪
  bg_stop
  rec 3-1 PASS "CPU（$tag）：$(grep -E "^══" "$D/load_$tag.txt" | tail -1)"
  # ★ 10/28：導航中暫停相機有沒有真的發生（看 HMI 自己的 log；平板上相機那格也會顯示「導航中自動暫停」）
  paused=$(journalctl -u smartnav-hmi --since "$since" --no-pager -o cat 2>/dev/null | grep -c '暫停相機（人臉模式）')
  resumed=$(journalctl -u smartnav-hmi --since "$since" --no-pager -o cat 2>/dev/null | grep -c '恢復相機（人臉模式）')
  warn=$(journalctl -u smartnav-hmi --since "$since" --no-pager -o cat 2>/dev/null | grep -c -E '停不了相機|開不了相機')
  st=PASS; { [ "$paused" -eq 0 ] || [ "$warn" -gt 0 ]; } && st=WARN
  rec 3-2 "$st" "導航中暫停相機（$tag）：暫停 $paused 次、恢復 $resumed 次、失敗警告 $warn 次（暫停 0 次＝沒生效：HMI 是不是舊版？）"
  rec 14 PASS "整體延遲（$tag）：$(sed -n '1,12p' "$D/latency_$tag/summary.txt" 2>/dev/null | sed -E 's/^\s+//' | paste -sd'|' -)"
  say "詳細：$D/latency_$tag/turns.csv（每一輪）、$D/load_$tag.txt"
}

# ── 5 導航新舊 A/B＋急停計時 ─────────────────────────────────────
nav_params() {
  local R V p FB
  if [ "$1" = 新 ]; then R=1.4; V=true; FB="[1.2, 0.95]"; else R=0.95; V=false; FB="[0.0]"; fi
  say "切到【$1】：規劃半徑 $R、四項 $V（Smac 重建查找表約 5 秒）"
  ros2 param set --timeout 15 /planner_server GridBased.minimum_turning_radius $R >/dev/null 2>&1 || say "⚠ 改規劃半徑失敗（導航沒開？）"
  for p in plan_candidates lookahead_interpolate xte_segment xte_integral; do
    ros2 param set --timeout 15 /path_teach_cc_node $p $V >/dev/null 2>&1 || say "⚠ 改 $p 失敗"
  done
  ros2 param set --timeout 15 /path_teach_cc_node plan_fallback_radii "$FB" >/dev/null 2>&1 || say "⚠ 改 plan_fallback_radii 失敗"
  sleep 5
}
step_navab() {
  local mode a pa pb n rc
  # ★ 2026-10-27 自動化（使用者：「這些都沒有自動化的方式?因為這樣很難測跟計算」）：以前每趟要在平板按帶位、
  #   到了回來按 Enter、急停要自己找時機按。現在 nav_ab_auto.py 自己發導航、等結束、每趟寫一列；最後一趟自動按急停。
  say "新舊導航 A/B（自動）：車子會自己在兩個地點之間來回，新（規劃半徑 1.40＋四項開）、舊（0.95＋四項關）各跑 4 趟，每趟自動記錄；最後一趟途中自動按急停量反應時間。"
  say "★ 車會自己連續跑：路線淨空、旁邊一定有人拿平板急停。跑到一半按 Ctrl-C＝立刻急停並結束這一步。"
  confirm "準備好了嗎？" || { STEP_ABORT=1; return; }
  # ★ 10/27：前一步 demo 的人臉／相機／語音／LLM 還開著 → load 25+，參數改不動、A/B 也不公平 → 先關掉
  hmi stop camera face user_auth bank_reception llm asr_chain
  hmi start sensors nav:localization   # ★ 10/27：只寫 nav 會被拒絕（要指定啟動方式）
  sleep 5
  localize_step || return
  ask a "來回的兩個地點（Enter＝櫃台 大廳；或輸入「地點A 地點B」，中間空一格）：" || { STEP_ABORT=1; return; }
  read -r pa pb _ <<< "${a:-櫃台 大廳}"
  [ -n "$pb" ] || { say "要兩個地點"; STEP_ABORT=1; return; }
  ask n "每組幾趟（Enter＝4；偶數＝兩組都從「$pa」出發、最後回到「$pa」）：" || n=4
  case "$n" in ''|*[!0-9]*) n=4 ;; esac
  python3 "$T/estop_timing.py" --limit 1.3 --out "$D/estop_timing.tsv" >> "$D/estop_timing.txt" 2>&1 &
  echo $! > "$D/.estop.pid"
  rc=0
  soft_on   # Ctrl-C＝急停並結束 A/B（nav_ab_auto 自己會送急停）；SOFT_INT 一起看，參數切換中按也不會再開車
  for mode in 新 舊; do
    nav_params "$mode"
    [ "$SOFT_INT" = 1 ] && { rc=130; break; }
    say "【$mode】先開回「$pa」（這趟不記錄）…"
    python3 "$T/nav_ab_auto.py" --goto "$pa"; rc=$?
    [ "$rc" -eq 130 ] || [ "$SOFT_INT" = 1 ] && { rc=130; break; }
    python3 "$T/nav_ab_auto.py" --a "$pa" --b "$pb" --trips "$n" --tag "$mode" --tsv "$D/nav_ab.tsv"; rc=$?
    [ "$rc" -eq 130 ] || [ "$SOFT_INT" = 1 ] && { rc=130; break; }
  done
  nav_params 新   # ★ 10/27 傍晚 A/B 新 4/4、舊 0/4 → 預設＝新（1.40＋四項開），A/B 做完切回預設
  if [ "$rc" -ne 130 ]; then
    say "最後一趟：開往「$pb」，途中自動按急停（量急停反應時間）…"
    python3 "$T/nav_ab_auto.py" --a "$pa" --b "$pb" --estop-trip --tag 急停 --tsv "$D/nav_ab.tsv"
    sleep 3   # 等 estop_timing 量完
  else
    say "已中止（已完成的趟次都有記錄）"
  fi
  soft_off
  bg_stop
  python3 - "$D/nav_ab.tsv" <<'EOF' | tee "$D/nav_ab_summary.txt"
import csv, statistics, sys
try:
    rows = list(csv.DictReader(open(sys.argv[1], encoding="utf-8"), delimiter="\t"))
except OSError:
    rows = []
for tag in ("新", "舊"):
    rs = [r for r in rows if r["標籤"] == tag]
    if not rs:
        continue
    ok = [r for r in rs if r["結果"] == "成功"]
    secs = [float(r["秒"]) for r in ok]
    p95 = [float(r["偏離p95cm"]) for r in rs if r["偏離p95cm"]]
    print(f"{tag}：成功 {len(ok)}/{len(rs)}"
          + (f"、到達中位 {statistics.median(secs):.1f} s" if secs else "")
          + (f"、偏離 p95 中位 {statistics.median(p95):.1f} cm" if p95 else ""))
EOF
  rec 4-0b PASS "$(tr '\n' ' ' < "$D/nav_ab_summary.txt")（明細 $D/nav_ab.tsv）"
  if [ -s "$D/estop_timing.tsv" ]; then
    rec 2-4 PASS "急停計時：$(tail -n +2 "$D/estop_timing.tsv" | awk -F'\t' '{print $(NF-1), $NF}' | tr '\n' ' ')"
  fi
}

# ── 6 LLM 十題 ──────────────────────────────────────────────────
step_llm() {
  say "銀行十題（每題前清記憶）。★ 第②③④⑩題會真的帶位，而且不會再停下來問（★ 10/27 自動化）：車旁有人拿平板急停、路線淨空。"
  say "語音輸入要先關（不然麥克風雜音會餵進 LLM）；做完會開回來。"
  confirm "車子會自己帶位 4 次（櫃檯、貴賓室、洗手間、大廳），車旁有人拿急停了嗎？" || { STEP_ABORT=1; return; }
  hmi stop asr_chain
  hmi start sensors nav:localization llm   # ★ 10/27：直接從這一步開始時導航可能沒開
  sleep 5
  localize_step || return
  soft_on; python3 "$T/llm10.py" --clear-each --auto-nav --out "$D/llm10_$(date +%H%M).csv" 2>&1 | teeq "$D/llm10_console.txt"; soft_off   # Ctrl-C＝中止（已答的照常存）
  rec 7 PASS "LLM 十題的答案與延遲在 $(ls -t "$D"/llm10_*.csv 2>/dev/null | head -1)（對錯由人判）"
  hmi start asr_chain
}

# ── 7 人臉×距離 ─────────────────────────────────────────────────
step_face() {
  local a dist
  say "會切到「測人臉辨識／註冊」情境：導航會關掉（導航中人臉不推論）。"
  confirm "可以關導航嗎？" || return
  hmi scenario all_off
  sleep 8
  hmi scenario face camera face user_auth
  sleep 15
  ask a "要先多角度重新註冊嗎？要的話輸入 y，再去平板按「開始註冊」（註冊完按 Ctrl-C 回來）："
  if [ "$a" = "y" ]; then soft_on; python3 "$T/face_test.py" --watch-registration 2>&1 | teeq "$D/face_test.txt"; soft_off; fi   # 註冊完按 Ctrl-C 回來
  for dist in 0.5m 1.0m 1.5m 2.0m; do
    ask a "已註冊的人站到 $dist、正對鏡頭，按 Enter 開始 20 秒（s＝跳過這個距離）："
    [ "$a" = "s" ] && continue
    python3 "$T/face_test.py" --label "$dist" --secs 20 --expect-registered 2>&1 | teeq "$D/face_test.txt"
  done
  ask a "換一個沒註冊的人站 1 m，按 Enter 開始 20 秒（s＝跳過）："
  [ "$a" != "s" ] && python3 "$T/face_test.py" --label stranger --secs 20 2>&1 | teeq "$D/face_test.txt"
  python3 "$T/face_test.py" --summary 2>&1 | tee "$D/face_summary.txt"
  rec 4-3 PASS "人臉×距離總表在 $D/face_summary.txt"
}

# ── 8 操作步驟錄影 ───────────────────────────────────────────────
step_video() {
  cat <<'EOF'
報告第 9～12 節要用的畫面（手機錄影或平板截圖）：
   9  管理者：人臉註冊（平板 → 開始註冊 → 完成）
  10  管理者：建圖＋標地點（平板建圖 → 遙控繞一圈 → 存圖 → 地圖頁標地點）
  11  使用者：語意操作（對機器人說「帶我去櫃台」→ 車子出發）
  12  使用者：前端直接執行（迎賓頁點「櫃台」→ 確認 → 出發）
EOF
  local a
  ask a "錄完了嗎？（y＝都錄了、其他＝還沒）："
  if [ "$a" = "y" ]; then rec 9-12 PASS "操作步驟已錄"; else rec 9-12 未做 "操作步驟錄影"; fi
}

# ── 9 收工 ──────────────────────────────────────────────────────
step_wrap() {
  local stamp out
  bg_stop
  hmi scenario all_off
  "$T/result.sh" show > "$D/results_today.txt" 2>&1
  stamp=$(date +%m%d_%H%M)
  out=$PACK_DIR/day_test_${stamp}.tar.gz
  (cd /home/user && find maprun/verify_out maprun/logs .smartnav/logs/system_control -type f \
      -newermt "$(date +%Y-%m-%d) 00:00" 2>/dev/null | grep -vE "secrets|face_database|\.pid$" > "$D/.files.txt")
  tar czf "$out" -C /home/user -T "$D/.files.txt" 2>/dev/null
  say "✓ 今天的資料：$out（$(tar tzf "$out" 2>/dev/null | wc -l) 檔，$(du -h "$out" | cut -f1)）"
  echo "筆電 cmd.exe 貼這一行："
  echo "scp user@192.168.137.106:$out \"%USERPROFILE%\\Desktop\\專題\\backups\\研究數據\\$(basename "$out")\""
  echo "今天登記的結果："
  cat "$D/results_today.txt"
}

# ── 主流程 ──────────────────────────────────────────────────────
main() {
  local i g start mk rc
  echo "════ 一鍵測試（$(date '+%m/%d %H:%M')；紀錄在 $D）════"
  for i in "${!STEPS[@]}"; do
    mk="  "; grep -qx "${STEPS[$i]}" "$DONE" && mk="✓ "
    echo "  $mk$((i + 1)). ${TITLES[$i]}"
  done
  ask start "從第幾步開始？（Enter＝第一個還沒做的；Ctrl-C＝離開）：" || { say "離開（沒有收工）"; return; }
  if [ -z "$start" ]; then
    start=${#STEPS[@]}
    for i in "${!STEPS[@]}"; do grep -qx "${STEPS[$i]}" "$DONE" || { start=$((i + 1)); break; }; done
  fi
  case "$start" in ''|*[!0-9]*) start=1 ;; esac
  for ((i = start - 1; i < ${#STEPS[@]}; i++)); do
    echo
    echo "════ 第 $((i + 1)) 步：${TITLES[$i]} ════"
    if [ "${STEPS[$i]}" != wrap ]; then
      apt_running && say "⚠ 系統正在背景自動更新：數據會失真；若重啟 HMI，平板斷約 1 分鐘、開著的功能會全部被關掉。建議輸入 w 等它跑完再開始"
      ask g "Enter＝開始、s＝跳過、x＝先離開（不收工）、q＝直接收工：" || g=x   # ★ 10/27：Ctrl-C＝離開（以前＝跳過，連按會跳到收工）
      case "$g" in
        s|S) log "跳過 ${TITLES[$i]}"; continue ;;
        x|X) say "先離開（沒有收工）。要重做某一步：再打 ~/welcoming_robot_ws/src/smartnav_ws/maprun/day_test.sh，問從第幾步開始時輸入那一步的號碼"; return ;;
        q|Q) step_wrap; return ;;
        w|W) apt_wait ;;   # 每一步自己會開需要的功能，所以等完直接開始這一步就好
      esac
    fi
    log "開始 ${TITLES[$i]}"
    run_step "${STEPS[$i]}"; rc=$?
    if [ "$rc" -eq 130 ]; then   # Ctrl-C：收掉這一步開的背景記錄，回到同一步的選單
      bg_stop
      log "中斷 ${TITLES[$i]}（Ctrl-C；沒有記成做完）"
      [ "${STEPS[$i]}" = wrap ] && { say "收工被中斷：要打包就再打一次 day_test.sh、輸入 9"; return; }
      i=$((i - 1)); continue
    fi
    # ★ 10/27：步驟自己說「先不做」（定位沒過、錄音前 Ctrl-C）→ 不記成做完，回到同一步的提示（可 s／x／q）
    if [ "$rc" -eq 3 ]; then log "沒做完 ${TITLES[$i]}（沒有記成做完）"; i=$((i - 1)); continue; fi
    grep -qx "${STEPS[$i]}" "$DONE" || echo "${STEPS[$i]}" >> "$DONE"
    log "完成 ${TITLES[$i]}"
  done
}

main
