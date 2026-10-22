#!/usr/bin/env bash
# 轉向分級曲率實驗（疑點 C）的一鍵腳本 —— 交接文件 §5-4 ＋ ~/car_docs/轉向診斷_20261015.md「⑤ 上車 30 分鐘實驗」。
#
# ★ 這支會讓車子動（直接對 /cmd_vel 發指令，繞過 trim 與防撞）。階段指令、速度、秒數與文件**完全相同**：
#   前進／倒車 0.15 m/s ×6 s；左右滿舵 0.10 m/s ×6 s；分級曲率 κ=0.2…1.068 @0.10 m/s ×5 s；
#   倒車打舵 ×6 s；slalom 8×2 s；最後走完整鏈路 chain_left（/cmd_vel_smoothed）。
#   車子總共前進／倒車／繞弧的範圍約 2~3 m；要求前後左右各 ≥ 2 m 空曠。
#
# 判讀準則已在實驗前凍結：~/maprun/verify_out/steer_criteria_frozen_20261020.md（先別改它）。
#
# 前提（腳本會檢查，不夠就拒絕開始）：
#   - 平板「系統開關」已開：底盤＋雷達＋IMU，以及導航堆疊（定位模式：EKF、AMCL、steering_trim、collision_monitor、cmd_vel_floor 都要在）
#   - 電池 ≥ 23.5 V
#   - 你人在車旁，**手放在這個終端機的 Ctrl-C 上**。
#     ★★ 平板「緊急停止」與 ~/maprun/estop_cc.sh **都停不了這支腳本**（2026-10-20 讀 HMI 原始碼確認）：
#        腳本直接對 /cmd_vel 以 20 Hz 發指令（在 collision_monitor 與平板急停的下游），
#        平板急停只對 cmd_vel_trimmed 發**一則**零速再取消導航作業，蓋不過它每秒 20 則的指令。
#        真要停：在這個終端機按 Ctrl-C（停止發送並補送零速）；或把車抓住／關底盤電源。
#   - 感測器啟動後已過 20 秒（IMU 校準期間車不能動）
#
# 用法：
#   ~/maprun/tools_1020/run_steer_experiment.sh --dry-run      # 只印計畫（不碰車、不需要 ROS）
#   ~/maprun/tools_1020/run_steer_experiment.sh                # 完整實驗
#   ~/maprun/tools_1020/run_steer_experiment.sh --only graded --suffix _b   # 只重錄 8 個分級階段，標記加 _b（準則裡「離門檻 ≤0.03 要重錄」用）
#
# 輸出（都在家目錄，collect_data.sh --all 會收）：
#   ~/steer_diag_<HHMM>.csv    原始錄製        ~/steer_diag_<HHMM>.json   分析器的 JSON
#   ~/steer_diag_<HHMM>_analysis.md   人看的分析（含錄製的訊息數、電池前後、分析器全文）
set -u
DRY=0; ONLY="all"; SUFFIX=""
while [ $# -gt 0 ]; do
  case "$1" in
    --dry-run) DRY=1 ;;
    --only) ONLY="${2:-all}"; shift ;;
    --suffix) SUFFIX="${2:-}"; shift ;;
    -h|--help) sed -n '2,28p' "$0"; exit 0 ;;
    *) echo "不認得的參數：$1（--help 看用法）"; exit 2 ;;
  esac; shift
done
case "$ONLY" in all|graded) ;; *) echo "--only 只能是 all 或 graded"; exit 2 ;; esac

TOOLS="$HOME/maprun/tools_1015"
STAMP="$(date +%H%M)"
CSV="$HOME/steer_diag_${STAMP}.csv"; JSON="$HOME/steer_diag_${STAMP}.json"
MD="$HOME/steer_diag_${STAMP}_analysis.md"; REC_LOG="$HOME/steer_diag_${STAMP}.rec.log"
REC_PID=""

# ── 階段動作（與文件一字不差；dry-run 只印）──────────────────────
mark()  { if [ $DRY = 1 ]; then echo "    [標記] $1$SUFFIX"; else ros2 topic pub --once /steer_diag_marker std_msgs/msg/String "{data: $1$SUFFIX}" >/dev/null 2>&1; fi; }
drive() { # drive <x> <z> <秒>   以 20 Hz 發 /cmd_vel
  if [ $DRY = 1 ]; then echo "    [發送] /cmd_vel  linear.x=$1 angular.z=$2  共 $3 秒（約 $(awk -v v="$1" -v s="$3" 'BEGIN{printf "%.2f", (v<0?-v:v)*s}') m）"
  else ros2 topic pub -r 20 -t $(( $3 * 20 )) /cmd_vel geometry_msgs/msg/Twist "{linear: {x: $1}, angular: {z: $2}}" >/dev/null 2>&1; fi; }
zk() { awk -v k="$1" -v v="$2" 'BEGIN{printf "%.4f", v*k}'; }

stages_all() {
  mark rest;           if [ $DRY = 1 ]; then echo "    [等待] 20 秒（靜止基準）"; else sleep 20; fi
  mark straight_fwd;   drive 0.15 0.0 6
  mark straight_rev;   drive -0.15 0.0 6
  mark left_lock;      drive 0.10 0.14 6          # κ=1.4 → 韌體壓到 R=0.75（滿舵）
  mark right_lock;     drive 0.10 -0.14 6
  stages_graded
  mark lowspeed_L;     drive 0.10 0.005 5         # κ=0.05：量化與死區
  mark reverse_right;  drive -0.10 0.083 6        # 倒車 + ω>0 = 輪子打右
  mark reverse_left;   drive -0.10 -0.095 6
  mark slalom
  for i in 1 2 3 4 5 6 7 8; do z=$([ $((i%2)) = 1 ] && echo 0.07 || echo -0.07); drive 0.12 $z 2; done
  mark chain_left
  if [ $DRY = 1 ]; then echo "    [發送] /cmd_vel_smoothed  linear.x=0.15 angular.z=0.143  共 6 秒（走完整鏈路 trim→防撞→地板，κ=0.95）"
  else ros2 topic pub -r 20 -t 120 /cmd_vel_smoothed geometry_msgs/msg/Twist "{linear: {x: 0.15}, angular: {z: 0.143}}" >/dev/null 2>&1; fi
}
stages_graded() {
  for k in 0.2 0.5 0.717 1.0;   do mark "kL_$k"; drive 0.10 "$(zk $k 0.10)" 5; done
  for k in 0.2 0.5 0.83 1.068;  do mark "kR_$k"; drive 0.10 "-$(zk $k 0.10)" 5; done
}

if [ $DRY = 1 ]; then
  echo "【dry-run】只印計畫，不會碰車、不會發任何話題。標記後綴：'${SUFFIX}'；範圍：$ONLY"
  if [ "$ONLY" = graded ]; then stages_graded; mark end; else stages_all; mark end; fi
  echo "【dry-run 結束】"; exit 0
fi

# ── 以下才是真的 ─────────────────────────────────────────────
stop_car() { timeout 3 ros2 topic pub -r 30 -t 30 /cmd_vel geometry_msgs/msg/Twist '{}' >/dev/null 2>&1; }
stop_rec() { if [ -n "$REC_PID" ] && kill -0 "$REC_PID" 2>/dev/null; then kill -INT "$REC_PID" 2>/dev/null; for _ in $(seq 1 20); do kill -0 "$REC_PID" 2>/dev/null || break; sleep 0.5; done; fi; REC_PID=""; }
cleanup() { echo; echo "⚠ 中止：補送零速、停止錄製……"; stop_car; stop_rec; echo "已停止。已錄到的部分在 $CSV（可用 analyze 分析，但這份實驗要當作未完成）"; exit 130; }
trap cleanup INT TERM

[ -t 0 ] || { echo "✗ 這支需要在互動終端機執行（要輸入 YES 確認安全）"; exit 2; }
[ -f "$HOME/maprun/env.sh" ] || { echo "✗ 找不到 ~/maprun/env.sh"; exit 1; }
# ★ ROS 的 setup.bash 會讀未定義的變數：在 set -u 下 source 它會直接中止腳本（而且我把錯誤導去 /dev/null，看起來像無聲退出）。
set +u
# shellcheck disable=SC1091
source "$HOME/maprun/env.sh" >/dev/null 2>&1
set -u
[ -f "$TOOLS/record_steering_chain.py" ] && [ -f "$TOOLS/analyze_steering_logs.py" ] || { echo "✗ 找不到 $TOOLS 底下的錄製／分析工具"; exit 1; }

echo "══ 前置檢查 ══"
NODES="$(ros2 node list 2>/dev/null)"
need_ok=1
for pat in "wheeltec_robot" "ekf_filter_node"; do
  if echo "$NODES" | grep -q "$pat"; then echo "  ✓ 節點 $pat"; else echo "  ✗ 沒有節點 $pat（底盤＋雷達＋IMU 沒開？）"; need_ok=0; fi
done
for pat in "steering_trim" "collision_monitor" "cmd_vel_floor" "amcl" "path_teach"; do
  if echo "$NODES" | grep -q "$pat"; then echo "  ✓ 節點 $pat"; else echo "  ✗ 沒有節點 $pat（導航堆疊沒開成定位模式？chain_left 階段與 AMCL 旗標會缺）"; need_ok=0; fi
done
batt() { ( timeout 8 ros2 topic echo /PowerVoltage --once --field data 2>/dev/null | grep -Eo '^[0-9]+([.][0-9]+)?$' | head -1 ); }
V0="$(batt)"
if [ -z "$V0" ]; then echo "  ✗ 讀不到電池電壓"; need_ok=0
elif awk -v v="$V0" 'BEGIN{exit !(v>=23.5)}'; then echo "  ✓ 電池 ${V0} V（≥23.5）"
else echo "  ✗ 電池 ${V0} V，低於 23.5 V —— 先充電，不要做實驗"; need_ok=0; fi
if [ "$need_ok" != 1 ]; then echo; echo "✗ 前置檢查沒過，不開始。把上面 ✗ 的處理掉再跑一次（沒有開任何東西、車子沒有動）。"; exit 1; fi

echo; echo "══ 安全確認 ══"
cat <<'EOF'
  這個實驗會讓車子自己前進、倒車、繞弧，總共約 5 分鐘、範圍約 2~3 m，速度最高 0.15 m/s。
    □ 車子前後左右各 ≥ 2 m 空曠（沒有牆、椅子、人的腳）
    □ 有人站在車旁看著，**手一直放在這個終端機的 Ctrl-C 上**
    □ ★ 平板「緊急停止」與 estop_cc.sh **停不了這個實驗**（腳本直接發 /cmd_vel，比它們更下游）；
       唯一可靠的停法：這個終端機按 Ctrl-C；或把車抓住／關底盤電源。
    □ 感測器啟動已超過 20 秒，車子沒有被碰
EOF
printf "  都確認了請輸入 YES 開始（其他任何輸入都會取消）： "
read -r ANS
[ "$ANS" = "YES" ] || { echo "已取消，什麼都沒動。"; exit 0; }

"$HOME/maprun/trip_mark.sh" "T5 轉向實驗開始" "範圍=$ONLY 後綴='$SUFFIX' 電池=${V0}V csv=$CSV" >/dev/null 2>&1

start_rec() { python3 "$TOOLS/record_steering_chain.py" --out "$CSV" "$@" > "$REC_LOG" 2>&1 & REC_PID=$!; }
echo; echo "══ 錄製：先空轉 30 秒，看各來源訊息數 ══"
start_rec
sleep 31
LAST="$(grep '訊息數' "$REC_LOG" | tail -1)"
echo "  ${LAST:-（沒有印出訊息數！錄製可能沒起來）}"
cnt() { echo "$LAST" | grep -oE "(^|[ ：])$1=[0-9]+" | grep -oE '[0-9]+$' | head -1; }
IMU_ARGS=""
if [ "$(cnt imu_unbiased)" = "" ] || [ "$(cnt imu_unbiased)" = "0" ]; then
  if [ "$(cnt imu_raw)" != "" ] && [ "$(cnt imu_raw)" != "0" ]; then
    echo "  ⚠ imu_unbiased 是 0、imu_raw 有資料 → 依文件改用 --imu-unbiased /imu/data_raw 重新錄製（報告要註明）"
    stop_rec; IMU_ARGS="--imu-unbiased /imu/data_raw"; start_rec $IMU_ARGS; sleep 12; LAST="$(grep '訊息數' "$REC_LOG" | tail -1)"; echo "  ${LAST}"
  fi
fi
bad=""
for s in odom_raw odom_ekf; do [ "$(cnt $s)" = "" ] || [ "$(cnt $s)" = "0" ] && bad="$bad $s"; done
if [ "$(cnt imu_unbiased)" = "" ] || [ "$(cnt imu_unbiased)" = "0" ]; then [ -z "$IMU_ARGS" ] && bad="$bad imu(兩個都沒資料)"; fi
if [ -n "$bad" ]; then echo "  ✗ 這些來源沒有訊息：$bad —— 實驗資料會不完整，不開始。（車子沒有動）"; stop_rec; exit 1; fi
for s in amcl tf_map_base; do [ "$(cnt $s)" = "" ] || [ "$(cnt $s)" = "0" ] && echo "  ⚠ $s 沒有訊息（定位沒起來？）：GYRO_SCALE／AMCL 類旗標會缺，轉向分級本身不受影響"; done

echo; echo "══ 開始實驗（Ctrl-C 隨時中止）══"
if [ "$ONLY" = graded ]; then stages_graded; else stages_all; fi
mark end
sleep 2
stop_car
V1="$(batt)"
stop_rec
echo; echo "══ 結束。電池 ${V0} V → ${V1:-讀不到} V ══"

echo "══ 分析 ══"
{
  echo "# 轉向實驗分析 $(date '+%F %T')（範圍=$ONLY 後綴='$SUFFIX'）"
  echo; echo "- 電池：開始 ${V0} V → 結束 ${V1:-讀不到} V"; echo "- 錄製檔：$CSV（$(wc -l < "$CSV") 行）"; echo "- IMU 參數：${IMU_ARGS:-預設 /imu/data_unbiased}"
  echo "- 判讀準則（實驗前凍結）：~/maprun/verify_out/steer_criteria_frozen_20261020.md"
  echo; echo "## 錄製節點的訊息數（每 5 秒一行；最後 3 行）"; echo '```'; grep '訊息數' "$REC_LOG" | tail -3; echo '```'
  echo; echo "## analyze 輸出"; echo '```'
  python3 "$TOOLS/analyze_steering_logs.py" analyze "$CSV" --json "$JSON" 2>&1
  echo '```'
} > "$MD"
cat "$MD"
"$HOME/maprun/trip_mark.sh" "T5 轉向實驗結束" "電池 ${V1:-?}V 分析=$MD" >/dev/null 2>&1
echo
echo "已寫出：$CSV  $JSON  $MD"
echo "下一步：照 steer_criteria_frozen_20261020.md 判讀（R1／R2／R3），再登記："
echo "  ~/maprun/tools_1020/result.sh 5 PASS|FAIL|WARN \"R1 …；R2 …\""
trap - INT TERM
exit 0
