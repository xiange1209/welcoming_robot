#!/bin/bash
# 車上操作選單（★ 2026-10-24 車上 agent）：demo 與下週測試**不必開 VS Code**。
#
#   筆電 cmd.exe：ssh user@192.168.137.106      （IP 以筆電熱點清單為準）
#   進去之後打：  ~/maprun/car_menu.sh
#
# 平常操作（開情境、帶位、遙控、急停）都在平板 HMI；這裡放平板做不到的：看狀態、量測、A/B 切換、打包、關機。
# ★ 會讓車子動的選項一律先講清楚、要你打 yes。
# ★ 平板急停**停不了**「直接發 /cmd_vel」的工具（選項 12）——那種只能在這個視窗按 Ctrl-C，或選項 15。
# ★ 任何一項跑到一半按 Ctrl-C 會停掉那一項、回到選單（選單本身不會被關掉）。
# 注意：不要 set -u（source ROS 會靜默中止，見記憶 set-u-breaks-ros-source）。

source /home/user/maprun/env.sh >/dev/null 2>&1
T=/home/user/maprun/tools_1020
LOGS=/home/user/.smartnav/logs/system_control
TRIM_JSON=/home/user/.smartnav/steering_trim.json
trap 'echo; echo "（已中斷這一項）"' INT

pause() { read -rp "（按 Enter 回選單）" _; }
confirm() { local a; read -rp "$1 確定請打 yes：" a; [ "$a" = "yes" ]; }
hmi() { python3 "$T/hmi_unit.py" "$@"; }

nav_last() {
  local L C
  L=$(ls -t "$LOGS"/nav_*.log 2>/dev/null | head -1)
  if [ -z "$L" ]; then echo "還沒有導航 log"; return; fi
  echo "最新導航 log：$L"
  grep -hE "多候選規劃|循跡修正：|暫時降到|開始重播|導航成功|導航失敗|已抵達終點|偏離路徑|交回導航層|折返點：|重新規劃第" "$L" \
    | tail -25 | sed -E 's/^\[[^]]*\] \[[A-Z]+\] \[[0-9.]+\] \[[a-z_]+\]: //' | cut -c1-200
  C=$(ls -t /home/user/maprun/logs/replay_*_live_*.csv 2>/dev/null | head -1)
  [ -n "$C" ] && python3 - "$C" <<'EOF'
import csv, sys
rows = list(csv.DictReader(open(sys.argv[1], encoding="utf-8")))
x = sorted(abs(float(r["xte_m"])) for r in rows if r.get("xte_m"))
if x:
    print(f"最近一趟逐拍紀錄 {sys.argv[1].split('/')[-1]}：{len(rows)} 拍、偏離 中位 {100 * x[len(x) // 2]:.1f} cm、"
          f"p95 {100 * x[int(0.95 * (len(x) - 1))]:.1f} cm、最大 {100 * x[-1]:.1f} cm")
EOF
}

nav_ab() {
  echo "目前的設定值（這一趟實際用了什麼，以導航 log 的「循跡修正：…」與「多候選規劃」為準）："
  echo "  規劃半徑：$(ros2 param get /planner_server GridBased.minimum_turning_radius 2>&1 | tail -1)"
  for p in plan_candidates lookahead_interpolate xte_segment xte_integral; do
    echo "  $p：$(ros2 param get /path_teach_cc_node $p 2>&1 | tail -1)"
  done
  echo "  n) 新行為（規劃半徑 1.40＋四項全開）   o) 舊行為（0.95＋四項全關）   其他) 不改"
  local m R V p
  read -rp "選：" m
  case "$m" in n) R=1.4; V=true ;; o) R=0.95; V=false ;; *) echo "沒改"; return ;; esac
  echo "改規劃半徑（Smac 要重建查找表，約 5 秒）…"
  ros2 param set /planner_server GridBased.minimum_turning_radius $R
  for p in plan_candidates lookahead_interpolate xte_segment xte_integral; do
    ros2 param set /path_teach_cc_node $p $V
  done
  echo "✓ 下一趟導航生效（不用重啟）。導航單元沒開的話上面會報錯——先在平板開情境。"
}

trim_set() {
  echo "目前：$(cat "$TRIM_JSON" 2>/dev/null || echo '（沒有檔案）')"
  echo "把選項 12 量到的「trim_rad_per_m 設 … 可抵消」那個值填進來（範圍 -0.30 ~ +0.30）。"
  local v
  read -rp "新的 trim_rad_per_m：" v
  python3 -c "import sys; v=float(sys.argv[1]); sys.exit(0 if -0.30 <= v <= 0.30 else 1)" "$v" 2>/dev/null \
    || { echo "✗ 不是 -0.30~+0.30 的數字，沒改"; return; }
  confirm "要把 trim 改成 $v（舊檔會備份）？" || { echo "沒改"; return; }
  cp -p "$TRIM_JSON" "$TRIM_JSON.bak_$(date +%Y%m%d_%H%M%S)" 2>/dev/null
  python3 - "$TRIM_JSON" "$v" <<'EOF'
import datetime, json, sys
json.dump({"trim_rad_per_m": float(sys.argv[2]), "saved_at": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
           "updates": 0, "note": "car_menu 手動設定"}, open(sys.argv[1], "w"))
EOF
  echo "✓ 已寫入。下次開導航單元時載入（在平板先全部關閉再開情境）。"
}

while true; do
  [ -t 1 ] && clear
  cat <<EOF
================ 迎賓機器人 操作選單（$(hostname)，$(date '+%m/%d %H:%M')）================
 平常操作都在平板 HMI；這裡是平板做不到的事。任何一項按 Ctrl-C 都會停下來回選單。
 ── 看狀態（唯讀）─────────────────────────────────────────────
   1) 一頁看完現況（電池／溫度／CPU／記憶體／單元／話題／ASR／LLM／導航警告）
   2) 開工檢查 V0（沒接周邊時電池、底盤、相機、麥克風會 FAIL，屬正常）
   3) 最近一趟導航結果（多候選規劃、循跡修正、成功／失敗、偏離統計）
 ── 開關（等同平板「系統開關」）────────────────────────────────
   4) 開「完整迎賓故事線」   5) 開「完整迎賓故事線（人臉省電）」   6) 全部關閉
 ── 測試（下週清單）───────────────────────────────────────────
   7) 量 CPU／負載 30 分鐘（清單第 3 節；開情境、跑故事線時一起量）
   8) 語音錯字率（清單 2-1：先印稿，再錄 180 秒唸 10 句；會先關 LLM）
   9) LLM 銀行十題（★ 有幾題會真的開車：場地淨空、有人拿急停）
  10) 導航新舊 A/B 切換（清單 4-0b；不用重啟）
  11) 登記一項測試結果（PASS／FAIL＋備註）
  17) 量整體延遲（說完話→文字→LLM→車子動；跟故事線一起開，Ctrl-C 結束；報告第 14 節）
 ── 舵機／轉向 ───────────────────────────────────────────────
  12) 轉向量測：直走＋左右打滿（★ 車會自己動；四周 1.5 m 淨空；平板急停停不了，只能 Ctrl-C）
  13) 把量到的 trim 寫回去（校完舵機電位器之後一定要做）
 ── 收尾 ───────────────────────────────────────────────────
  14) 打包回傳（會印出 scp 指令）
  15) 緊急停車（殺掉所有發速度指令的行程並補送零速）
  16) 關機（先全部關閉單元，再關機）
   0) 離開
EOF
  read -rp "選項：" c
  case "$c" in
    1) python3 "$T/live_status.py" --secs 6; pause ;;
    2) /home/user/maprun/verify/verify_0_preflight.sh; pause ;;
    3) nav_last; pause ;;
    4) hmi scenario demo sensors nav llm; pause ;;
    5) hmi scenario demo_eco sensors nav llm; pause ;;
    6) hmi scenario all_off; pause ;;
    7) read -rp "這組叫什麼（例：單板全開、單板省電）：" lab
       echo "每 10 秒一列、量 30 分鐘；要提早結束按 Ctrl-C（已量的會存下來）。"
       python3 "$T/load_monitor.py" --label "${lab:-單板}" --secs 1800; pause ;;
    8) echo "會先關 LLM（不然機器人會回話、甚至開車），並確認語音單元開著。"
       if confirm "開始？"; then
         hmi stop llm; hmi start asr_chain
         python3 "$T/asr_cer.py" --print-script
         read -rp "準備好就按 Enter 開始錄 180 秒（站 60 cm，照稿唸，每句停 3 秒）…" _
         python3 "$T/asr_cer.py"
       fi; pause ;;
    9) if confirm "★ 會真的開車的題目會讓車子動。場地淨空、有人拿急停了嗎？"; then
         python3 "$T/llm10.py" --clear-each
       fi; pause ;;
   10) nav_ab; pause ;;
   11) read -rp "項目編號（例 2-1、4-0）：" it; read -rp "結果（PASS／FAIL／WARN／未做）：" rs
       read -rp "數字或備註：" nt; "$T/result.sh" "$it" "$rs" "$nt"; pause ;;
   12) echo "會先關導航、開感測器，再印出量測計畫（不開車）；確認後才真的開。"
       if confirm "要開始轉向量測？"; then
         hmi stop nav; hmi start sensors
         python3 /home/user/maprun/steer_asym_check_cc.py --dry
         if confirm "★ 車子要動了（直走＋左右打滿）。四周 1.5 m 淨空、人站在旁邊了嗎？"; then
           python3 /home/user/maprun/steer_asym_check_cc.py
           echo "★ 記下「trim_rad_per_m 設 … 可抵消」與左右半徑，給選項 13 和筆電端用。"
         fi
       fi; pause ;;
   13) trim_set; pause ;;
   17) echo "唯讀：只聽話題、不發指令。要同時量 CPU 的話，另開一個 SSH 視窗跑選項 7。"
       python3 "$T/latency_logger.py"; pause ;;
   14) /home/user/maprun/verify/collect_data.sh --all; pause ;;
   15) /home/user/maprun/estop_cc.sh; pause ;;
   16) if confirm "先全部關閉單元，再關機？"; then
         hmi scenario all_off; sleep 10; sudo systemctl poweroff
       fi; pause ;;
    0) exit 0 ;;
    *) ;;
  esac
done
