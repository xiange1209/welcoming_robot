#!/usr/bin/env bash
# V5 自動探索：跑一趟、當場判斷「這次卡住是哪一類」。
#
# 為什麼要分類：卡住的原因決定下一步該去哪裡修 ——
#   CPU／TF 類（TF 空窗、controller 報 extrapolation）→ 模擬器重現不了，要減負載
#   幾何類（no valid path、Start occupied、選到掉不了頭的點）→ 可以帶回 Gazebo 調參
#   實體類（輪子被光達看不到的矮障礙卡住）→ 清場或加感測
# 8/19~9/15 的 21 份 nav log 裡探索一次都沒真正跑過，所以「現在」卡的是哪一類其實沒有證據。
# 同一批 log 在一般導航時就已經出現 101 次 extrapolation into the future ——
# 那正是 7/29 擋下探索的 CPU 瓶頸，所以這支特別量它。
#
# 用法（兩個終端機）：
#   終端機 A：探索開始前  ~/maprun/verify/verify_5_explore.sh watch    # Ctrl-C 結束
#   平板    ：系統開關 → 測試情境「建圖（自動探索）」→ 遙控建圖分頁「開始建圖」
#   探索結束（自動存圖或按「完成存檔」）後：
#   終端機 B：~/maprun/verify/verify_5_explore.sh report [nav_cc_*.log]  # 預設取最新一份

source "$(dirname "$0")/_lib.sh"
MODE="${1:-report}"
LOGDIR="$HOME/maprun/logs"
CPU_CSV="$VDIR/explore_cpu.csv"

# ════════════════════════════════════════════════════════════════
# watch：每 5 秒記一次 CPU 閒置率與負載。只讀 /proc，不呼叫 ros2 ——
#        Pi 在探索時 CPU 已經很緊，量測本身不能再搶資源。
# ════════════════════════════════════════════════════════════════
if [ "$MODE" = "watch" ]; then
  echo "V5 探索中 CPU 記錄 → $CPU_CSV（每 5 秒一筆，Ctrl-C 結束）"
  echo "時間,cpu閒置%,load1" > "$CPU_CSV"
  read -r _ u n s i w x y z _ < /proc/stat
  prev_total=$((u+n+s+i+w+x+y+z)); prev_idle=$((i+w))
  trap 'echo; echo "已停止。筆數：$(($(wc -l < "$CPU_CSV")-1))。探索結束後跑 report。"; exit 0' INT
  while true; do
    sleep 5
    read -r _ u n s i w x y z _ < /proc/stat
    total=$((u+n+s+i+w+x+y+z)); idle=$((i+w))
    dt=$((total-prev_total)); di=$((idle-prev_idle))
    pct=$(awk -v a="$di" -v b="$dt" 'BEGIN{printf "%.1f", (b>0)?100*a/b:0}')
    load=$(cut -d' ' -f1 /proc/loadavg)
    echo "$(date +%H:%M:%S),$pct,$load" >> "$CPU_CSV"
    printf '\r  %s  閒置 %5s%%  load %s   ' "$(date +%H:%M:%S)" "$pct" "$load"
    prev_total=$total; prev_idle=$idle
  done
fi

# ════════════════════════════════════════════════════════════════
# report：解析這趟的 nav log，分類卡住原因
# ════════════════════════════════════════════════════════════════
LOG="${2:-$(ls -t "$LOGDIR"/nav_cc_*.log 2>/dev/null | head -1)}"
echo "V5 自動探索報告   $(date '+%Y-%m-%d %H:%M:%S')"
if [ -z "$LOG" ] || [ ! -f "$LOG" ]; then
  rec "nav log" FAIL "找不到 ~/maprun/logs/nav_cc_*.log" "導航堆疊要用 run_nav_cc.sh（或平板的測試情境）啟動才會寫 log"
  banner_done; exit 1
fi
rec "分析的 log" INFO "$(basename "$LOG")" "$(du -h "$LOG" | cut -f1)"

# 找最新的 CPU 記錄（watch 可能是另一次 verify 的 STAMP 目錄）
CPU_FILE=$(ls -t "$HOME"/maprun/verify_out/*/explore_cpu.csv 2>/dev/null | head -1)

python3 - "$LOG" "$RESULT" "${CPU_FILE:-}" <<'PY'
import re, sys, statistics

log_path, result_path, cpu_path = sys.argv[1], sys.argv[2], sys.argv[3]
text = open(log_path, encoding="utf-8", errors="replace").read()

# 只看「最後一次探索」那一段：從最後一個「自動探索已啟動」起算。
# 同一份 log 可能先有一般導航的雜訊（extrapolation 在導航時就會出現）。
start = text.rfind("自動探索已啟動")
seg = text[start:] if start >= 0 else ""
explored = start >= 0

def n(pat, s=seg):
    return len(re.findall(pat, s, flags=re.I))

# nav2 的錯誤會被 explorer 的失敗訊息轉述一次（error_msg='no valid path'），
# 只數 nav2 自己印的那行，否則同一次失敗算兩遍
nav2_lines = "\n".join(l for l in seg.splitlines() if "frontier finished with status" not in l)

def n2(pat):
    return n(pat, nav2_lines)

C = {
    "sent":        n(r"Sending (updated )?frontier goal"),
    "reached":     n(r"Frontier goal reached"),
    "aborted":     n(r"frontier finished with status STATUS_ABORTED"),
    "fail_other":  n(r"frontier finished with status (?!STATUS_ABORTED)"),
    "suppressed":  n(r"frontiers (are|remain) (temporarily )?suppressed"),
    "no_more":     n(r"No more frontiers found"),
    "stuck":       n(r"偵測到卡住"),
    "deferred":    n(r"自動探索中：不取消目標"),
    "extrap":      n2(r"extrapolation into the future"),
    "no_path":     n2(r"no valid path|Failed to create plan|No path found"),
    "start_occ":   n2(r"Start occupied|START_OCCUPIED"),
    "goal_occ":    n2(r"Goal occupied|GOAL_OCCUPIED|GoalOccupied"),
    "no_progress": n2(r"Failed to make progress"),
}
end = re.findall(r"探索結束（(\w+)）", seg)
end = end[-1] if end else ("cancel" if "已被系統或使用者取消" in seg else "未知")
saved = "地圖建立成功" in seg

rows = []
def rec(item, verdict, value, basis=""):
    color = {"PASS": "\033[32m", "FAIL": "\033[31m", "WARN": "\033[33m"}.get(verdict, "\033[2m")
    print(f"  {color}{verdict:4}\033[0m {item:<30} {value}")
    if basis:
        print(f"       \033[2m{basis}\033[0m")
    rows.append(f"{item}\t{verdict}\t{value}\t{basis}\n")

print("\n══ 這趟探索 ══")
if not explored:
    rec("自動探索有沒有真的開始", "FAIL", "log 裡沒有「自動探索已啟動」",
        "情境要選「建圖（自動探索）」，而且要在遙控建圖分頁按「開始建圖」")
else:
    rec("結束方式", "INFO", {"done": "探索完成", "stalled": "停滯（地圖很久沒長大，原因看下面的分類）",
                              "timeout": "30 分鐘逾時", "cancel": "被取消", "未知": "未知（log 截斷？）"}.get(end, end))
    rec("地圖有沒有存下來", "PASS" if saved else "FAIL",
        "有" if saved else "沒有", "" if saved else "9/24 起逾時與停滯都會存圖；沒存多半是被取消或中途當掉")
    ok_rate = C["reached"] / C["sent"] if C["sent"] else 0
    rec("frontier 目標", "INFO",
        f"送出 {C['sent']}、抵達 {C['reached']}（{ok_rate:.0%}）、nav2 放棄 {C['aborted']}、其他結束 {C['fail_other']}")
    rec("全部被抑制的次數", "INFO", str(C["suppressed"]), "多 = 很多地方到不了，停滯看門狗會在 4 分鐘後結束")

print("\n══ 卡住分類 ══")
classes = {}

# CPU／TF：controller 查 TF 查不到 = TF 發布跟不上 = CPU 不夠
cpu_idle = []
if cpu_path:
    for line in open(cpu_path, encoding="utf-8").read().splitlines()[1:]:
        try:
            cpu_idle.append(float(line.split(",")[1]))
        except (IndexError, ValueError):
            pass
if cpu_idle:
    med = statistics.median(cpu_idle)
    low = sum(1 for v in cpu_idle if v < 5) / len(cpu_idle)
    rec("CPU 閒置率（中位數）", "WARN" if med < 10 else "PASS", f"{med:.1f}%（低於 5% 的時間佔 {low:.0%}）",
        "7/29 決定性的那一輪是 0.4~7%")
else:
    rec("CPU 記錄", "WARN", "沒有", "下次探索時另開終端機跑 verify_5_explore.sh watch")
v = "WARN" if C["extrap"] > 5 else "PASS"
rec("TF 空窗（extrapolation）", v, f"{C['extrap']} 次",
    "controller 用「現在」查 TF 卻查不到 → map/odom 的 TF 發布跟不上 → CPU 不夠。模擬器重現不了")
classes["CPU／TF（模擬器救不了，要減負載）"] = C["extrap"] + (10 if cpu_idle and statistics.median(cpu_idle) < 10 else 0)

geo = C["no_path"] + C["start_occ"] + C["goal_occ"]
rec("幾何類（規劃不出路）", "WARN" if geo > 3 else "INFO",
    f"no valid path {C['no_path']}、Start occupied {C['start_occ']}、Goal occupied {C['goal_occ']}",
    "阿克曼轉不過／掉不了頭／目標貼牆。這類可以帶回 Gazebo 調參")
classes["幾何（可以帶回模擬器調）"] = geo

rec("實體卡住（矮障礙）", "WARN" if C["stuck"] else "PASS",
    f"偵測到卡住 {C['stuck']} 次，其中探索中讓步 {C['deferred']} 次",
    "讓步次數 = 9/24 的修正有生效（交給 explorer 記失敗，不會再無限重試同一點）")
if explored and C["stuck"] and C["deferred"] == 0:
    rec("卡住偵測器讓步", "FAIL", "探索中卡住卻沒有讓步", "車上可能還是舊版 stuck_detector —— 檢查有沒有重新 colcon build")
classes["實體（清場或加感測）"] = C["stuck"] * 3

rec("nav2 無進展", "INFO", f"{C['no_progress']} 次",
    "多 = 在 BT 恢復裡來回倒車。9/24 起 explorer 看門狗 15 秒會先於 nav2 的 20 秒取消")

if explored:
    top = max(classes, key=classes.get)
    print()
    if classes[top] == 0:
        rec("主要卡住類型", "PASS", "沒有明顯卡住", "")
    else:
        rec("主要卡住類型", "INFO", top, "依次數加權粗估，細節看上面各項")

open(result_path, "a", encoding="utf-8").writelines(rows)
PY

banner_done
