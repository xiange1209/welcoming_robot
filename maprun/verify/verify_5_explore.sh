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

# 車上 explorer 的原始碼有沒有我們的修補（看門狗濾 0）。只看原始碼、看不到 binary —— V0 修正11 同理
SUPP="$HOME/welcoming_robot_ws/src/frontier_exploration_ros2/src/frontier_suppression.cpp"
if [ -f "$SUPP" ]; then grep -q "smartnav patch" "$SUPP" && PATCHED=1 || PATCHED=0; else PATCHED=unknown; fi

python3 - "$LOG" "$RESULT" "${CPU_FILE:-}" "$PATCHED" <<'PY'
import re, sys, statistics

log_path, result_path, cpu_path, patched = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
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
    "accepted":    n(r"Frontier goal accepted"),
    "rejected":    n(r"Frontier goal was rejected"),      # 會記失敗
    "stuck":       n(r"偵測到卡住"),
    "deferred":    n(r"自動探索中：不取消目標"),
    "grace_cancel": n(r"照舊取消，先讓車停下來"),          # 探索開始 30 秒寬限期內，照舊取消（9/25）
    "backstop":    n(r"後備取消"),                        # 讓步約 20 秒（卡住後約 24 秒）車仍沒動 -> 照舊取消（9/25）
    "budget":      n(r"探索中卡住第 \d+ 次"),             # map_service 的卡住預算計數
    "extrap":      n2(r"extrapolation into the future"),
    "no_path":     n2(r"no valid path|Failed to create plan|No path found"),
    "start_occ":   n2(r"Start occupied|START_OCCUPIED"),
    "goal_occ":    n2(r"Goal occupied|GOAL_OCCUPIED|GoalOccupied"),
    "no_progress": n2(r"Failed to make progress"),
}
end = re.findall(r"探索結束（(\w+)）", seg)
end = end[-1] if end else ("cancel" if "已被系統或使用者取消" in seg else "未知")
saved = "地圖建立成功" in seg

# ── 被取消的目標與「看門狗誤判」特徵（審查 upstream#1，9/25 查證 Jazzy 原始碼確認）──
# explorer 的看門狗取消、被擋取消都只印 DEBUG，INFO log 看不到原因。只能推算：
# 一個目標 accepted 之後，下一件事是另一個「Sending … frontier goal」而不是 reached／finished／rejected
# = 它被取消或搶佔了。nav2（Jazzy）每個新目標的第一筆 distance_remaining 都是 0.0，
# 看門狗拿 0 當基準後再怎麼前進都不算進展 -> 取消時間會集中在「逾時 + 約 1~5 秒（沉澱＋重選）」。
# ★ 9/25 審查更正：這個區間在**修補後**也會有正常的取消 —— 近距完成（complete_if_within 0.6 m，
#   地圖一更新就算到、不記失敗）、一開始就規劃不出路（接受後 15 秒被看門狗正確取消）。
#   所以區間內有 nav2「規劃不出路」或「偵測到卡住」的先排除；原始碼已修補時只當參考（INFO），
#   而且不再算進「主要卡住類型」—— 以前會讓人以為修補沒生效，去追一個已經修好的 bug。
TS = re.compile(r"\[(\d{10}\.\d+)\]")
ev = []
for line in seg.splitlines():
    m = TS.search(line)
    if not m:
        continue
    t = float(m.group(1))
    if "Frontier goal accepted" in line:
        ev.append((t, "acc"))
    elif re.search(r"Sending (updated )?frontier goal", line):
        ev.append((t, "send"))
    elif re.search(r"Frontier goal reached|frontier finished with status|Frontier goal was rejected", line):
        ev.append((t, "end"))
    elif re.search(r"no valid path|Failed to create plan|No path found|Start occupied|START_OCCUPIED", line, re.I):
        ev.append((t, "geo"))
    elif re.search(r"偵測到卡住|自動探索中：不取消目標", line):
        ev.append((t, "stuck"))
cancel_durs = []   # (accepted 到下一個送出的秒數, 期間有沒有「規劃不出路／卡住」可以解釋)
for i, (t, k) in enumerate(ev):
    if k != "acc":
        continue
    explained = False
    for t2, k2 in ev[i + 1:]:
        if k2 in ("geo", "stuck"):
            explained = True
        elif k2 == "end":
            break
        elif k2 == "send":
            cancel_durs.append((t2 - t, explained))
            break
m = re.search(r"no_progress_timeout=([\d.]+)s", text)
wd_timeout = float(m.group(1)) if m else None
wd_hits = [d for d, expl in cancel_durs if wd_timeout and wd_timeout <= d <= wd_timeout + 6 and not expl]

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
                              "stuck": "同一處卡住多次（疑似低於雷達的障礙，清場後再建）",
                              "timeout": "30 分鐘逾時", "cancel": "被取消", "未知": "未知（log 截斷？）"}.get(end, end))
    rec("地圖有沒有存下來", "PASS" if saved else "FAIL",
        "有" if saved else "沒有", "" if saved else "9/24 起逾時與停滯都會存圖；沒存多半是被取消或中途當掉")
    ok_rate = C["reached"] / C["sent"] if C["sent"] else 0
    rec("frontier 目標", "INFO",
        f"送出 {C['sent']}、抵達 {C['reached']}（{ok_rate:.0%}）、nav2 放棄 {C['aborted']}、"
        f"被拒 {C['rejected']}、其他結束 {C['fail_other']}")
    rec("被取消／被搶佔（推算）", "INFO", f"{len(cancel_durs)} 個",
        "INFO log 看不到取消原因；推算方式：accepted 之後下一件事是另一個送出")
    rec("車上 explorer 原始碼", "PASS" if patched == "1" else ("FAIL" if patched == "0" else "INFO"),
        {"1": "有修補", "0": "上游原版（沒修補）", "unknown": "找不到原始碼"}.get(patched, patched),
        "" if patched == "1" else "沒修補的話看門狗會把開超過 15 秒的目標誤判失敗 —— 看 V0 修正11，重新打包部署並 colcon build")
    win = f"{wd_timeout:.0f}~{wd_timeout + 6:.0f} 秒" if wd_timeout else ""
    if wd_timeout is None:
        rec("看門狗取消時間", "INFO", "log 裡找不到 no_progress_timeout（explorer 啟動那行）", "")
    elif len(cancel_durs) < 3:
        rec("看門狗取消時間", "INFO", f"取消只有 {len(cancel_durs)} 個，樣本太少不判", "")
    elif patched == "1":
        rec("看門狗取消時間", "INFO",
            f"{len(wd_hits)}/{len(cancel_durs)} 個取消落在 {win}（已排除期間有規劃不出路、卡住的）",
            "修補後落在這裡的多半是正常機制：近距完成（0.6 m 內就算到）、一開始就沒進展。"
            "要確認有沒有鎖死，對 explorer 開 debug，看 'Distance remaining: 0.000' 之後 15 秒被取消")
    else:
        frac = len(wd_hits) / len(cancel_durs)
        rec("看門狗誤判特徵", "WARN" if frac >= 0.5 else "INFO",
            f"{len(wd_hits)}/{len(cancel_durs)} 個取消落在 {win}（已排除期間有規劃不出路、卡住的）",
            "集中在這裡**可能**是上游看門狗被 nav2 第一筆 distance_remaining=0 鎖死、"
            "正常前進的目標也被判失敗（審查 upstream#1）；也可能是近距完成")
    rec("全部被抑制的次數", "INFO", str(C["suppressed"]), "多 = 很多地方到不了，停滯看門狗會在 4 分鐘後結束")
    rec("explorer 宣告沒有 frontier", "INFO", str(C["no_more"]), "有 = 真的探索完了（阿克曼車上很少見）")

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
    f"偵測到卡住 {C['stuck']} 次：探索中讓步 {C['deferred']} 次、寬限期內照舊取消 {C['grace_cancel']} 次、"
    f"讓步逾時後備取消 {C['backstop']} 次；map_service 記到 {C['budget']} 次",
    "讓步 = 交給 explorer 記失敗；寬限期（探索開始 30 秒）內 explorer 的看門狗不動作，所以照舊取消；"
    "後備取消多 = explorer 沒在接手（當掉、卡在長計算），或換了目標又頂上同一個障礙")
# 探索開始前的一般導航也可能卡住，所以「卡住次數 > 讓步＋寬限期取消」不一定是錯；
# 只有「探索中卡住、三種處理都沒出現」才代表車上是舊版 stuck_detector
if explored and C["stuck"] and C["deferred"] == 0 and C["grace_cancel"] == 0 and C["backstop"] == 0:
    rec("卡住偵測器讓步", "FAIL", "探索中卡住，卻沒有讓步、寬限期取消、後備取消任何一種",
        "車上可能還是舊版 stuck_detector —— 檢查有沒有重新 colcon build")
if explored and C["stuck"] and C["budget"] == 0:
    rec("卡住預算", "WARN", "探索中卡住，map_service 卻沒記到",
        "車上可能還是舊版 map_service（沒有 exploration_stuck_* 參數）—— 檢查有沒有重新 colcon build")
classes["實體（清場或加感測）"] = C["stuck"] * 3
# 「看門狗取消時間」刻意不算進來：它是推算、分不出近距完成，修補後會把正常的一趟排成主要問題

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
