#!/usr/bin/env bash
# V4 教導-重現：當場算出循跡誤差，不必等回電腦。
#
# ★ 統計配方（已用 8/29 三趟原始檔驗證可重現報告的 2,576 筆 / 0.172 公尺）：
#     每個檔案逐列讀，一遇到 state 是 escaping 或 blocked 就【停止讀這個檔案】。
#     是【截斷】不是【過濾】—— 脫困之後 AMCL 定位已被破壞，那段的橫向誤差沒有物理意義。
#     單純濾掉脫困狀態會得到 1.37 公尺，差一個數量級。
#
# 做法：教導錄一條 → 重現 4 趟 → 跑這支
#
# 用法：~/maprun/verify/verify_4_replay.sh [今天日期 YYYYMMDD，預設今天]

source "$(dirname "$0")/_lib.sh"
DAY="${1:-$(date +%Y%m%d)}"

echo "V4 教導-重現循跡誤差   $(date '+%Y-%m-%d %H:%M:%S')"
echo "  統計對象：~/maprun/logs/replay_path_*_${DAY}_*.csv"

FILES=$(ls -1 "$HOME"/maprun/logs/replay_path_*_"${DAY}"_*.csv 2>/dev/null)
N=$(echo "$FILES" | grep -c . )
if [ "$N" -eq 0 ]; then
  rec "重現趟數" FAIL "0" "找不到今天的 replay CSV —— 先錄教導路徑並重現幾趟"
  banner_done; exit 1
fi
rec "重現趟數" INFO "$N 趟" "目標 4 趟"

hdr "逐趟與合併統計"
python3 - "$DAY" "$VDIR" <<'PY'
import csv, glob, os, statistics, sys, json

day, vdir = sys.argv[1], sys.argv[2]
paths = sorted(glob.glob(os.path.expanduser(
    f"~/maprun/logs/replay_path_*_{day}_*.csv")))

def truncated(path):
    """取到第一次 state 為 escaping/blocked 之前。★ 截斷，不是過濾。"""
    vals, total, cut = [], 0, None
    with open(path, encoding="utf-8-sig", newline="") as f:
        for i, row in enumerate(csv.DictReader(f), 1):
            total = i
            if row.get("state") in ("escaping", "blocked"):
                cut = i
                break
            v = row.get("cross_track_m", "")
            if v not in ("", "nan"):
                try:
                    vals.append(float(v))
                except ValueError:
                    pass
    return vals, total, cut

def pct(a, q):
    return sorted(a)[min(int(len(a) * q), len(a) - 1)]

allv, rows = [], []
for p in paths:
    v, total, cut = truncated(p)
    allv += v
    tag = f"第 {cut} 列脫困" if cut else "全程無脫困"
    rows.append({"檔案": os.path.basename(p), "採計": len(v), "全檔": total, "截斷": tag,
                 "p95": round(pct(v, .95), 4) if v else None})
    print(f"  {os.path.basename(p)[-22:]:>22}  採計 {len(v):>5} / 全檔 {total:>5}  "
          f"p95 {pct(v,.95):.4f} m  ({tag})" if v else
          f"  {os.path.basename(p)[-22:]:>22}  無有效資料")

print()
if allv:
    p95 = pct(allv, .95)
    summary = {"筆數": len(allv), "平均": round(statistics.fmean(allv), 4),
               "中位": round(statistics.median(allv), 4),
               "p95": round(p95, 4), "最大": round(max(allv), 4)}
    print(f"  合併：{len(allv)} 筆   平均 {summary['平均']} m   中位 {summary['中位']} m")
    print(f"        95 百分位 \033[1m{p95:.4f} m\033[0m   最大 {summary['最大']} m")
    print()
    print(f"  對照 2026-08-29 舊值：2,576 筆 / 0.1718 m")
    if p95 <= 0.20:
        print("  \033[32m→ 與舊值同級或更好\033[0m")
    else:
        print("  \033[33m→ 比舊值差，檢查定位品質與路徑是否經過最窄處\033[0m")
    json.dump({"逐趟": rows, "合併": summary},
              open(os.path.join(vdir, "replay_stats.json"), "w", encoding="utf-8"),
              ensure_ascii=False, indent=2)
PY

P95=$(python3 -c "
import json,os,sys
p=os.path.join('$VDIR','replay_stats.json')
print(json.load(open(p,encoding='utf-8'))['合併']['p95'] if os.path.exists(p) else '')
" 2>/dev/null)
if [ -n "$P95" ]; then
  if [ "$(awk -v x="$P95" 'BEGIN{print (x<=0.20)?1:0}')" = 1 ]; then
    rec "循跡誤差 95 百分位" PASS "${P95} m" "8/29 舊值 0.1718 m"
  else
    rec "循跡誤差 95 百分位" WARN "${P95} m" "比 8/29 的 0.1718 m 差"
  fi
fi

banner_done
