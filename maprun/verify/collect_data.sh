#!/usr/bin/env bash
# 打包今天的驗證結果與原始數據，並印出 scp 指令。
#
# ★★ 資安：以下兩項【絕對不打包】
#      ~/.smartnav/secrets/       Telegram token
#      ~/.smartnav/face_database/ 人臉特徵向量與姓名（個資）
#    打包前會把清單印出來讓你過目，有疑慮就 Ctrl-C。
#
# 用法：~/maprun/verify/collect_data.sh [YYYYMMDD，預設今天]

set -uo pipefail
DAY="${1:-$(date +%Y%m%d)}"
OUT="$HOME/car_data_${DAY}.tar.gz"
STAGE=$(mktemp -d)
trap 'rm -rf "$STAGE"' EXIT

echo "打包 $DAY 的資料"
echo

mkdir -p "$STAGE/verify" "$STAGE/logs" "$STAGE/smartnav"

# 1) 驗證結果
if [ -d "$HOME/maprun/verify_out" ]; then
  cp -r "$HOME"/maprun/verify_out/* "$STAGE/verify/" 2>/dev/null
fi

# 2) 今天的 CSV 與 log（研究數據要留著寫報告，全部帶走）
find "$HOME/maprun/logs" -maxdepth 1 \( -name "*${DAY}*.csv" -o -name "*.log" \) \
     -newermt '-24 hours' -exec cp {} "$STAGE/logs/" \; 2>/dev/null

# 3) .smartnav 的設定與資料庫 —— 白名單，不是黑名單
for f in steering_trim.json nav_state.json \
         path_database/paths.json waypoint_database/waypoints.json \
         map_database/maps_db.json; do
  if [ -f "$HOME/.smartnav/$f" ]; then
    mkdir -p "$STAGE/smartnav/$(dirname "$f")"
    cp "$HOME/.smartnav/$f" "$STAGE/smartnav/$f"
  fi
done

# 4) 今天建的地圖
find "$HOME/.smartnav/map_database" -maxdepth 1 \( -name '*.pgm' -o -name '*.yaml' \) \
     -newermt '-24 hours' -exec cp {} "$STAGE/smartnav/map_database/" \; 2>/dev/null

# ── 資安複查：確認排除項真的不在包裡 ────────────────────────
echo "── 資安複查 ──"
BAD=0
for pat in secrets face_database user_registry token api_key; do
  HIT=$(find "$STAGE" -iname "*${pat}*" 2>/dev/null)
  if [ -n "$HIT" ]; then
    echo "  ✗ 發現不該打包的：$HIT"; BAD=1
  fi
done
[ "$BAD" = 0 ] && echo "  ✓ 無 secrets / face_database / token"

echo
echo "── 打包清單 ──"
(cd "$STAGE" && find . -type f | sed 's|^\./|  |' | sort | head -40)
TOTAL=$(find "$STAGE" -type f | wc -l)
echo "  ... 共 $TOTAL 個檔案"
echo

if [ "$BAD" != 0 ]; then
  echo "★ 資安複查沒過，已中止打包。"; exit 1
fi

tar --exclude='*secrets*' --exclude='*face_database*' \
    -czf "$OUT" -C "$STAGE" . 2>/dev/null

SIZE=$(du -h "$OUT" | cut -f1)
MD5=$(md5sum "$OUT" | cut -d' ' -f1)

cat <<EOF

✓ 完成：$OUT  ($SIZE)
  MD5: $MD5

───────────────────────────────────────────────────────
在【Windows 筆電】開一個終端機，貼這行把檔案拉回去：

  scp user@192.168.137.106:~/car_data_${DAY}.tar.gz "\$HOME/Desktop/專題/車子/"

拉完在筆電上核對 MD5（要跟上面一樣）：

  certutil -hashfile "\$HOME/Desktop/專題/車子/car_data_${DAY}.tar.gz" MD5
───────────────────────────────────────────────────────

快速自我檢查 —— 有 FAIL 的項目：
EOF
grep -h $'\tFAIL\t' "$HOME"/maprun/verify_out/*/results.tsv 2>/dev/null \
  | awk -F'\t' '{printf "  ✗ %s = %s\n     %s\n", $1, $3, $4}' \
  || echo "  （沒有 FAIL）"
echo
