#!/usr/bin/env bash
# 只換 HMI 前端（不重跑 colcon build）的部署：把新 dist 的檔案放進去、補 symlink-install 的逐檔連結、
# 確認每個新檔都讀得到（HTTP 200）之後，最後才換 index.html。
#
# ★ 2026-10-22 踩過：install/.../frontend/dist/assets/ 是 colcon --symlink-install 的「逐檔連結」，
#   新檔案只放進 src 的 dist 不會出現 → 先換 index.html 會讓平板整頁空白（新 JS 404）。
#
# 用法：deploy_hmi_dist_cc.sh <新建好的 dist 目錄>
set -eo pipefail
NEW="${1:?用法：deploy_hmi_dist_cc.sh <新建好的 dist 目錄>}"
WS="$HOME/welcoming_robot_ws"
SRC="$WS/src/smartnav_hmi/frontend/dist"
BLD="$WS/build/smartnav_hmi/frontend/dist"
INS="$WS/install/smartnav_hmi/share/smartnav_hmi/frontend/dist"
HMI="http://127.0.0.1:8080"
[ -f "$NEW/index.html" ] || { echo "✗ $NEW 底下沒有 index.html"; exit 1; }

added=0
for f in "$NEW"/assets/*; do
  b=$(basename "$f")
  if [ ! -e "$SRC/assets/$b" ]; then cp "$f" "$SRC/assets/$b"; added=$((added + 1)); fi
  [ -e "$BLD/assets/$b" ] || ln -s "$SRC/assets/$b" "$BLD/assets/$b"
  [ -e "$INS/assets/$b" ] || ln -s "$BLD/assets/$b" "$INS/assets/$b"
done
echo "新增 $added 個資產檔（舊的留著，還沒重載的平板照樣能用）"

# 新 index.html 引用的每個檔都要先讀得到
for u in $(grep -oE '/assets/[A-Za-z0-9_.-]+' "$NEW/index.html" | sort -u); do
  code=$(curl -s -o /dev/null -w '%{http_code}' "$HMI$u")
  [ "$code" = 200 ] || { echo "✗ $u → HTTP $code，不換 index.html"; exit 1; }
done
echo "✓ 新 index.html 引用的檔案都是 200"

cp -a "$SRC/index.html" "$SRC/index.html.bak_$(date +%Y%m%d_%H%M%S)"
cp "$NEW/index.html" "$SRC/index.html"
echo "✓ 已換 index.html：$(grep -oE 'index-[A-Za-z0-9_-]+\.js' "$SRC/index.html")（備份在同目錄 index.html.bak_*）"
