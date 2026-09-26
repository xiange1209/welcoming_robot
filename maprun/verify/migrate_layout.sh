#!/usr/bin/env bash
# 一次性遷移：車上舊的巢狀結構 → 9/23 之後的攤平結構。
#
#   舊：~/welcoming_robot_ws/src/smartnav_ws/src/<套件>
#   新：~/welcoming_robot_ws/src/<套件>
#
# 為什麼一定要先跑這支，而不是直接解壓新的 src/：
#   1. 新舊兩份套件同時在 src/ 底下 → colcon 報 Duplicate package names
#   2. --symlink-install 的 install/ 裡是指向「舊路徑」的符號連結，
#      舊目錄一搬走就全部懸空 → 節點啟動時 import 失敗
#   3. build/ 的 CMake 快取寫死了舊的原始碼路徑
#
# 這支只「搬」不「刪」：舊的原始碼與建置產物都搬到 ~/layout_old_<時間>/，
# 確認新版跑得起來之後再自己決定要不要刪。
#
# 用法：~/maprun/verify/migrate_layout.sh

set -euo pipefail

WS="$HOME/welcoming_robot_ws"
OLD="$WS/src/smartnav_ws"
STAMP=$(date +%Y%m%d_%H%M%S)
KEEP="$HOME/layout_old_$STAMP"

echo "目錄結構遷移   $(date '+%Y-%m-%d %H:%M:%S')"
echo

[ -d "$WS" ] || { echo "✗ 找不到 $WS —— 這支要在車上（Pi）跑"; exit 1; }

if [ ! -d "$OLD" ]; then
  echo "✓ 沒有舊的 src/smartnav_ws/ —— 已經是新結構，不需要遷移。"
  exit 0
fi

echo "偵測到舊結構：$OLD"
echo "  裡面的套件：$(ls "$OLD/src" 2>/dev/null | tr '\n' ' ')"
echo

# ── 有節點在跑就先停 ─────────────────────────────────────────
# 只偵測、不殺（專案鐵則：不准用字串比對去 kill）。請用 HMI 或 kill_node_cc.sh 停。
RUNNING=$(ps -eo pid,args --no-headers | grep -E "ros2 (launch|run)|_node|/install/" \
          | grep -v grep | grep -v "$0" || true)
if [ -n "$RUNNING" ]; then
  echo "⚠ 還有 ROS 相關行程在跑（搬走 install/ 會讓它們崩潰）："
  echo "$RUNNING" | head -10 | sed 's/^/    /'
  echo
  echo "  請先在 HMI 把所有節點關掉，或用 ~/maprun/kill_node_cc.sh 逐一停止，再重跑這支。"
  exit 1
fi

# ── 空間檢查（備份壓縮檔 + 搬移都在同一個檔案系統，主要成本是備份）──
NEED_KB=$(du -sk "$OLD" | cut -f1)
FREE_KB=$(df -k "$HOME" | awk 'NR==2 {print $4}')
if [ "$FREE_KB" -lt $((NEED_KB * 2)) ]; then
  echo "✗ 空間不足：需要約 $((NEED_KB * 2 / 1024)) MB，剩 $((FREE_KB / 1024)) MB"
  exit 1
fi

echo "將會做的事："
echo "  1. 備份 $WS/src → ~/backup_layout_$STAMP.tar.gz"
echo "  2. 搬走舊原始碼   $OLD → $KEEP/smartnav_ws"
echo "  3. 搬走建置產物   build/ install/ log/ → $KEEP/"
echo "  ★ 全部只搬不刪。.smartnav/（人臉、地圖、路徑資料）完全不動。"
echo
read -r -p "確認執行？(y/N) " OK
[ "$OK" = y ] || { echo "已取消，什麼都沒動。"; exit 0; }

mkdir -p "$KEEP"

echo
echo "[1/3] 備份..."
tar -czf "$HOME/backup_layout_$STAMP.tar.gz" -C "$WS" src
echo "      ✓ ~/backup_layout_$STAMP.tar.gz ($(du -h "$HOME/backup_layout_$STAMP.tar.gz" | cut -f1))"

echo "[2/3] 搬走舊原始碼..."
mv "$OLD" "$KEEP/smartnav_ws"
echo "      ✓ → $KEEP/smartnav_ws"

echo "[3/3] 搬走建置產物..."
for d in build install log; do
  if [ -e "$WS/$d" ]; then
    mv "$WS/$d" "$KEEP/$d"
    echo "      ✓ $d/ → $KEEP/$d"
  fi
done

cat <<EOF

✓ 遷移完成。接下來：

  cd ~/welcoming_robot_ws && tar -xzf ~/car_code_<日期>.tar.gz src
  colcon build --symlink-install        # ★ 全部重建，RPi4 約需 10~20 分鐘
  source install/setup.bash
  ~/maprun/verify/verify_0_preflight.sh

確認新版一切正常後，舊東西可以自己刪：
  rm -rf $KEEP

⚠ 這幾支舊的研究工具寫死了舊路徑，遷移後不能直接跑（不影響本週驗證）：
    ~/maprun/tools_0814/offline_probe.py
    ~/maprun/tools_0817/mic_*_cc.py
EOF
