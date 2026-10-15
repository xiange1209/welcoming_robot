#!/usr/bin/env bash
# 打包驗證結果與原始數據，並印出 scp 指令。
#
# ★★ 資安：以下兩項【絕對不打包】
#      ~/.smartnav/secrets/       Telegram token
#      ~/.smartnav/face_database/ 人臉特徵向量與姓名（個資）
#    打包前會把清單印出來讓你過目，有疑慮就 Ctrl-C。
#
# 用法：
#   ~/maprun/verify/collect_data.sh                      今天的資料（只收「近 24 小時」內有改動的 log／CSV）
#   ~/maprun/verify/collect_data.sh 20261022             指定日期（CSV 以檔名含該日期篩，log 仍只收近 24 小時）
#   ~/maprun/verify/collect_data.sh --all                ★ 全部帶走（不篩時間、全部 CSV）——要拉「前幾天」的資料用這個
#   ~/maprun/verify/collect_data.sh --since='-10 days'   近 10 天有改動的 log／CSV（find -newermt 的時間描述）
#   --all／--since 產生的檔名是 car_data_all_<日期>／car_data_since_<日期>，不會和每日包混淆。
#
# ★ 2026-10-15 改版（原因都是實際踩到或查到的）：
#   1. 原本固定只收「近 24 小時」→ 下次上車時拉不到上週的資料。加 --all／--since。
#   2. 轉向實驗 record_steering_chain.py 預設輸出在家目錄 ~/steer_diag_*.csv，原本不在任何收集範圍。
#   3. 加 system_snapshot.txt：時鐘同步、溫度、vcgencmd get_throttled（欠壓／降頻旗標，全專案之前沒有任何一筆紀錄）、
#      記憶體、磁碟、USB 裝置、套件版本（PACKAGE_VERSION.txt，pack_car_code.py 打包時寫入）。
#   4. 加 MANIFEST.sha256（每個檔的雜湊），拉回去後可以核對沒有缺漏或損毀。

set -uo pipefail

DAY=""; SINCE='-24 hours'; ALL=0; SINCE_SET=0
for a in "$@"; do
  case "$a" in
    --all) ALL=1 ;;
    --since=*) SINCE="${a#--since=}"; SINCE_SET=1 ;;
    20[0-9][0-9][01][0-9][0-3][0-9]) DAY="$a" ;;
    -h|--help) sed -n '2,18p' "$0"; exit 0 ;;
    *) echo "不認得的參數：$a（用法見檔頭，或加 --help）"; exit 2 ;;
  esac
done
DAY="${DAY:-$(date +%Y%m%d)}"
if [ "$SINCE_SET" = 1 ] && ! date -d "$SINCE" >/dev/null 2>&1; then
  echo "✗ --since 的時間描述無法解析：'$SINCE'（例：--since='-10 days'）"; exit 2
fi
if [ "$ALL" = 1 ]; then TAG="all_${DAY}"; TF=(); CSV_PAT="*.csv"
elif [ "$SINCE_SET" = 1 ]; then TAG="since_${DAY}"; TF=(-newermt "$SINCE"); CSV_PAT="*.csv"
else TAG="${DAY}"; TF=(-newermt "$SINCE"); CSV_PAT="*${DAY}*.csv"
fi
OUT="$HOME/car_data_${TAG}.tar.gz"
STAGE=$(mktemp -d)
trap 'rm -rf "$STAGE"' EXIT

echo "打包 $DAY 的資料（模式：$([ "$ALL" = 1 ] && echo '全部，不篩時間' || echo "只收 $SINCE 之後有改動的 log／CSV")）"
echo
# ★ 2026-09-30（車子端 9/29 發現）：Pi 沒有 RTC，開機後到 NTP 同步前時鐘是錯的（實測 00:54 → 10:02）。
#   「近 N 小時」是用檔案時間篩的，同步前跑會漏檔；預設日期過了午夜也會變成隔天 —— 那時請明寫日期，或用 --all。
if [ "$ALL" = 0 ] && ! timedatectl show -p NTPSynchronized --value 2>/dev/null | grep -q yes; then
  echo "⚠ 時鐘還沒同步（timedatectl）：「近 N 小時」的 log 可能漏收。等同步後再跑，或改用 --all。"
  echo
fi

mkdir -p "$STAGE/verify" "$STAGE/logs" "$STAGE/smartnav" "$STAGE/steer_diag"

# 1) 驗證結果
if [ -d "$HOME/maprun/verify_out" ]; then
  cp -r "$HOME"/maprun/verify_out/* "$STAGE/verify/" 2>/dev/null
fi

# 2) CSV 與 log（研究數據要留著寫報告，全部帶走）
find "$HOME/maprun/logs" -maxdepth 1 \( -name "$CSV_PAT" -o -name "*.log" \) \
     "${TF[@]}" -exec cp {} "$STAGE/logs/" \; 2>/dev/null
# 2b) ★ 2026-09-29：9/26 起平板（LaunchSupervisor）起的單元，輸出寫在 ~/.smartnav/logs/system_control/，
#     不在 ~/maprun/logs —— 導航、語音、人臉等當天的 log 都在這裡，不收的話 V5 要的 nav log 帶不回來
if [ -d "$HOME/.smartnav/logs/system_control" ]; then
  mkdir -p "$STAGE/logs/system_control"
  find "$HOME/.smartnav/logs/system_control" -maxdepth 1 -name "*.log" \
       "${TF[@]}" -exec cp {} "$STAGE/logs/system_control/" \; 2>/dev/null
fi
# 2c) ★ 2026-10-15：轉向實驗（record_steering_chain.py）預設輸出在家目錄 ~/steer_diag_*.csv
find "$HOME" -maxdepth 1 -name 'steer_diag_*' \( -name '*.csv' -o -name '*.json' -o -name '*.md' \) \
     "${TF[@]}" -exec cp {} "$STAGE/steer_diag/" \; 2>/dev/null

# 3) .smartnav 的設定與資料庫 —— 白名單，不是黑名單
for f in steering_trim.json nav_state.json \
         path_database/paths.json waypoint_database/waypoints.json \
         map_database/maps_db.json; do
  if [ -f "$HOME/.smartnav/$f" ]; then
    mkdir -p "$STAGE/smartnav/$(dirname "$f")"
    cp "$HOME/.smartnav/$f" "$STAGE/smartnav/$f"
  fi
done

# 4) 地圖
# ★ 2026-09-30（車子端 9/29 發現）：地圖實際存在 map_database/data/，原本 -maxdepth 1 **永遠**打包不到。
#   也不再用「近 24 小時」篩：一張圖約 40 KB，全帶最保險（時鐘沒同步也不會漏）；路徑照 repo 的 .smartnav/ 結構放
if [ -d "$HOME/.smartnav/map_database/data" ]; then
  mkdir -p "$STAGE/smartnav/map_database/data"
  find "$HOME/.smartnav/map_database/data" -maxdepth 1 \( -name '*.pgm' -o -name '*.yaml' \) \
       -exec cp {} "$STAGE/smartnav/map_database/data/" \; 2>/dev/null
fi

# 5) ★ 2026-10-15：套件版本與系統快照（不含環境變數，避免帶出 token）
[ -f "$HOME/maprun/PACKAGE_VERSION.txt" ] && cp "$HOME/maprun/PACKAGE_VERSION.txt" "$STAGE/PACKAGE_VERSION.txt"
{
  echo "# 系統快照 $(date '+%F %T %z')"
  echo "hostname: $(hostname)"; uname -a
  echo "--- 時鐘 ---"; timedatectl 2>/dev/null | head -8
  echo "--- 溫度／降頻旗標（get_throttled：0x0＝沒有欠壓或降頻；bit0 目前欠壓、bit16 曾欠壓、bit2 目前降頻、bit18 曾降頻）---"
  (vcgencmd measure_temp; vcgencmd get_throttled) 2>/dev/null || echo "vcgencmd 不可用"
  echo "--- load ---"; cat /proc/loadavg
  echo "--- 記憶體 ---"; free -m
  echo "--- 磁碟 ---"; df -h / | tail -2
  echo "--- USB 裝置 ---"; lsusb 2>/dev/null || echo "lsusb 不可用"
  echo "--- 網路介面 ---"; ip -br addr 2>/dev/null || echo "ip 不可用"
  echo "--- 套件版本 ---"; cat "$HOME/maprun/PACKAGE_VERSION.txt" 2>/dev/null || echo "（沒有 ~/maprun/PACKAGE_VERSION.txt：這個包不是用新版 pack_car_code.py 打的）"
} > "$STAGE/system_snapshot.txt" 2>&1
cat > "$STAGE/COLLECT_INFO.txt" <<EOF
collect_data.sh 參數：DAY=$DAY ALL=$ALL SINCE=$SINCE（SINCE_SET=$SINCE_SET）
收集時間：$(date '+%F %T %z')
EOF

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

# 每個檔的雜湊（拉回去後可核對有沒有缺漏或損毀）
(cd "$STAGE" && find . -type f ! -name MANIFEST.sha256 -print0 | sort -z | xargs -0 sha256sum > MANIFEST.sha256)

tar --exclude='*secrets*' --exclude='*face_database*' \
    -czf "$OUT" -C "$STAGE" . 2>/dev/null

SIZE=$(du -h "$OUT" | cut -f1)
MD5=$(md5sum "$OUT" | cut -d' ' -f1)
SHA=$(sha256sum "$OUT" | cut -d' ' -f1)
PI_IP=$(hostname -I 2>/dev/null | awk '{print $1}')
PI_IP="${PI_IP:-<Pi的IP>}"

cat <<EOF

✓ 完成：$OUT  ($SIZE，$TOTAL 個檔案)
  MD5:    $MD5
  SHA256: $SHA

───────────────────────────────────────────────────────
在【Windows 筆電】開一個終端機（PowerShell），貼這行把檔案拉回去
（研究數據一律放 backups，不要放進 車子/ 的 git 資料夾）：

  scp $(whoami)@${PI_IP}:~/car_data_${TAG}.tar.gz "\$HOME/Desktop/專題/backups/研究數據/"

拉完在筆電上核對雜湊（要跟上面一樣）：

  certutil -hashfile "\$HOME/Desktop/專題/backups/研究數據/car_data_${TAG}.tar.gz" SHA256
───────────────────────────────────────────────────────

快速自我檢查 —— 有 FAIL 的項目：
EOF
grep -h $'\tFAIL\t' "$HOME"/maprun/verify_out/*/results.tsv 2>/dev/null \
  | awk -F'\t' '{printf "  ✗ %s = %s\n     %s\n", $1, $3, $4}' \
  || echo "  （沒有 FAIL）"
echo
