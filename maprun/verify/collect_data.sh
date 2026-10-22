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
#   --all／--since 產生的檔名是 car_data_all_<日期>_<時分>／car_data_since_<日期>_<時分>，不會和每日包混淆。
#   ★ 2026-10-22：所有檔名都加 <時分>。同一天打第二包會蓋掉第一包（拉回筆電時同名也會蓋掉），已經撞過兩次。
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
TAG="${TAG}_$(date +%H%M)"
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
# 2d) ★ 2026-10-21：語音除錯錄音（speech_recognizer 的 debug_wav_dir；交接檔用 ~/maprun/logs/asr_wav）。
#     10/20 只有辨識文字、沒有聲音，疊字與句首遺失的根因查不下去。每段約 30~200 KB。
#     導航修補的逐拍紀錄 ~/maprun/logs/replay_*_live_<日期>.csv 已經被上面 2) 的 *.csv 收到。
if [ -d "$HOME/maprun/logs/asr_wav" ]; then
  mkdir -p "$STAGE/logs/asr_wav"
  find "$HOME/maprun/logs/asr_wav" -maxdepth 1 -name '*.wav' "${TF[@]}" \
       -exec cp {} "$STAGE/logs/asr_wav/" \; 2>/dev/null
fi

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
  # ★ 2026-10-21：使用者帳號不在 video 群組，vcgencmd 讀不到 /dev/vcio（10/20 收尾快照就這樣漏掉）。
  #   依序試：vcgencmd → sudo -n（免密碼才成功、不會卡住）→ 韌體 sysfs（不需權限，車上 agent 10/20 驗證過）
  tv=$(vcgencmd get_throttled 2>/dev/null | grep -o '0x[0-9a-fA-F]*')
  [ -z "$tv" ] && tv=$(sudo -n vcgencmd get_throttled 2>/dev/null | grep -o '0x[0-9a-fA-F]*')
  [ -z "$tv" ] && [ -r /sys/devices/platform/soc/soc:firmware/get_throttled ] && \
      tv="0x$(cat /sys/devices/platform/soc/soc:firmware/get_throttled 2>/dev/null)"
  echo "throttled=${tv:-讀不到}"
  tz=$(cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null) && echo "溫度 $((tz / 1000)).$(( (tz % 1000) / 100 )) °C"
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
# ★ 2026-10-22：筆電走 Wi-Fi 熱點連進來，要給「預設路由那張網卡」的位址。
#   hostname -I 的第一個是 eth0（雙板網段 10.10.10.x，筆電連不到）——10/22 印出 10.10.10.2 給使用者就是這個。
PI_IP=$(ip -4 route get 1.1.1.1 2>/dev/null | awk '{for (i = 1; i <= NF; i++) if ($i == "src") { print $(i + 1); exit }}')
[ -n "$PI_IP" ] || PI_IP=$(ip -4 -o addr show wlan0 2>/dev/null | awk '{sub("/.*", "", $4); print $4; exit}')
[ -n "$PI_IP" ] || PI_IP=$(hostname -I 2>/dev/null | awk '{print $1}')
PI_IP="${PI_IP:-<Pi的IP>}"

cat <<EOF

✓ 完成：$OUT  ($SIZE，$TOTAL 個檔案)
  MD5:    $MD5
  SHA256: $SHA

───────────────────────────────────────────────────────
在【Windows 筆電】把檔案拉回去（研究數據一律放 backups，不要放進 車子/ 的 git 資料夾）。
★ 檔名本身就含時間（_<時分>），同一天打包多次不會互相蓋掉——Pi 上與筆電上都一樣。

  命令提示字元（cmd.exe）：
  scp $(whoami)@${PI_IP}:~/car_data_${TAG}.tar.gz "%USERPROFILE%\Desktop\專題\backups\研究數據\"
  certutil -hashfile "%USERPROFILE%\Desktop\專題\backups\研究數據\car_data_${TAG}.tar.gz" SHA256

  PowerShell：
  scp $(whoami)@${PI_IP}:~/car_data_${TAG}.tar.gz "\$HOME/Desktop/專題/backups/研究數據/"

（雜湊要跟上面的 SHA256 一樣）
───────────────────────────────────────────────────────

快速自我檢查 —— 近 24 小時驗證紀錄裡有 FAIL 的項目：
EOF
# ★ 2026-10-22：原本把 verify_out/*/results.tsv「所有歷史 run」的 FAIL 一起列出（9/22 電池、9/29 沒接硬體、
#   10/05 滿舵 0 度……），看起來像今天車子壞了。改成只看近 24 小時的 run，並標出是哪一次。
RECENT=$(find "$HOME"/maprun/verify_out -mindepth 2 -maxdepth 2 -name results.tsv -newermt '-24 hours' 2>/dev/null | sort)
if [ -z "$RECENT" ]; then
  echo "  （近 24 小時沒有跑過驗證腳本）"
else
  FAILS=$(for f in $RECENT; do
    awk -F'\t' -v run="$(basename "$(dirname "$f")")" '$2 == "FAIL" {printf "  ✗ [%s] %s = %s\n     %s\n", run, $1, $3, $4}' "$f"
  done)
  if [ -n "$FAILS" ]; then echo "$FAILS"; else echo "  （沒有 FAIL）"; fi
fi
echo
