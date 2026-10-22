#!/bin/bash
# 幫板子取名字，讓 SSH 不會搞混。用法：sudo ~/maprun/dualboard/set_board_name.sh A   （4 GB 那塊用 B）
#   A＝8 GB（車上：底盤＋導航＋HMI）→ hostname smartnav-a，短名 car-a，網路線 10.10.10.1
#   B＝4 GB（人臉＋語音）           → hostname smartnav-b，短名 car-b，網路線 10.10.10.2
#
# 會做四件事（可以重複執行，不會重複加內容）：
#   1. cloud-init 覆寫：這張卡的 hostname 與 /etc/hosts 原本由 cloud-init 管（preserve_hostname: false、
#      user-data 寫死 wheeltec-r680），不覆寫的話下次開機會被改回去
#   2. /etc/hosts：127.0.1.1 換成新名字，加上 smartnav-a／smartnav-b 兩行
#   3. hostnamectl set-hostname
#   4. 使用者 user 的 ~/.bashrc 加「板子標示」：提示字元顯示 [A·8GB]／[B·4GB]（依實際記憶體判斷，SD 卡換板也不會標錯）
# 復原：刪 /etc/cloud/cloud.cfg.d/99-smartnav-hostname.cfg，/etc/hosts 與 ~/.bashrc 用 .bak_<日期> 還原，
#       sudo hostnamectl set-hostname wheeltec-r680
# ★ 2026-10-22 車上 agent：先在 B（4 GB 測試板）手動做過一遍、確認有效，再整理成這支；B 上重跑過一次確認不會重複加。
set -e
[ "$(id -u)" = 0 ] || { echo "要用 sudo 執行：sudo $0 A|B"; exit 1; }
case "${1:-}" in
  A) NAME=smartnav-a ;;
  B) NAME=smartnav-b ;;
  *) echo "用法：sudo $0 A|B"; exit 1 ;;
esac
MEM=$(awk '/MemTotal/{printf "%d", $2/1048576 + 0.5}' /proc/meminfo)
if { [ "$1" = A ] && [ "$MEM" != 8 ]; } || { [ "$1" = B ] && [ "$MEM" != 4 ]; }; then
  echo "⚠ 這塊板子是 ${MEM} GB，但你要設成 $1（A 應是 8 GB、B 應是 4 GB）。確定的話 10 秒內按 Ctrl-C 取消，否則繼續。"
  sleep 10
fi
STAMP=$(date +%Y%m%d)
U=/home/user

# 1. cloud-init 覆寫
cat > /etc/cloud/cloud.cfg.d/99-smartnav-hostname.cfg <<'EOF'
# 2026-10-22 雙板：保留手動設定的 hostname（smartnav-a／smartnav-b），不要每次開機改回 user-data 的 wheeltec-r680
# 復原：刪掉這個檔（下次開機 cloud-init 會把 hostname 與 /etc/hosts 改回原樣）
preserve_hostname: true
manage_etc_hosts: false
EOF
echo "✓ cloud-init 覆寫：/etc/cloud/cloud.cfg.d/99-smartnav-hostname.cfg"

# 2. /etc/hosts
[ -e "/etc/hosts.bak_$STAMP" ] || cp -p /etc/hosts "/etc/hosts.bak_$STAMP"
if grep -q '^127\.0\.1\.1' /etc/hosts; then
  sed -i "s/^127\.0\.1\.1 .*/127.0.1.1 $NAME $NAME/" /etc/hosts
else
  echo "127.0.1.1 $NAME $NAME" >> /etc/hosts
fi
grep -q 'smartnav-a car-a' /etc/hosts || printf '%s\n' "" \
  "# 雙板（2026-10-22）：A＝8 GB（車上：底盤＋導航＋HMI）、B＝4 GB（人臉＋語音）" \
  "10.10.10.1   smartnav-a car-a" "10.10.10.2   smartnav-b car-b" >> /etc/hosts
echo "✓ /etc/hosts（備份 /etc/hosts.bak_$STAMP）"

# 3. hostname
hostnamectl set-hostname "$NAME"
echo "✓ hostname → $(hostname)"

# 4. 板子標示（~/.bashrc）
if grep -q '>>> 2026-10-22 板子標示' "$U/.bashrc"; then
  echo "✓ ~/.bashrc 已有板子標示，不重複加"
else
  [ -e "$U/.bashrc.bak_$STAMP" ] || cp -p "$U/.bashrc" "$U/.bashrc.bak_$STAMP"
  cat >> "$U/.bashrc" <<'EOF'

# >>> 2026-10-22 板子標示（雙板：A＝8 GB、B＝4 GB）>>>
# 依「實際記憶體」判斷是哪塊板子：SD 卡換到另一塊板子也不會標錯。hostname 跟著 SD 卡走，
# 提示字元的 [A·8GB]／[B·4GB] 和 hostname（smartnav-a／smartnav-b）對不起來，就是卡插錯板了。
# 移除：刪掉這兩行標記之間的內容（備份 ~/.bashrc.bak_<日期>）
__smartnav_board() {
  case "$(awk '/MemTotal/{printf "%d", $2/1048576 + 0.5}' /proc/meminfo)" in
    8) echo "A·8GB" ;;
    4) echo "B·4GB" ;;
    *) echo "?GB" ;;
  esac
}
case $- in *i*)
  PS1="[\$(__smartnav_board)] $PS1"
  echo "★ 這是 $(__smartnav_board) 板（hostname：$(hostname)）"
  ;;
esac
# <<< 2026-10-22 板子標示 <<<
EOF
  chown user:user "$U/.bashrc"
  echo "✓ ~/.bashrc 加上板子標示（備份 ~/.bashrc.bak_$STAMP）"
fi
echo "完成：$(hostname)（這塊是 ${MEM} GB）。開新的終端機就會看到 [A·8GB]／[B·4GB]。"
