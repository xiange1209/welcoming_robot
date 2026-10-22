#!/bin/bash
# 設定兩板之間網路線的固定 IP（NetworkManager）。
#   用法：sudo ~/maprun/dualboard/nm_eth0_setup.sh A     （B 板改成 B）
#   A（8 GB，車上，底盤＋導航＋HMI）＝ 10.10.10.1/24
#   B（4 GB，行動電源，人臉＋語音）  ＝ 10.10.10.2/24
#   不設閘道、不設 DNS：A 照舊用 Wi-Fi 上網與連平板／筆電；B 只有這條線。
#
# 會做三件事：①安裝 nm/60-dualboard-eth0.conf（沒插線也保有 IP）②建立（或重建）連線設定「dualboard-eth0」③啟用它。
# 不會動 Wi-Fi 的任何設定。
# 復原：sudo nmcli con delete dualboard-eth0
#       sudo rm /etc/NetworkManager/conf.d/60-dualboard-eth0.conf && sudo systemctl reload NetworkManager
# ★ 2026-10-22 車上 agent 起草，只做過 bash -n，尚未實機執行。
# ★ 筆電端：V3（拔線重開機位址還在）失敗時改用備案 eth0_static_fallback.sh。
set -e
[ "$(id -u)" = 0 ] || { echo "要用 sudo 執行：sudo $0 A|B"; exit 1; }
case "${1:-}" in
  A) IP=10.10.10.1 ;;
  B) IP=10.10.10.2 ;;
  *) echo "用法：sudo $0 A|B"; exit 1 ;;
esac
HERE="$(cd "$(dirname "$0")" && pwd)"

install -m 644 "$HERE/nm/60-dualboard-eth0.conf" /etc/NetworkManager/conf.d/60-dualboard-eth0.conf
systemctl reload NetworkManager

if nmcli -t -f NAME con show | grep -qx dualboard-eth0; then
  nmcli con delete dualboard-eth0
fi
nmcli con add type ethernet ifname eth0 con-name dualboard-eth0 \
  ipv4.method manual ipv4.addresses "$IP/24" ipv4.never-default yes ipv4.ignore-auto-dns yes \
  ipv6.method disabled connection.autoconnect yes connection.autoconnect-priority 100
nmcli con up dualboard-eth0 || echo "（沒插線時啟用可能失敗；ignore-carrier 生效的話位址仍會在，下一行看得到）"
ip -br addr show eth0
