#!/bin/bash
# 備案：V3 失敗（拔掉網路線後 eth0 的位址消失）時，改成「NetworkManager 不管 eth0，由 systemd 開機時用 ip 設固定位址」。
#
# 為什麼可靠：Linux 核心不會因為沒有載波（沒插線）而移除 eth0 上的 IPv4 位址；位址消失是 NetworkManager 替它撤掉的。
#            NM 不管這張網卡，位址就一直在，CycloneDDS 綁 eth0 就永遠建得起來（本機節點之間照樣能通）。
#
#   用法：sudo ~/maprun/dualboard/eth0_static_fallback.sh A|B     （A＝10.10.10.1、B＝10.10.10.2）
#   復原：sudo ~/maprun/dualboard/eth0_static_fallback.sh off      （刪掉單元與 NM 設定；再跑 nm_eth0_setup.sh 回主方案）
#   做的事：①NM 設定 61-dualboard-eth0-unmanaged.conf（eth0 不歸 NM 管）②systemd 單元 smartnav-eth0.service（開機設位址）
#          ③刪掉主方案的 NM 連線 dualboard-eth0（不然兩邊搶）。不動 Wi-Fi。
# ★ 2026-10-22 筆電端新增，只做過 bash -n，尚未實機執行。
set -e
[ "$(id -u)" = 0 ] || { echo "要用 sudo 執行：sudo $0 A|B|off"; exit 1; }
UNIT=/etc/systemd/system/smartnav-eth0.service
CONF=/etc/NetworkManager/conf.d/61-dualboard-eth0-unmanaged.conf
case "${1:-}" in
  A) IP=10.10.10.1 ;;
  B) IP=10.10.10.2 ;;
  off)
    systemctl disable --now smartnav-eth0.service 2>/dev/null || true
    rm -f "$UNIT" "$CONF"
    systemctl daemon-reload; systemctl reload NetworkManager
    echo "已還原備案。要回主方案：sudo $(dirname "$0")/nm_eth0_setup.sh A|B"; exit 0 ;;
  *) echo "用法：sudo $0 A|B|off"; exit 1 ;;
esac
nmcli con delete dualboard-eth0 2>/dev/null || true
printf '[keyfile]\nunmanaged-devices=interface-name:eth0\n' > "$CONF"
cat > "$UNIT" <<UNIT_EOF
[Unit]
Description=SmartNav 雙板：eth0 固定位址 $IP/24（沒插線也保有）
After=sys-subsystem-net-devices-eth0.device
Wants=sys-subsystem-net-devices-eth0.device
Before=network-pre.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/sbin/ip link set eth0 up
ExecStart=/sbin/ip addr replace $IP/24 dev eth0
ExecStop=/sbin/ip addr flush dev eth0

[Install]
WantedBy=multi-user.target
UNIT_EOF
systemctl daemon-reload
systemctl reload NetworkManager
systemctl enable --now smartnav-eth0.service
ip -br addr show eth0
echo "★ 接著重做 V3：拔線重開機後 ip -br addr show eth0 仍要有 $IP"
