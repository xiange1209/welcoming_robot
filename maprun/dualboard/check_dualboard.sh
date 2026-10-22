#!/bin/bash
# 雙板連線檢查（唯讀，不改任何設定）。在 A 或 B 上跑：~/maprun/dualboard/check_dualboard.sh
#
# 檢查四件事：①網路線（速度、雙工、掉包）②頻寬（對方要先開 iperf3 -s）③校時 ④DDS 跨板看得到對方的節點
# 通過標準照交接文件 §5-10：1000Mb/s Full、ping 1000 次 0% 掉包、iperf3 雙向 ≥ 300 Mbit/s；
# 另加：B 的校時 offset 要在毫秒等級，④的節點全部 ✓。
# ★ 2026-10-22 車上 agent 起草，只做過 bash -n 與「eth0 沒有 IP」拒絕分支的實跑。
ME=$(ip -4 -br addr show eth0 2>/dev/null | awk '{print $3}' | cut -d/ -f1)
case "$ME" in
  10.10.10.1) ROLE=A; PEER=10.10.10.2; EXPECT="face_node user_auth_node voice_trigger_node speech_recognizer_node camera" ;;
  10.10.10.2) ROLE=B; PEER=10.10.10.1; EXPECT="hmi_server_node wheeltec_robot controller_server" ;;
  *) echo "✗ eth0 沒有 10.10.10.x（現在：${ME:-沒有 IPv4}）→ 先跑 sudo ~/maprun/dualboard/nm_eth0_setup.sh A（或 B）"; exit 1 ;;
esac
echo "══ 本機 $ROLE（$ME）→ 對方 $PEER　$(date '+%F %T') ══"

echo "── ① 網路線 ──"
ethtool eth0 2>/dev/null | grep -E "Speed|Duplex|Link detected" | sed 's/^\s*/  /'
# 先試 3 次：對方不在線時，1000 次 ping 會因為等 ARP 拖過兩分鐘（10/22 車上實測被逾時砍掉）
if ping -c 3 -W 1 -q "$PEER" >/dev/null 2>&1; then
  ping -c 1000 -i 0.01 -w 30 -q "$PEER" 2>&1 | grep -E "packet loss|rtt" | sed 's/^/  /'
else
  echo "  ✗ 對方 $PEER 不通（網路線沒插、對方沒開機或 IP 沒設）→ 跳過 1000 次 ping 與頻寬測試"
  PEER_DOWN=1
fi

echo "── ② 頻寬 ──"
if [ -n "${PEER_DOWN:-}" ]; then
  echo "  （對方不通，略過）"
elif command -v iperf3 >/dev/null; then
  echo "  本機→對方：$(timeout 20 iperf3 -c "$PEER" -t 5 2>&1 | grep -E 'receiver$' | awk '{print $(NF-2), $(NF-1)}')"
  echo "  對方→本機：$(timeout 20 iperf3 -c "$PEER" -t 5 -R 2>&1 | grep -E 'receiver$' | awk '{print $(NF-2), $(NF-1)}')"
  echo "  （空白＝對方沒開 iperf3 -s）"
else
  echo "  iperf3 沒裝：sudo apt install iperf3（兩邊都要；對方先開 iperf3 -s）"
fi

echo "── ③ 校時 ──"
if [ "$ROLE" = A ]; then
  if command -v chronyc >/dev/null; then
    chronyc tracking 2>/dev/null | grep -E "Reference ID|System time|Leap status" | sed 's/^/  /'
    echo "  向 A 校時的客戶端："; chronyc clients 2>/dev/null | tail -n +3 | sed 's/^/    /'
  else
    echo "  ✗ A 沒裝 chrony（見 chrony/50-dualboard-server.conf）"
  fi
else
  timedatectl timesync-status 2>/dev/null | grep -E "Server|Offset|Packet count" | sed 's/^/  /'
fi
echo "  NTPSynchronized=$(timedatectl show -p NTPSynchronized --value)"

echo "── ④ DDS 跨板 ──"
set +u
source /home/user/maprun/dualboard/env_dualboard.sh >/dev/null 2>&1
NODES=$(timeout 30 ros2 node list --no-daemon --spin-time 5 2>&1)
for n in $EXPECT; do
  if echo "$NODES" | grep -q "$n"; then echo "  ✓ 看得到對方的 $n"; else echo "  ✗ 看不到 $n"; fi
done
echo "  （本機 ROS_DOMAIN_ID=$ROS_DOMAIN_ID；CYCLONEDDS_URI=$CYCLONEDDS_URI；共看到 $(echo "$NODES" | grep -c '^/') 個節點）"
echo "$NODES" | grep -iE "error|failed|does not match" | head -3 | sed 's/^/  ⚠ /'

echo "── ⑤ 環境一致（2026-10-22 筆電端加）──"
ROLE_FILE=/home/user/.smartnav/dualboard_role
echo "  標記檔 $ROLE_FILE：$( [ -f "$ROLE_FILE" ] && cat "$ROLE_FILE" || echo '沒有（source env.sh 會是單板環境！先跑 dualboard_mode.sh '"$ROLE"'）' )"
[ -f "$ROLE_FILE" ] && [ "$(head -c1 "$ROLE_FILE")" != "$ROLE" ] && echo "  ✗ 標記檔寫的角色跟 eth0 的位址（$ROLE）不一樣"
WANT=$( (set +u; source /home/user/maprun/dualboard/overlay.sh; echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_AUTOMATIC_DISCOVERY_RANGE=$ROS_AUTOMATIC_DISCOVERY_RANGE CYCLONEDDS_URI=$CYCLONEDDS_URI SMARTNAV_DUALBOARD=$SMARTNAV_DUALBOARD") )
if [ "$ROLE" = A ]; then
  # HMI 是 systemd 服務，環境寫在服務裡（不 source env.sh）：逐項比對它拿到的值
  SVC=$(systemctl show smartnav-hmi -p Environment --value 2>/dev/null)
  for kv in $WANT; do
    if echo "$SVC" | grep -qF -- "$kv"; then echo "  ✓ HMI 服務有 $kv"; else echo "  ✗ HMI 服務沒有 $kv（HMI 會看不到 B；裝 systemd/smartnav-hmi.service.d 再 daemon-reload＋restart）"; fi
  done
else
  # B 的感知服務開機 source env_dualboard.sh（強制套用）；這裡確認手動開的終端機也會吃到
  (set +u; source /home/user/maprun/env.sh >/dev/null 2>&1
   for kv in $WANT; do k=${kv%%=*}; [ "$(printenv "$k")" = "${kv#*=}" ] && echo "  ✓ source env.sh 之後 $kv" || echo "  ✗ source env.sh 之後 $k 不是 ${kv#*=}（標記檔沒寫？）"; done)
fi
