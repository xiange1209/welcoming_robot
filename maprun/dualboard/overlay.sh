#!/bin/bash
# 雙板模式的 ROS 環境「覆蓋層」（A、B 共用）。不要直接執行，用 source。
#
# 誰會 source 它：
#   ① ~/maprun/env.sh：~/.smartnav/dualboard_role 存在時自動 source（所以 run_nav_cc.sh、run_node_cc.sh、verify_*.sh
#      等所有「先 source env.sh」的腳本都會吃到雙板環境；HMI 的「定位＋導航」單元就是走 run_nav_cc.sh）
#   ② env_dualboard.sh：不管有沒有標記檔都強制套用（B 的 systemd 感知服務、手動測試用）
#
# 做四件事：
#   1. DDS 只走兩板之間的網路線（eth0），不走 Wi-Fi —— Wi-Fi 網段上還有別台機器跑同一套系統
#   2. ROS_DOMAIN_ID 0 → 42：第二道隔離
#   3. ROS_AUTOMATIC_DISCOVERY_RANGE LOCALHOST → SUBNET：單板只在本機互找；雙板要跨過網路線
#   4. SMARTNAV_DUALBOARD=1、SMARTNAV_BOARD=A|B：給程式辨認「現在是雙板、我是哪一塊」
#
# ★ 這幾個值要與 systemd/smartnav-hmi.service.d/50-dualboard.conf 一模一樣（HMI 是 systemd 服務，不 source env.sh；
#   check_dualboard.sh ⑤ 會比對）。兩塊板子也必須一樣。
# ★ 2026-10-22 筆電端審查後新增（草稿是 env_dualboard.sh 一支）：原因見 README 第 9 節 ①。
_DB_DIR="${DUALBOARD_DIR:-/home/user/maprun/dualboard}"
_DB_ROLE_FILE="${SMARTNAV_ROLE_FILE:-/home/user/.smartnav/dualboard_role}"
export CYCLONEDDS_URI="file://${_DB_DIR}/cyclonedds_dualboard.xml"
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export SMARTNAV_DUALBOARD=1
if [ -f "$_DB_ROLE_FILE" ]; then
  export SMARTNAV_BOARD="$(head -c 1 "$_DB_ROLE_FILE")"
else
  export SMARTNAV_BOARD="${SMARTNAV_BOARD:-?}"
fi
unset _DB_DIR _DB_ROLE_FILE
