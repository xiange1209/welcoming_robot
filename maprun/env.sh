#!/bin/bash
# 建圖用的共通環境設定
# - CYCLONEDDS_URI: ParticipantIndex=none，避免 nav2 大量節點耗盡 participant 索引
# - ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST: 這個子網路上還有別台機器在跑同一套 stack
#   (曾經看到 3 個 /waypoint_service_node、3 個 /navigation_service_node)，
#   不隔離的話 map_service 可能會呼叫到遠端的 /control_exploration。

source /opt/ros/jazzy/setup.bash
# 遷移後、第一次建置完成前 install/ 不存在 —— 不要每開一個 shell 就印一次 No such file（9/29 車子端）
[ -f /home/user/welcoming_robot_ws/install/setup.bash ] && source /home/user/welcoming_robot_ws/install/setup.bash

export CYCLONEDDS_URI=file:///home/user/cyclonedds.xml
# git clone 部署時 ~/cyclonedds.xml 不存在，但 repo 根目錄（= workspace）有一份一樣的
# （2026-09-25 比對 md5 相同）。少了它 nav2 節點一多就會耗盡 participant 索引而大量崩潰。
if [ ! -f /home/user/cyclonedds.xml ] && [ -f /home/user/welcoming_robot_ws/cyclonedds.xml ]; then
  export CYCLONEDDS_URI=file:///home/user/welcoming_robot_ws/cyclonedds.xml
fi
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

export LOGDIR=/home/user/maprun/logs
mkdir -p "$LOGDIR"

# ★★ 2026-10-22 雙板模式 ★★
# ~/.smartnav/dualboard_role（內容 A 或 B）存在 → 套用雙板的 DDS 設定（domain 42、只走 eth0、SUBNET）。
# 為什麼放在這裡：HMI 的「定位＋導航」單元走 run_nav_cc.sh，那支一開頭就 source 這個檔；
# 上面幾行會把 domain 洗回 0、範圍洗回 LOCALHOST，HMI（systemd，domain 42）就看不到自己開出來的導航。
# 所有 run_*.sh、verify_*.sh 都 source 這個檔，所以只在這裡判斷一次。
# 回到單板：~/maprun/dualboard/dualboard_mode.sh off（不存在標記檔時這段什麼都不做＝完全是單板行為）。
_DB_ROLE_FILE="${SMARTNAV_ROLE_FILE:-/home/user/.smartnav/dualboard_role}"
_DB_DIR="${DUALBOARD_DIR:-/home/user/maprun/dualboard}"
if [ -f "$_DB_ROLE_FILE" ] && [ -f "$_DB_DIR/overlay.sh" ]; then
  source "$_DB_DIR/overlay.sh"
fi
unset _DB_ROLE_FILE _DB_DIR
