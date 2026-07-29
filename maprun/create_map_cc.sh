#!/bin/bash
# 觸發建圖 (需要 run_nav_cc.sh 已經在跑)
#
# 用法： ./create_map_cc.sh <地圖名稱>
#
# 自動探索結束的依據是 frontier_explorer 發出的 /exploration_complete，
# 不再是舊版比對 /rosout 字串的做法。
# 想提前收工： ros2 service call /finish_map std_srvs/srv/Trigger

source /home/user/maprun/env.sh

NAME="${1:-home_map}"

exec ros2 action send_goal /create_map smartnav_msgs/action/CreateMap \
  "{map_name: '$NAME'}" 2>&1 | tee "$LOGDIR/create_map_cc.log"
