#!/bin/bash
source /home/user/maprun/env.sh
NAME="${1:-home_map}"
exec ros2 action send_goal /create_map smartnav_msgs/action/CreateMap \
  "{map_name: '$NAME'}" > "$LOGDIR/create_map.log" 2>&1
