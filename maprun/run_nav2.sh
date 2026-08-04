#!/bin/bash
source /home/user/maprun/env.sh
exec ros2 launch smartnav_navigation nav2.launch.py use_rviz:=false use_sim_time:=false \
  > "$LOGDIR/nav2.log" 2>&1
