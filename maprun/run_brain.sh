#!/bin/bash
source /home/user/maprun/env.sh
exec ros2 launch smartnav_navigation brain.launch.py use_sim_time:=false \
  > "$LOGDIR/brain.log" 2>&1
