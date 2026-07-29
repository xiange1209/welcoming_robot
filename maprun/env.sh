#!/bin/bash
# 建圖用的共通環境設定
# - CYCLONEDDS_URI: ParticipantIndex=none，避免 nav2 大量節點耗盡 participant 索引
# - ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST: 這個子網路上還有別台機器在跑同一套 stack
#   (曾經看到 3 個 /waypoint_service_node、3 個 /navigation_service_node)，
#   不隔離的話 map_service 可能會呼叫到遠端的 /control_exploration。

source /opt/ros/jazzy/setup.bash
source /home/user/welcoming_robot_ws/install/setup.bash

export CYCLONEDDS_URI=file:///home/user/cyclonedds.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

export LOGDIR=/home/user/maprun/logs
mkdir -p "$LOGDIR"
