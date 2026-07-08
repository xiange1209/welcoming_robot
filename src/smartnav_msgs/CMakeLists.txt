cmake_minimum_required(VERSION 3.8)
project(smartnav_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# 訊息定義
set(msg_files
  "msg/RecognitionResult.msg"
  "msg/AudioData.msg"
  "msg/MapInfo.msg"
  "msg/WaypointInfo.msg"
)

# 服務定義
set(srv_files
  "srv/RegisterFace.srv"
  "srv/ListMaps.srv"
  "srv/SwitchMap.srv"
  "srv/CreateWaypoint.srv"
  "srv/ListWaypoints.srv"
  "srv/GetWaypoint.srv"
)

# 動作定義
set(action_files
  "action/CreateMap.action"
  "action/GlobalLocalization.action"
  "action/Navigate.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_package()
