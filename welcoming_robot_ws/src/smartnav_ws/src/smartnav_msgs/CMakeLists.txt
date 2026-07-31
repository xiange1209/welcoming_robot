cmake_minimum_required(VERSION 3.8)
project(smartnav_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generate message files
set(msg_files
  "msg/AudioData.msg"
  "msg/FaceEmbedding.msg"
  "msg/MapInfo.msg"
  "msg/RegistrationProgress.msg"
  "msg/UserIdentity.msg"
  "msg/UserInfo.msg"
  "msg/UserType.msg"
  "msg/WaypointInfo.msg"
)

# Generate service files
set(srv_files
  "srv/CreateWaypoint.srv"
  "srv/DeleteMap.srv"
  "srv/DeleteUser.srv"
  "srv/DeleteWaypoint.srv"
  "srv/ExtractFaceEmbedding.srv"
  "srv/GetWaypoint.srv"
  "srv/ListMaps.srv"
  "srv/ListUsers.srv"
  "srv/ListWaypoints.srv"
  "srv/RegisterFace.srv"
  "srv/RegisterFacePhoto.srv"
  "srv/SwitchMap.srv"
  "srv/UpdateUser.srv"
)

# Generate action files
set(action_files
  "action/CreateMap.action"
  "action/GlobalLocalization.action"
  "action/Navigate.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs builtin_interfaces
)

ament_package()
