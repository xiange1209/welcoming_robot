"""假機器人 (_cc) —— 沒有底座時取代 wheeltec_sensors.launch.py

提供 nav2 / slam_toolbox 需要的最小輸入：/scan、/odom_combined、
odom_combined -> base_footprint -> laser 的 TF。純 2D 幾何模擬，
不需要 Gazebo，Pi 4 跑得動。

用法：
    # 終端機 1
    ros2 launch smartnav_navigation_cc fake_robot_cc.launch.py
    # 終端機 2 (照常啟動導航)
    ros2 launch smartnav_navigation_cc nav_bringup_cc.launch.py start_mode:=mapping
    # 終端機 3 (遙控，或直接送導航目標)
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

阿克曼運動學是照實模擬的 (最小轉彎半徑 0.8 m，v=0 時無法轉向)，
所以在真車上跑不動的動作，在這裡一樣跑不動。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "smartnav_navigation_cc"


def generate_launch_description():
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    use_rviz = LaunchConfiguration("use_rviz")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")

    fake_robot = Node(
        package=PKG,
        executable="fake_robot_cc",
        name="fake_robot_cc_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "initial_x": initial_x,
                "initial_y": initial_y,
                "initial_yaw": initial_yaw,
                # 對齊 senior_akm 實車
                "min_turning_radius": 0.80,
                "max_linear_speed": 0.5,
            }
        ],
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_share, "launch", "rviz_launch.py")),
        condition=IfCondition(use_rviz),
        launch_arguments={"use_sim_time": "false"}.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("initial_x", default_value="0.0"),
            DeclareLaunchArgument("initial_y", default_value="0.0"),
            DeclareLaunchArgument("initial_yaw", default_value="0.0"),
            LogInfo(
                msg=(
                    "\n===== 假機器人已啟動 (無底座測試用) =====\n"
                    "虛擬房間 10x8 m，含隔間、門與家具。\n"
                    "接著啟動導航： ros2 launch smartnav_navigation_cc nav_bringup_cc.launch.py start_mode:=mapping\n"
                    "==========================================\n"
                )
            ),
            fake_robot,
            rviz,
        ]
    )
