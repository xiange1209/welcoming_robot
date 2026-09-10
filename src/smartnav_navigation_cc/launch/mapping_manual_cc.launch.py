"""手動遙控建圖 (_cc)

這是最單純、也最不容易出錯的建圖路徑：只有 slam_toolbox，沒有 nav2、
沒有 amcl、沒有 map_server、沒有自動探索。/map 與 map -> odom_combined
都只有 slam_toolbox 一個發布者，所以不可能出現舊版那種
「兩張大小不同的地圖搶同一個 static_layer」的問題。

前置：底盤與雷達要先起來
    ros2 launch turn_on_wheeltec_robot wheeltec_sensors.launch.py

啟動建圖：
    ros2 launch smartnav_navigation_cc mapping_manual_cc.launch.py

另開一個終端機遙控 (teleop_twist_keyboard 發的是 geometry_msgs/Twist 到 /cmd_vel，
跟 wheeltec_robot_node 訂閱的型別一致)：
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

    阿克曼車不能原地轉：'j' / 'l' 只有在同時有前進或後退速度時才會轉向。
    請用 'i' 前進搭配 'j'/'l' 轉向，速度建議壓在 0.3 m/s 以下建圖品質比較穩。

存檔 (存到 SmartNav 的地圖資料庫，之後 HMI 就看得到)：
    ros2 run nav2_map_server map_saver_cli -f ~/.smartnav/map_database/data/<map_id>
    再把 <map_id> 寫進 ~/.smartnav/map_database/maps_db.json 與 ~/.smartnav/nav_state.json，
    或直接用本包提供的 /save_current_map 服務 (由 map_service_cc 提供，需跑 nav_bringup_cc)。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

PKG = "smartnav_navigation_cc"


def generate_launch_description():
    pkg_share = get_package_share_directory(PKG)
    slam_toolbox_share = get_package_share_directory("slam_toolbox")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    default_slam_params = os.path.join(pkg_share, "config", "slam_mapping_cc.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    slam_params_file = LaunchConfiguration("slam_params_file")

    # slam_toolbox 官方的 online_async_launch.py 已經正確處理了
    # LifecycleNode 的 configure -> activate 事件鏈，不要自己重寫。
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_toolbox_share, "launch", "online_async_launch.py")),
        launch_arguments={
            "slam_params_file": slam_params_file,
            "use_sim_time": use_sim_time,
            "autostart": "true",
            "use_lifecycle_manager": "false",
        }.items(),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_share, "launch", "rviz_launch.py")),
        condition=IfCondition(use_rviz),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("slam_params_file", default_value=default_slam_params),
            LogInfo(
                msg=(
                    "\n===== 手動遙控建圖已啟動 =====\n"
                    "另開終端機執行： ros2 run teleop_twist_keyboard teleop_twist_keyboard\n"
                    "阿克曼車無法原地旋轉，請用前進/後退搭配轉向。\n"
                    "存檔： ros2 run nav2_map_server map_saver_cli -f <輸出路徑>\n"
                    "=============================\n"
                )
            ),
            slam,
            rviz,
        ]
    )
