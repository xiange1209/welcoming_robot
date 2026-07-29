"""自動探索建圖 (_cc)

其實就是 nav_bringup_cc.launch.py 直接以建圖模式開機，並確保
frontier_explorer 有被啟動。分成獨立一支只是為了少打幾個參數。

前置：底盤與雷達要先起來
    ros2 launch turn_on_wheeltec_robot wheeltec_sensors.launch.py

啟動：
    ros2 launch smartnav_navigation_cc mapping_explore_cc.launch.py

觸發探索 (HMI 的「建立地圖」按鈕送的也是這個 action)：
    ros2 action send_goal /create_map smartnav_msgs/action/CreateMap "{map_name: 'my_map'}"

探索結束的判定是 frontier_explorer 發出的 /exploration_complete (std_msgs/Empty)，
不再像舊版那樣去比對 /rosout 的字串。地圖會自動存檔並切回 AMCL 定位模式。

想提前收工：
    ros2 service call /finish_map std_srvs/srv/Trigger
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

PKG = "smartnav_navigation_cc"


def generate_launch_description():
    pkg_share = get_package_share_directory(PKG)

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "nav_bringup_cc.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_rviz": use_rviz,
            "use_exploration": "true",
            "start_mode": "mapping",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            bringup,
        ]
    )
