import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

nav2_bringup_dir = get_package_share_directory("nav2_bringup")
pkg_share_dir = get_package_share_directory("smartnav_navigation")


def generate_launch_description():
    param_file = os.path.join(pkg_share_dir, "config", "burger.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
            ),
            # 啟動 Nav2 導航核心
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")),
                launch_arguments={
                    "params_file": param_file,
                    "use_sim_time": use_sim_time,
                    "log_level": "warn",
                }.items(),
            ),
            # 啟動 RViz2 介面
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, "launch", "rviz_launch.py")),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "log_level": "warn",
                }.items(),
            ),
        ]
    )
