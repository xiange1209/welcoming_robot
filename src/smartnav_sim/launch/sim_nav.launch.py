"""模擬車＋定位模式導航（不開自動探索）—— 測教導重播、點地圖導航用。

    ros2 launch smartnav_sim sim_nav.launch.py gui:=false
    ros2 launch smartnav_sim sim_nav.launch.py self_echo_bars:=true   # 加上實車的鋁擠條自身回波

地圖與起始位姿照實車的規矩：map_service_cc 載入 ~/.smartnav/nav_state.json 的 last_active_map，
AMCL 從 last_pose 起算。用 sim_explore 建過圖的話，模擬車預設就在建圖時的起點 (0, 0) 出生，
所以把 last_pose 設成 (0, 0, 0) 就對得上（見 README「定位模式與重播測試」）。

導航起好之後（bt_navigator active），用 navigate 動作指定目標：
    ros2 action send_goal /navigate smartnav_msgs/action/Navigate \
      "{use_target_pose: true, target_pose: {position: {x: 2.0, y: 0.5}, orientation: {w: 1.0}}, target_name: '測試'}"
→ navigation_action_cc 先規劃一條教導路徑（_auto_測試_N）再交給 path_teach 純追蹤重播，跟實車點地圖導航同一條路。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

PKG = "smartnav_sim"
PASS_THROUGH = ["world", "map_yaml", "gui", "render_engine", "realism", "x", "y", "yaw",
                "lidar_x_actual", "self_echo_bars", "clock_rate_hz"]


def generate_launch_description():
    share = get_package_share_directory(PKG)
    nav_share = get_package_share_directory("smartnav_navigation_cc")

    sim_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "sim_robot.launch.py")),
        launch_arguments={k: LaunchConfiguration(k) for k in PASS_THROUGH}.items(),
    )
    # 跟 sim_explore 一樣晚幾秒再起導航，讓 /clock 與 odom_combined -> base_footprint 先出來
    nav = TimerAction(
        period=LaunchConfiguration("nav_delay"),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav_share, "launch", "nav_bringup_cc.launch.py")),
            launch_arguments={
                "use_sim_time": "true",
                "use_rviz": LaunchConfiguration("use_rviz"),
                "start_mode": LaunchConfiguration("start_mode"),
                "use_exploration": "false",
                "self_echo_filter": LaunchConfiguration("self_echo_filter"),
                "collision_scan_topic": LaunchConfiguration("collision_scan_topic"),
                # ★ 2026-10-08：掃參數（例如規劃器迴轉半徑）時指向改過的複本，不必動 repo 的 yaml
                "params_file": LaunchConfiguration("params_file"),
            }.items(),
        )],
    )
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="bottlenecks.sdf"),
        DeclareLaunchArgument("map_yaml", default_value=""),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("render_engine", default_value=""),
        DeclareLaunchArgument("realism", default_value="true"),
        DeclareLaunchArgument("x", default_value="auto"),
        DeclareLaunchArgument("y", default_value="auto"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("lidar_x_actual", default_value=""),
        DeclareLaunchArgument("self_echo_bars", default_value="false"),
        DeclareLaunchArgument("clock_rate_hz", default_value="100",
                              description="/clock 限流（Hz），0 = 不限流；見 clock_throttle_node.py"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("start_mode", default_value="localization",
                              description="localization（預設，載入上次的地圖）／auto／mapping"),
        DeclareLaunchArgument("nav_delay", default_value="5.0"),
        DeclareLaunchArgument("self_echo_filter", default_value="true",
                              description="false = path_teach 不過濾自身回波（重現 9/29 實車的卡住）"),
        DeclareLaunchArgument("collision_scan_topic", default_value="/scan",
                              description="/scan_nav = collision_monitor 也去掉自身回波"),
        DeclareLaunchArgument("params_file",
                              default_value=os.path.join(nav_share, "config", "nav2_senior_akm_cc.yaml"),
                              description="nav2 參數檔（預設＝導航套件的 nav2_senior_akm_cc.yaml）"),
        sim_robot,
        nav,
    ])
