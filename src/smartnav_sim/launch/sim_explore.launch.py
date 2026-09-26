"""自動建圖模擬 —— 模擬車＋實車同一套導航鏈（建圖＋frontier 探索）

一行跑完整趟：
    ros2 launch smartnav_sim sim_explore.launch.py

做的事：
    1. sim_robot.launch.py：Gazebo 世界、模擬車、橋接、akm_realism
    2. smartnav_navigation_cc/mapping_explore_cc.launch.py（use_sim_time:=true）
       —— 跟實車 `mapping_explore_cc` 是**同一支**，參數檔也是同一份。
          模擬裡驗過的參數可以直接上車，這是這個套件存在的理由。
    3. 等 nav2 的 bt_navigator 進到 active，再自動送 /create_map（地圖名 sim_<時間>）
       —— 等同在 HMI 按「建立地圖」。auto_explore:=false 則不送，自己用指令觸發。

看結果：
    - 地圖存在 WSL 的 ~/.smartnav/map_database/（跟車上同一套存法，不會碰到車上的資料）
    - 卡住、看門狗、預算的 log 跟車上一樣，出發清單的 V5 判讀方式可以照用
    - 提前收工：ros2 service call /finish_map std_srvs/srv/Trigger

其他參數見 sim_robot.launch.py（world、map_yaml、gui、realism、x/y/yaw…），這裡全部轉傳。

★ 已知差異（模擬「不會」出現的實車問題）：沒有 IMU 零偏、沒有 CPU 滿載造成的 TF 空窗
  （筆電比 Pi4 快很多）、沒有人跟在車後。模擬過了不代表車上一定過，但模擬沒過的，車上一定不會過。
★ stuck_detector 與 steering_trim 用的是牆上時鐘（time.monotonic）。模擬即時率 < 1 時，
  它們會覺得車子「比指令慢」—— 先看 Gazebo 的 RTF（gz topic -e -t /stats），低於 0.9 的卡住判定不可信。
"""

import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

PKG = "smartnav_sim"
PASS_THROUGH = ["world", "map_yaml", "gui", "render_engine", "realism", "x", "y", "yaw", "lidar_x_actual"]


def generate_launch_description():
    share = get_package_share_directory(PKG)
    nav_share = get_package_share_directory("smartnav_navigation_cc")
    map_name = "sim_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    sim_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "sim_robot.launch.py")),
        launch_arguments={k: LaunchConfiguration(k) for k in PASS_THROUGH}.items(),
    )

    # 晚幾秒再起導航：讓 /clock 與 odom_combined -> base_footprint 先出來。
    # 不等也不會壞（map_service_cc 會等 TF），只是開頭會洗一堆「等 TF」的警告
    nav = TimerAction(
        period=LaunchConfiguration("nav_delay"),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav_share, "launch", "mapping_explore_cc.launch.py")),
            launch_arguments={"use_sim_time": "true", "use_rviz": LaunchConfiguration("use_rviz")}.items(),
        )],
    )

    # 用 bt_navigator 的生命週期狀態當「nav2 起好了」的訊號，比固定等幾秒可靠
    # （map_service_cc 在模式就緒後才叫 nav2 STARTUP，筆電上約 10~20 秒）
    trigger = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("auto_explore")),
        cmd=["bash", "-c",
             "until ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q '^active'; do sleep 2; done; "
             f"echo '[smartnav_sim] nav2 已就緒，送出 /create_map（{map_name}）'; "
             "ros2 action send_goal /create_map smartnav_msgs/action/CreateMap "
             f"\"{{map_name: '{map_name}'}}\""],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="bottlenecks.sdf"),
            DeclareLaunchArgument("map_yaml", default_value=""),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("render_engine", default_value=""),
            DeclareLaunchArgument("realism", default_value="true"),
            DeclareLaunchArgument("x", default_value="auto"),
            DeclareLaunchArgument("y", default_value="auto"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument("lidar_x_actual", default_value=""),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("auto_explore", default_value="true"),
            DeclareLaunchArgument("nav_delay", default_value="5.0"),
            sim_robot,
            nav,
            trigger,
        ]
    )
