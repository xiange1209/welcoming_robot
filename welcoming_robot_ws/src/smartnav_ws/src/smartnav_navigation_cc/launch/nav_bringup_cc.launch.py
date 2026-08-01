"""SmartNav 導航總啟動檔 (_cc 重寫版)

架構重點 —— 這是舊版最主要的修正：

  1. map -> odom_combined 這條 TF 在任何時刻「只有一個發布者」。
     建圖時是 slam_toolbox，導航時是 amcl，兩者互斥。
     amcl / map_server / slam_toolbox 全部以 unconfigured 啟動，
     完全交給 map_service_cc 一個節點管生命週期。
     nav2 的 lifecycle_manager 只管 nav2 自己的節點，絕對不碰這三個。

  2. /map 同樣只有一個發布者。
     建圖時 map_server 會被退到 unconfigured (不是只 deactivate)，
     才能真正銷毀它的 transient_local publisher；
     導航時 slam_toolbox 也會被退到 unconfigured。
     舊版少了後者，於是建完圖之後 map_server 與 slam_toolbox 同時在發 /map，
     global_costmap 的 static_layer 在兩張大小不同的地圖之間反覆 resize，
     車子一走出舊地圖範圍就變成 "Robot is out of bounds of the costmap"，
     規劃器接著一路噴 "Start occupied"。

  3. nav2 的 lifecycle_manager 用 autostart:=false。
     global_costmap 在 activate 時會等 map -> base_footprint 的 TF，
     那條 TF 要等 map_service_cc 決定好模式才會出現。
     改由 map_service_cc 在模式就緒後呼叫 /lifecycle_manager_navigation_cc/manage_nodes
     來啟動 nav2，就沒有開機競態了。

啟動範例：
    ros2 launch smartnav_navigation_cc nav_bringup_cc.launch.py
    ros2 launch smartnav_navigation_cc nav_bringup_cc.launch.py start_mode:=mapping
    ros2 launch smartnav_navigation_cc nav_bringup_cc.launch.py use_rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "smartnav_navigation_cc"

# nav2 自己的節點，由 lifecycle_manager_navigation_cc 管理。
# 刻意不含 route_server 與 docking_server：這台車沒有用到，
# 但它們在 Pi 4 上會各自吃掉一份 costmap 訂閱與執行緒。
NAV2_LIFECYCLE_NODES = [
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
    "velocity_smoother",
    "collision_monitor",
]


def generate_launch_description():
    pkg_share = get_package_share_directory(PKG)
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    default_params = os.path.join(pkg_share, "config", "nav2_senior_akm_cc.yaml")
    default_slam_params = os.path.join(pkg_share, "config", "slam_mapping_cc.yaml")
    default_explore_params = os.path.join(pkg_share, "config", "frontier_explore_cc.yaml")
    default_bt_xml = os.path.join(pkg_share, "behavior_trees", "navigate_to_pose_akm_cc.xml")
    default_bt_through_xml = os.path.join(pkg_share, "behavior_trees", "navigate_through_poses_akm_cc.xml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    use_exploration = LaunchConfiguration("use_exploration")
    auto_start_exploration = LaunchConfiguration("auto_start_exploration")
    params_file = LaunchConfiguration("params_file")
    slam_params_file = LaunchConfiguration("slam_params_file")
    explore_params_file = LaunchConfiguration("explore_params_file")
    bt_xml_file = LaunchConfiguration("bt_xml_file")
    bt_through_xml_file = LaunchConfiguration("bt_through_xml_file")
    start_mode = LaunchConfiguration("start_mode")
    log_level = LaunchConfiguration("log_level")

    declare_args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument(
            "use_exploration",
            default_value="true",
            description="是否啟動 frontier_explorer (仍然是 autostart:=false，只有 create_map 才會叫它跑)",
        ),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("slam_params_file", default_value=default_slam_params),
        DeclareLaunchArgument("explore_params_file", default_value=default_explore_params),
        DeclareLaunchArgument("bt_xml_file", default_value=default_bt_xml),
        DeclareLaunchArgument("bt_through_xml_file", default_value=default_bt_through_xml),
        DeclareLaunchArgument(
            "start_mode",
            default_value="auto",
            description="auto=有地圖就定位、沒地圖就進建圖待命; localization=強制定位; mapping=強制建圖",
        ),
        DeclareLaunchArgument(
            "auto_start_exploration",
            default_value="true",
            description=(
                "create_map 收到請求時是否立刻開始自動探索。"
                "設 false 則 explorer 節點仍在線但待命，改由 /start_exploration 手動觸發 —— "
                "適合『先遙控通過窄走廊、進到寬敞區域再交給自動探索』的流程"
            ),
        ),
        DeclareLaunchArgument("log_level", default_value="info"),
    ]

    common_ros_args = ["--ros-args", "--log-level", log_level]
    tf_remap = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # ------------------------------------------------------------------
    # Nav2 核心
    # ------------------------------------------------------------------
    nav2_nodes = [
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            arguments=common_ros_args,
            remappings=tf_remap + [("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            arguments=common_ros_args,
            remappings=tf_remap,
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            arguments=common_ros_args,
            remappings=tf_remap,
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            arguments=common_ros_args,
            remappings=tf_remap + [("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[
                params_file,
                {
                    "use_sim_time": use_sim_time,
                    # 阿克曼版行為樹：恢復動作不含 Spin。
                    # 兩棵樹都要換 —— bt_navigator 在 activate 時會把兩棵都載入並
                    # 檢查其中的 action server，漏掉 through_poses 會讓整個 nav2
                    # bringup 掛在 "Action server spin not available"。
                    "default_nav_to_pose_bt_xml": bt_xml_file,
                    "default_nav_through_poses_bt_xml": bt_through_xml_file,
                },
            ],
            arguments=common_ros_args,
            remappings=tf_remap,
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            arguments=common_ros_args,
            remappings=tf_remap,
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            arguments=common_ros_args,
            remappings=tf_remap + [("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_collision_monitor",
            executable="collision_monitor",
            name="collision_monitor",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            arguments=common_ros_args,
            remappings=tf_remap,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation_cc",
            output="screen",
            arguments=common_ros_args,
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    # 關鍵：不自動啟動。等 map_service_cc 把 map -> odom_combined
                    # 準備好之後，再由它呼叫 manage_nodes(STARTUP)。
                    "autostart": False,
                    "node_names": NAV2_LIFECYCLE_NODES,
                    # === bond 完全關閉 (2026-07-29) ===
                    #
                    # bond 是「lifecycle_manager 監控各節點是否還活著」的心跳機制。
                    # 之前已經把心跳週期從 0.1 放寬到 0.5 秒（27% -> 20%），
                    # 但剩下的成本在 bondcpp 自己的計時器裡，調週期救不了。
                    #
                    # 實測 Pi 4 在建圖+探索時 CPU idle 只剩 0.4%，
                    # TF 發布者被餓到出現 0.5 秒空窗，
                    # controller_server 每個目標都因為
                    # "Lookup would require extrapolation into the future" 而 abort，
                    # 車子完全動不了。這種情況下 20% 拿去做存活監控太奢侈。
                    #
                    # 0.0 = 停用 bond。代價是節點當掉時 lifecycle_manager 不會察覺 ——
                    # 但實測 slam_toolbox SIGABRT 時它本來也沒救回來，
                    # 而且滿載時 bond 反而容易誤判成當機。
                    "bond_timeout": 0.0,
                    "attempt_respawn_reconnection": False,
                }
            ],
        ),
    ]

    # ------------------------------------------------------------------
    # 地圖來源：三個節點都停在 unconfigured，由 map_service_cc 獨佔管理
    # ------------------------------------------------------------------
    map_source_nodes = [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            arguments=common_ros_args,
            remappings=tf_remap,
        ),
        Node(
            package="nav2_map_server",
            executable="map_saver_server",
            name="map_saver",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            arguments=common_ros_args,
            remappings=tf_remap,
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            arguments=common_ros_args,
            remappings=tf_remap,
        ),
        # slam_toolbox 吃的是 /scan_slam（車尾扇區已遮蔽），不是原始 /scan。
        # 建圖時操作者跟在車後走，人被掃進去會拖垮局部匹配、誘發錯誤的
        # 迴路閉合（實測 map->odom 單步跳 14~22 m）。costmap / collision_monitor
        # 仍然吃原始 /scan —— 避障必須看得到人。詳見 scan_filter_cc_node.py。
        Node(
            package=PKG,
            executable="scan_filter_cc",
            name="scan_filter_cc_node",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=common_ros_args,
        ),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_params_file, {"use_sim_time": use_sim_time}],
            arguments=common_ros_args,
            remappings=tf_remap,
        ),
        # map_saver 是 lifecycle node，但它跟地圖來源互斥性無關，
        # 交給 map_service_cc 一起 configure + activate 即可。
    ]

    # ------------------------------------------------------------------
    # SmartNav 應用層
    # ------------------------------------------------------------------
    # 轉向零位補償。實測這台車下 angular.z=0 仍會左偏約 6 度/公尺 (前輪機械零位偏移)，
    # 插在 velocity_smoother 與 collision_monitor 之間把它補回來。
    steering_trim = Node(
        package=PKG,
        executable="steering_trim_cc",
        name="steering_trim_cc_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # 卡住偵測：雷達裝在 0.11 m 高，比它矮的雜物 (電線、地毯邊、椅腳橫桿)
    # 完全掃不到，輪子卡住了 costmap 上還是一片空白。
    # 這裡用純運動學推論 —— 指令要求在動但里程計顯示沒動，就是卡住了，
    # 不需要任何額外感測器。
    stuck_detector = Node(
        package=PKG,
        executable="stuck_detector_cc",
        name="stuck_detector_cc_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # 教導-重現路徑。錄下人開過的位姿序列，之後用純追蹤重播。
    #
    # 與 nav2 並存而不是取代：nav2 負責「導航到任意點」，教導路徑負責
    # 「固定路線走得穩」。兩者不會同時送指令 —— 重播時是這個節點在發，
    # nav2 的 controller_server 沒有 goal 就不會發。
    #
    # 發到 cmd_vel_smoothed 而不是 cmd_vel：這樣仍會經過
    # steering_trim_cc（轉向零位補償）與 collision_monitor（防撞），
    # 跟 nav2 走同一條安全鏈。
    path_teach = Node(
        package=PKG,
        executable="path_teach_cc",
        name="path_teach_cc_node",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "cmd_topic": "cmd_vel_smoothed",
            # 建圖用的雷達已經被 scan_filter_cc 遮掉車尾，但避障要看得到人，
            # 所以這裡訂原始的 /scan 而不是 /scan_slam
            "scan_topic": "scan",
        }],
    )

    smartnav_nodes = [
        Node(
            package=PKG,
            executable="map_service_cc",
            # 降低排程優先度（nice 10）。
            #
            # 這幾個是 Python 服務節點，工作是「有人呼叫才做事」，
            # 但它們的 TransformListener 要在 rclpy 裡逐則處理 60 Hz 的 /tf，
            # 實測各吃掉約 30% CPU。Pi4 滿載時這會把真正即時的東西擠掉：
            #     odom_combined -> base_footprint 出現 333 ms 空窗
            #     -> controller 每個目標都噴 extrapolation into the future
            #     -> 所有導航目標 abort，車子完全不動
            #
            # 用正的 nice 值（不需要 root）把它們往後排，
            # 讓 EKF / slam_toolbox / controller_server 優先拿到 CPU。
            # 這不是減少總工作量，是決定「誰先做」——
            # 服務呼叫晚幾十毫秒沒差，控制迴路晚 300 毫秒就壞掉。
            # ★ prefix 必須是**單一字串**，不能給 list ★
            # 給 ["nice","-n","10"] 的話 launch_ros 會把它黏成一個 token
            # 'nice-n10'，然後 FileNotFoundError，節點根本起不來
            # （2026-07-31 踩過：三個服務節點全部沒啟動，
            #   症狀是 HMI 的開始建圖／結束儲存按了完全沒反應）。
            prefix="nice -n 10",
            name="map_service_cc_node",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "start_mode": start_mode,
                    "use_exploration": auto_start_exploration,
                    "nav2_lifecycle_manager": "lifecycle_manager_navigation_cc",
                }
            ],
        ),
        Node(
            package=PKG,
            executable="waypoint_service_cc",
            # 降低排程優先度（nice 10）。
            #
            # 這幾個是 Python 服務節點，工作是「有人呼叫才做事」，
            # 但它們的 TransformListener 要在 rclpy 裡逐則處理 60 Hz 的 /tf，
            # 實測各吃掉約 30% CPU。Pi4 滿載時這會把真正即時的東西擠掉：
            #     odom_combined -> base_footprint 出現 333 ms 空窗
            #     -> controller 每個目標都噴 extrapolation into the future
            #     -> 所有導航目標 abort，車子完全不動
            #
            # 用正的 nice 值（不需要 root）把它們往後排，
            # 讓 EKF / slam_toolbox / controller_server 優先拿到 CPU。
            # 這不是減少總工作量，是決定「誰先做」——
            # 服務呼叫晚幾十毫秒沒差，控制迴路晚 300 毫秒就壞掉。
            # ★ prefix 必須是**單一字串**，不能給 list ★
            # 給 ["nice","-n","10"] 的話 launch_ros 會把它黏成一個 token
            # 'nice-n10'，然後 FileNotFoundError，節點根本起不來
            # （2026-07-31 踩過：三個服務節點全部沒啟動，
            #   症狀是 HMI 的開始建圖／結束儲存按了完全沒反應）。
            prefix="nice -n 10",
            name="waypoint_service_cc_node",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package=PKG,
            executable="navigation_action_cc",
            # 降低排程優先度（nice 10）。
            #
            # 這幾個是 Python 服務節點，工作是「有人呼叫才做事」，
            # 但它們的 TransformListener 要在 rclpy 裡逐則處理 60 Hz 的 /tf，
            # 實測各吃掉約 30% CPU。Pi4 滿載時這會把真正即時的東西擠掉：
            #     odom_combined -> base_footprint 出現 333 ms 空窗
            #     -> controller 每個目標都噴 extrapolation into the future
            #     -> 所有導航目標 abort，車子完全不動
            #
            # 用正的 nice 值（不需要 root）把它們往後排，
            # 讓 EKF / slam_toolbox / controller_server 優先拿到 CPU。
            # 這不是減少總工作量，是決定「誰先做」——
            # 服務呼叫晚幾十毫秒沒差，控制迴路晚 300 毫秒就壞掉。
            # ★ prefix 必須是**單一字串**，不能給 list ★
            # 給 ["nice","-n","10"] 的話 launch_ros 會把它黏成一個 token
            # 'nice-n10'，然後 FileNotFoundError，節點根本起不來
            # （2026-07-31 踩過：三個服務節點全部沒啟動，
            #   症狀是 HMI 的開始建圖／結束儲存按了完全沒反應）。
            prefix="nice -n 10",
            name="navigation_action_cc_node",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ]

    explorer_node = Node(
        package="frontier_exploration_ros2",
        executable="frontier_explorer",
        name="frontier_explorer",
        output="screen",
        condition=IfCondition(use_exploration),
        parameters=[explore_params_file, {"use_sim_time": use_sim_time}],
        arguments=common_ros_args,
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_share, "launch", "rviz_launch.py")),
        condition=IfCondition(use_rviz),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    return LaunchDescription(
        declare_args + nav2_nodes + map_source_nodes + smartnav_nodes
        + [steering_trim, stuck_detector, path_teach, explorer_node, rviz]
    )
