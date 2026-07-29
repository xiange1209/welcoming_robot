import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

pkg_share_dir = get_package_share_directory("smartnav_navigation")
frontier_share_dir = get_package_share_directory("frontier_exploration_ros2")


def generate_launch_description():
    params_file = os.path.join(pkg_share_dir, "config", "wheeltec_akm.yaml")
    map_filename = os.path.join(pkg_share_dir, "config", "empty_map.yaml")
    frontier_params_file = os.path.join(frontier_share_dir, "config", "params.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
            ),
            Node(
                package="smartnav_navigation",
                executable="map_service",
                output="screen",
                parameters=[
                    {"exploration_timeout_sec": 1200.0},
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="smartnav_navigation",
                executable="waypoint_service",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="smartnav_navigation",
                executable="navigation_action",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                arguments=["--ros-args", "--log-level", "WARN"],
                parameters=[
                    params_file,
                    {"set_initial_pose": True},
                    {"initial_pose.x": 0.0},
                    {"initial_pose.y": 0.0},
                    {"initial_pose.z": 0.0},
                    {"initial_pose.yaw": 0.0},
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="slam_toolbox",
                executable="sync_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                arguments=["--ros-args", "--log-level", "WARN"],
                parameters=[
                    params_file,
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="frontier_exploration_ros2",
                executable="frontier_explorer",
                name="frontier_explorer",
                output="screen",
                parameters=[
                    # 先吃套件本身的 params.yaml，拿到針對 Ackermann 調過的距離門檻，
                    # 後面的 dict 再逐項覆寫成本專案必要的值 (後者優先)。
                    frontier_params_file,
                    {
                        # 由 create_map 動作透過 /control_exploration 啟動，不能自己開跑
                        "autostart": False,
                        # params.yaml 內是 false，但 map_service 會等這個服務，關掉會直接卡死
                        "control_service_enabled": True,
                        # map_service 是靠 /rosout 出現 "Exploration finished" 才判定完成，
                        # 而那兩行 log 只在 return_to_start_on_complete=true 的分支才會印出來。
                        # params.yaml 的 false 會讓 handle_exploration_complete 靜默 return，
                        # 導致建圖永遠等到 400 秒逾時。
                        "return_to_start_on_complete": True,
                        "completion_event_enabled": True,
                        # 阿克曼底盤最小轉彎半徑 0.8 m，太近的 frontier 根本無法規劃出路徑
                        "frontier_selection_min_distance": 0.8,
                        "frontier_candidate_min_goal_distance_m": 0.8,
                        # 這個門檻是拿來過濾「貼著牆」的 frontier 的：
                        # 鄰居 cost >= 此值就視為被擋住。膨脹半徑 0.35 + cost_scaling 2.0 之下，
                        # 45 大約等於「離牆 0.6 m 以內的 frontier 一律不選」，
                        # 避免車頭又被導進牆角而卡死。
                        "occ_threshold": 45,
                        # Pi 4 上 SLAM 與 Nav2 都偏慢，給每個目標一點沉澱時間比較穩
                        "post_goal_settle_enabled": True,
                        "post_goal_min_settle": 2.0,
                        "map_processing_rate_hz": 0.5,
                        # 規劃常失敗時暫時跳過該 frontier，避免卡在同一點
                        "frontier_suppression_enabled": True,
                        "goal_skip_on_blocked_goal": True,
                    },
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                arguments=["--ros-args", "--log-level", "WARN"],
                parameters=[
                    params_file,
                    {"yaml_filename": map_filename},
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="nav2_map_server",
                executable="map_saver_server",
                name="map_saver",
                output="screen",
                arguments=["--ros-args", "--log-level", "WARN"],
                parameters=[
                    params_file,
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map",
                output="screen",
                arguments=["--ros-args", "--log-level", "WARN"],
                parameters=[
                    {"autostart": True},
                    {"use_sim_time": use_sim_time},
                    {"node_names": ["map_server", "map_saver"]},
                    # bond_timeout=0.0 會完全關閉 bond 監控，這裡是刻意的：
                    #   1. Pi 4 滿載時 map_server 來不及在預設的 4 秒內回 bond，
                    #      lifecycle_manager 會誤判成節點掛掉而中止整個 bringup。
                    #   2. 更重要的是 map_server 的生命週期是由 map_service_node 掌管的，
                    #      建圖時它會被刻意退到 unconfigured (才不會有舊地圖殘留在 /map)。
                    #      bond 監控會把這個「刻意的下線」當成當機，然後把 map_server 重新拉回
                    #      active，於是 map_server(empty_map 200x200) 和 slam_toolbox(實際地圖)
                    #      同時在發 /map，global_costmap 的 static_layer 反覆 resize，
                    #      costmap 索引錯位 -> 機器人腳下變成 lethal -> 規劃器一律回報 "Start occupied"。
                    {"bond_timeout": 0.0},
                    {"attempt_respawn_reconnection": False},
                ],
            ),
        ]
    )
