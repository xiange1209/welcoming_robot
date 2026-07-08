import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

pkg_share_dir = get_package_share_directory("smartnav_navigation")


def generate_launch_description():
    map_filename = os.path.join(pkg_share_dir, "config", "empty_map.yaml")
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
                parameters=[{"use_sim_time": use_sim_time}],
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
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="frontier_exploration_ros2",
                executable="frontier_explorer",
                output="screen",
                parameters=[
                    {"autostart": False},
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
                parameters=[{"use_sim_time": use_sim_time}],
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
                ],
            ),
        ]
    )
