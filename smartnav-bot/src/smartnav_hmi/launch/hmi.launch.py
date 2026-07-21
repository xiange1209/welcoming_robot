import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    web_dir = os.path.join(get_package_share_directory("smartnav_hmi"), "web")

    rosbridge_port = LaunchConfiguration("rosbridge_port")
    hmi_port = LaunchConfiguration("hmi_port")
    video_port = LaunchConfiguration("video_port")

    return LaunchDescription(
        [
            DeclareLaunchArgument("rosbridge_port", default_value="9090", description="rosbridge websocket 埠"),
            DeclareLaunchArgument("hmi_port", default_value="8081", description="HMI 網頁 HTTP 埠"),
            DeclareLaunchArgument("video_port", default_value="8080", description="web_video_server 影像串流埠"),
            # rosbridge：瀏覽器透過 websocket 訂閱 ROS 話題（apt: ros-jazzy-rosbridge-server）
            Node(
                package="rosbridge_server",
                executable="rosbridge_websocket",
                name="rosbridge_websocket",
                output="screen",
                parameters=[{"port": ParameterValue(rosbridge_port, value_type=int)}],
            ),
            # 影像串流：網頁的 <img> 直接抓 http://<機器人IP>:8080/stream?topic=...
            # 少了它，HMI 的相機區塊會永遠停在「載入中…」（2026-07-21 實機踩過）
            Node(
                package="web_video_server",
                executable="web_video_server",
                name="web_video_server",
                output="screen",
                parameters=[{"port": ParameterValue(video_port, value_type=int)}],
            ),
            # 靜態網頁伺服器：平板瀏覽器開 http://<機器人IP>:8081
            ExecuteProcess(
                cmd=["python3", "-m", "http.server", hmi_port, "--directory", web_dir],
                name="hmi_http_server",
                output="screen",
            ),
        ]
    )
