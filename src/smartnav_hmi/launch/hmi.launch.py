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

    return LaunchDescription(
        [
            DeclareLaunchArgument("rosbridge_port", default_value="9090", description="rosbridge websocket 埠"),
            DeclareLaunchArgument("hmi_port", default_value="8081", description="HMI 網頁 HTTP 埠"),
            # rosbridge：瀏覽器透過 websocket 訂閱 ROS 話題（apt: ros-jazzy-rosbridge-server）
            Node(
                package="rosbridge_server",
                executable="rosbridge_websocket",
                name="rosbridge_websocket",
                output="screen",
                parameters=[{"port": ParameterValue(rosbridge_port, value_type=int)}],
            ),
            # 靜態網頁伺服器：平板瀏覽器開 http://<機器人IP>:8081
            ExecuteProcess(
                cmd=["python3", "-m", "http.server", hmi_port, "--directory", web_dir],
                name="hmi_http_server",
                output="screen",
            ),
        ]
    )
