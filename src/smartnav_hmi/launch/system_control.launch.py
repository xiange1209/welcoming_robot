"""HMI 系統控制的單元化 launch 入口。

只接受固定 unit/variant，避免網頁 API 成為任意命令執行入口。
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _include(package: str, filename: str, arguments: dict[str, str] | None = None):
    path = get_package_share_directory(package) + "/launch/" + filename
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path), launch_arguments=(arguments or {}).items())


def _actions(context):
    unit = context.launch_configurations["unit"]
    variant = context.launch_configurations["variant"]
    if unit == "sensors":
        return [_include("smartnav_navigation_cc", "sensors_cc.launch.py", {"with_camera": "false"})]
    if unit == "camera":
        if variant == "face":
            return [_include("smartnav_navigation_cc", "camera_face_cc.launch.py")]
        if variant == "obstacle":
            return [_include("smartnav_navigation_cc", "camera_obstacle_cc.launch.py", {
                "with_color": "false", "depth_width": "160", "depth_height": "120", "cloud_hz": "5.0", "pixel_step": "2",
            })]
    if unit == "nav":
        modes = {
            "mapping": ("mapping", "false"),
            "explore": ("mapping", "true"),
            "localization": ("localization", "true"),
        }
        if variant in modes:
            mode, explore = modes[variant]
            return [_include("smartnav_navigation_cc", "nav_bringup_cc.launch.py", {
                "use_sim_time": "false", "use_rviz": "false", "start_mode": mode,
                "use_exploration": explore, "auto_start_exploration": explore,
            })]
    if unit == "asr_chain":
        return [_include("smartnav_audio", "asr_chain.launch.py")]

    nodes = {
        "face": ("smartnav_vision", "face_embedding", {"enable_gpu": False}),
        "user_auth": ("smartnav_brain", "user_auth", {}),
        "bank_reception": ("smartnav_brain", "bank_reception", {}),
        "llm": ("smartnav_llm", "llm_service", {}),
        "speech_synthesizer": ("smartnav_audio", "speech_synthesizer", {}),
        "voice_playback": ("smartnav_audio", "voice_playback", {}),
    }
    if unit in nodes and not variant:
        package, executable, parameters = nodes[unit]
        return [Node(package=package, executable=executable, name=f"{unit}_node", output="screen", parameters=[parameters])]
    raise RuntimeError(f"不支援的 system-control unit/variant: {unit}/{variant}")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("unit"),
        DeclareLaunchArgument("variant", default_value=""),
        OpaqueFunction(function=_actions),
    ])
