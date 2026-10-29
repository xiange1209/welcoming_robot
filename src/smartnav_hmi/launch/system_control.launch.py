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
            # ★ 2026-09-29：true → false。定位模式一定要關探索，否則 explorer 帶 auto_start 一起起來，
            #   教導路徑／迎賓故事線的情境裡按「開始建圖」車子就自己開走（manager.py 9/25 註解）。
            "localization": ("localization", "false"),
        }
        if variant in modes:
            mode, explore = modes[variant]
            # ★ 2026-10-29：導航仲裁跟導航堆疊一起起、一起停（以前是獨立單元，忘了開＝客人與管理者帶位一律被拒）。
            #   三種模式都起：建圖存檔後 map_service 會自己切到定位模式，那時候也要能帶位。節點名維持 nav_arbiter_node
            #   （HMI 的節點健康表 core/constants.py 認這個名字）。
            return [
                _include("smartnav_navigation_cc", "nav_bringup_cc.launch.py", {
                    "use_sim_time": "false", "use_rviz": "false", "start_mode": mode,
                    "use_exploration": explore, "auto_start_exploration": explore,
                }),
                Node(package="smartnav_brain", executable="nav_arbiter", name="nav_arbiter_node", output="screen"),
            ]
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
    # ★ 2026-10-23：人臉的兩種啟動方式。normal＝跟沒給 variant 一樣（情境裡的 "face" 走這條）；
    #   eco＝沒人時每 4 秒推論一次（empty_interval_sec），一偵測到臉回到 2 秒。
    #   參數用 ros_arguments 帶在指令列上，HMI 才能從行程指令列看出目前是哪個模式（variant_detect）。
    if unit == "face" and variant in ("normal", "eco"):
        extra = ["-p", "empty_interval_sec:=4.0"] if variant == "eco" else []
        return [Node(package="smartnav_vision", executable="face_embedding", name="face_node", output="screen",
                     parameters=[{"enable_gpu": False}], ros_arguments=extra)]
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
