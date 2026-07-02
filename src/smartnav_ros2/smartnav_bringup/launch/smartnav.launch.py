import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    smartnav_llm_share = get_package_share_directory("smartnav_llm")
    smartnav_brain_share = get_package_share_directory("smartnav_brain")

    image_topic = LaunchConfiguration("image_topic")
    enable_gpu = LaunchConfiguration("enable_gpu")
    ollama_base_url = LaunchConfiguration("ollama_base_url")
    model_name = LaunchConfiguration("model_name")
    use_sim_time = LaunchConfiguration("use_sim_time")

    enable_vision = LaunchConfiguration("enable_vision")
    enable_audio = LaunchConfiguration("enable_audio")
    enable_llm = LaunchConfiguration("enable_llm")
    enable_brain = LaunchConfiguration("enable_brain")
    enable_nav2 = LaunchConfiguration("enable_nav2")

    return LaunchDescription(
        [
            # ── 共用參數 ──────────────────────────────────────────────
            DeclareLaunchArgument("image_topic", default_value="/image_raw", description="相機影像話題"),
            DeclareLaunchArgument("enable_gpu", default_value="true", description="視覺/推理是否使用 GPU"),
            DeclareLaunchArgument(
                "ollama_base_url",
                default_value="http://localhost:11434",
                description=(
                    "Ollama API 位址。現階段運算在筆電上執行，"
                    "請改成筆電的區網 IP，例如 http://192.168.1.xxx:11434"
                ),
            ),
            DeclareLaunchArgument("model_name", default_value="gemma4:e2b", description="smartnav_llm 使用的 Ollama 模型"),
            DeclareLaunchArgument("use_sim_time", default_value="false", description="是否使用模擬時間 (Gazebo)"),
            # ── 各模組開關 (硬體或模型尚未就緒時可個別關閉) ──────────────
            DeclareLaunchArgument("enable_vision", default_value="true", description="是否啟動人臉辨識/註冊/文字橋接"),
            DeclareLaunchArgument("enable_audio", default_value="true", description="是否啟動語音喚醒/辨識/合成/播放"),
            DeclareLaunchArgument("enable_llm", default_value="true", description="是否啟動 smartnav_llm 對話式 Agent"),
            DeclareLaunchArgument("enable_brain", default_value="true", description="是否啟動地圖/地點/導航動作服務"),
            DeclareLaunchArgument(
                "enable_nav2",
                default_value="false",
                description="是否啟動完整 Nav2 導航棧 + RViz (需要 TurtleBot3 或模擬環境)",
            ),
            # ── smartnav_vision ───────────────────────────────────────
            Node(
                package="smartnav_vision",
                executable="face_recognition",
                name="face_recognition_node",
                output="screen",
                condition=IfCondition(enable_vision),
                parameters=[
                    {
                        "image_topic": image_topic,
                        "enable_gpu": enable_gpu,
                    }
                ],
            ),
            Node(
                package="smartnav_vision",
                executable="face_registration",
                name="face_registration_node",
                output="screen",
                condition=IfCondition(enable_vision),
                parameters=[
                    {
                        "image_topic": image_topic,
                        "enable_gpu": enable_gpu,
                    }
                ],
            ),
            Node(
                package="smartnav_vision",
                executable="recognition_text_bridge",
                name="recognition_text_bridge_node",
                output="screen",
                condition=IfCondition(enable_vision),
            ),
            # ── smartnav_audio ────────────────────────────────────────
            Node(
                package="smartnav_audio",
                executable="voice_trigger",
                name="voice_trigger_node",
                output="screen",
                condition=IfCondition(enable_audio),
            ),
            Node(
                package="smartnav_audio",
                executable="speech_recognizer",
                name="speech_recognizer_node",
                output="screen",
                condition=IfCondition(enable_audio),
            ),
            Node(
                package="smartnav_audio",
                executable="speech_synthesizer",
                name="speech_synthesizer_node",
                output="screen",
                condition=IfCondition(enable_audio),
            ),
            Node(
                package="smartnav_audio",
                executable="voice_playback",
                name="voice_playback_node",
                output="screen",
                condition=IfCondition(enable_audio),
            ),
            # ── smartnav_llm (沿用既有 launch/llm.launch.py) ──────────
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(smartnav_llm_share, "launch", "llm.launch.py")),
                launch_arguments={
                    "ollama_base_url": ollama_base_url,
                    "model_name": model_name,
                }.items(),
                condition=IfCondition(enable_llm),
            ),
            # ── smartnav_brain：地圖/地點/導航動作服務 (沿用既有 brain.launch.py) ──
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(smartnav_brain_share, "launch", "brain.launch.py")),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
                condition=IfCondition(enable_brain),
            ),
            # ── smartnav_brain：完整 Nav2 導航棧 + RViz (需要機器人本體或模擬環境) ──
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(smartnav_brain_share, "launch", "nav2.launch.py")),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
                condition=IfCondition(enable_nav2),
            ),
        ]
    )
