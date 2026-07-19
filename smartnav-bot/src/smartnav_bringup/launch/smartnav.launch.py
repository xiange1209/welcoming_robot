import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    smartnav_llm_share = get_package_share_directory("smartnav_llm")
    smartnav_navigation_share = get_package_share_directory("smartnav_navigation")

    image_topic = LaunchConfiguration("image_topic")
    enable_gpu = LaunchConfiguration("enable_gpu")
    ollama_base_url = LaunchConfiguration("ollama_base_url")
    model_name = LaunchConfiguration("model_name")
    use_sim_time = LaunchConfiguration("use_sim_time")

    enable_vision = LaunchConfiguration("enable_vision")
    enable_audio = LaunchConfiguration("enable_audio")
    enable_llm = LaunchConfiguration("enable_llm")
    enable_navigation = LaunchConfiguration("enable_navigation")
    enable_nav2 = LaunchConfiguration("enable_nav2")
    enable_chassis = LaunchConfiguration("enable_chassis")
    enable_web_video = LaunchConfiguration("enable_web_video")
    web_video_port = LaunchConfiguration("web_video_port")
    enable_hmi = LaunchConfiguration("enable_hmi")
    audio_stack = LaunchConfiguration("audio_stack")
    llm_stack = LaunchConfiguration("llm_stack")
    enable_brain = LaunchConfiguration("enable_brain")
    system_prompt_file = LaunchConfiguration("system_prompt_file")
    notify_backend = LaunchConfiguration("notify_backend")
    telegram_bot_token = LaunchConfiguration("telegram_bot_token")
    telegram_chat_id = LaunchConfiguration("telegram_chat_id")

    # 雙版本選擇器的組合條件（enable 開關 + stack 選擇要同時成立）
    audio_smartnav = IfCondition(PythonExpression(["'", enable_audio, "' == 'true' and '", audio_stack, "' == 'smartnav'"]))
    audio_wheeltec = IfCondition(PythonExpression(["'", enable_audio, "' == 'true' and '", audio_stack, "' == 'wheeltec'"]))
    audio_any = IfCondition(enable_audio)
    llm_smartnav = IfCondition(PythonExpression(["'", enable_llm, "' == 'true' and '", llm_stack, "' == 'smartnav'"]))
    llm_wheeltec = IfCondition(PythonExpression(["'", enable_llm, "' == 'true' and '", llm_stack, "' == 'wheeltec'"]))

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
            DeclareLaunchArgument("enable_navigation", default_value="true", description="是否啟動地圖/地點/導航動作服務"),
            DeclareLaunchArgument(
                "enable_nav2",
                default_value="false",
                description="是否啟動完整 Nav2 導航棧 + RViz (需要 TurtleBot3 或模擬環境)",
            ),
            DeclareLaunchArgument(
                "enable_chassis",
                default_value="false",
                description=(
                    "是否啟動 WHEELTEC 實體底盤驅動（車型在 src/wheeltec/turn_on_wheeltec_robot/"
                    "config/wheeltec_param.yaml 的 car_mode 設定；建圖/導航請用 wheeltec 官方 launch，"
                    "不要與 enable_navigation 的 TurtleBot3 流程同時開）"
                ),
            ),
            DeclareLaunchArgument(
                "enable_web_video",
                default_value="false",
                description="是否啟動 web_video_server（瀏覽器看影像：http://<機器人IP>:<port>/stream?topic=/image_raw）",
            ),
            DeclareLaunchArgument("web_video_port", default_value="8080", description="web_video_server 的 HTTP 埠"),
            DeclareLaunchArgument(
                "enable_hmi",
                default_value="false",
                description="是否啟動平板 HMI（rosbridge :9090＋網頁 :8081；平板瀏覽 http://<機器人IP>:8081，影像需同開 enable_web_video）",
            ),
            DeclareLaunchArgument(
                "audio_stack",
                default_value="smartnav",
                description="語音方案：smartnav（sherpa-onnx 自由語句 ASR）或 wheeltec（麥克風陣列＋本地離線命令詞）",
            ),
            DeclareLaunchArgument(
                "llm_stack",
                default_value="smartnav",
                description="LLM 方案：smartnav（LangChain Agent，含導航工具）或 wheeltec（ollama_ros_chat 輕量單輪對話）",
            ),
            # ── 銀行迎賓劇本（smartnav_brain）與通報設定 ─────────────────
            DeclareLaunchArgument(
                "enable_brain",
                default_value="true",
                description="是否啟動銀行迎賓劇本節點（VIP 迎賓語音／黑名單通報／訪客記錄）",
            ),
            DeclareLaunchArgument(
                "system_prompt_file",
                default_value="system_prompt_bank.txt",
                description="LLM system prompt：銀行版 system_prompt_bank.txt 或導航版 system_prompt.txt",
            ),
            DeclareLaunchArgument(
                "notify_backend",
                default_value="none",
                description="行員通報後端：none（dry-run 只記錄）或 telegram（需同時給 token 與 chat_id）",
            ),
            DeclareLaunchArgument("telegram_bot_token", default_value="", description="Telegram Bot Token"),
            DeclareLaunchArgument("telegram_chat_id", default_value="", description="Telegram Chat ID"),
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
            # ── 語音輸入：smartnav 方案（sherpa-onnx 自由語句）────────────
            Node(
                package="smartnav_audio",
                executable="voice_trigger",
                name="voice_trigger_node",
                output="screen",
                condition=audio_smartnav,
            ),
            Node(
                package="smartnav_audio",
                executable="speech_recognizer",
                name="speech_recognizer_node",
                output="screen",
                condition=audio_smartnav,
            ),
            # ── 語音輸入：wheeltec 方案（麥克風陣列＋離線命令詞，免金鑰）──
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("wheeltec_mic_ros2"), "launch", "base.launch.py"])
                ),
                condition=audio_wheeltec,
            ),
            Node(
                # 把 wheeltec 的辨識文字話題 voice_words 轉發到 /user_text（型別同為 String）
                package="topic_tools",
                executable="relay",
                name="voice_words_relay",
                arguments=["voice_words", "/user_text"],
                output="screen",
                condition=audio_wheeltec,
            ),
            # ── 語音輸出（TTS＋播放）：兩種輸入方案共用 ────────────────
            Node(
                package="smartnav_audio",
                executable="speech_synthesizer",
                name="speech_synthesizer_node",
                output="screen",
                condition=audio_any,
            ),
            Node(
                package="smartnav_audio",
                executable="voice_playback",
                name="voice_playback_node",
                output="screen",
                condition=audio_any,
            ),
            # ── LLM：smartnav 方案（LangChain Agent，含導航＋銀行工具）────
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(smartnav_llm_share, "launch", "llm.launch.py")),
                launch_arguments={
                    "ollama_base_url": ollama_base_url,
                    "model_name": model_name,
                    # 導航關閉時 LLM 不阻塞等待導航服務（在家測試不卡住）
                    "wait_for_nav_services": enable_navigation,
                    "system_prompt_file": system_prompt_file,
                }.items(),
                condition=llm_smartnav,
            ),
            # ── 銀行迎賓劇本：VIP 迎賓／黑名單 Telegram 通報／訪客記錄 ────
            Node(
                package="smartnav_brain",
                executable="bank_reception",
                name="bank_reception_node",
                output="screen",
                condition=IfCondition(enable_brain),
                parameters=[
                    {
                        "notify_backend": notify_backend,
                        "telegram_bot_token": telegram_bot_token,
                        "telegram_chat_id": telegram_chat_id,
                    }
                ],
            ),
            # ── LLM：wheeltec 方案（ollama_ros_chat 單輪對話＋橋接節點）──
            Node(
                package="ollama_ros_chat",
                executable="topic_server",
                name="ollama_topic_server",
                output="screen",
                parameters=[
                    {
                        # ollama_ros_chat 走 OpenAI 相容端點，需要 /v1 尾綴
                        "base_url": [ollama_base_url, "/v1"],
                        "api_key": "ollama",
                        # 務必顯式指定模型，避免它自動選到清單第一個模型
                        "use_model": model_name,
                    }
                ],
                condition=llm_wheeltec,
            ),
            Node(
                # /user_text（純文字）↔ chat_message/chat_response（JSON）格式橋接
                package="smartnav_llm",
                executable="ollama_chat_bridge",
                name="ollama_chat_bridge_node",
                output="screen",
                condition=llm_wheeltec,
            ),
            # ── 平板 HMI：rosbridge + 網頁（辨識結果/對話/相機串流；預設關閉）──
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("smartnav_hmi"), "launch", "hmi.launch.py"])
                ),
                condition=IfCondition(enable_hmi),
            ),
            # ── web_video_server：瀏覽器即時看影像話題（apt 二進位；預設關閉）──
            Node(
                package="web_video_server",
                executable="web_video_server",
                name="web_video_server",
                output="screen",
                condition=IfCondition(enable_web_video),
                parameters=[{"port": ParameterValue(web_video_port, value_type=int)}],
            ),
            # ── WHEELTEC 實體底盤驅動（odom/TF/cmd_vel；預設關閉）────────
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("turn_on_wheeltec_robot"), "launch", "turn_on_wheeltec_robot.launch.py"]
                    )
                ),
                condition=IfCondition(enable_chassis),
            ),
            # ── smartnav_navigation：地圖/地點/導航動作服務 (沿用既有 brain.launch.py) ──
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(smartnav_navigation_share, "launch", "brain.launch.py")),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
                condition=IfCondition(enable_navigation),
            ),
            # ── smartnav_navigation：完整 Nav2 導航棧 + RViz (需要機器人本體或模擬環境) ──
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(smartnav_navigation_share, "launch", "nav2.launch.py")),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
                condition=IfCondition(enable_nav2),
            ),
        ]
    )
