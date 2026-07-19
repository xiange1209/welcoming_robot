from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ollama_base_url = LaunchConfiguration("ollama_base_url")
    model_name = LaunchConfiguration("model_name")
    temperature = LaunchConfiguration("temperature")
    wait_for_nav_services = LaunchConfiguration("wait_for_nav_services")
    system_prompt_file = LaunchConfiguration("system_prompt_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ollama_base_url",
                default_value="http://localhost:11434",
                description=(
                    "Ollama API 位址。現階段運算在筆電上執行，"
                    "請改成筆電的區網 IP，例如 http://192.168.1.xxx:11434"
                ),
            ),
            DeclareLaunchArgument(
                "model_name",
                default_value="gemma4:e2b",
                description="Ollama 模型名稱",
            ),
            DeclareLaunchArgument(
                "temperature",
                default_value="0.0",
                description="LLM 取樣溫度",
            ),
            DeclareLaunchArgument(
                "wait_for_nav_services",
                default_value="true",
                description="是否阻塞等待導航服務就緒（無導航環境測 LLM 時設 false）",
            ),
            DeclareLaunchArgument(
                "system_prompt_file",
                default_value="system_prompt_bank.txt",
                description="system prompt 檔名（config/ 下）：銀行版 system_prompt_bank.txt 或導航版 system_prompt.txt",
            ),
            Node(
                package="smartnav_llm",
                executable="llm_service",
                name="llm_service_node",
                output="screen",
                parameters=[
                    {
                        "ollama_base_url": ollama_base_url,
                        "model_name": model_name,
                        "temperature": temperature,
                        "wait_for_nav_services": wait_for_nav_services,
                        "system_prompt_file": system_prompt_file,
                    }
                ],
            ),
        ]
    )
