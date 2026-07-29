from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from typing import Any

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parse_optional_bool(value: str, arg_name: str):
    normalized = value.strip().lower()
    if normalized == "":
        return None
    if normalized in ("true", "1", "yes", "on"):
        return True
    if normalized in ("false", "0", "no", "off"):
        return False
    raise RuntimeError(
        f"Invalid boolean value for launch argument '{arg_name}': '{value}'. "
        "Expected one of: true/false, 1/0, yes/no, on/off"
    )


def _create_frontier_actions(context):
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")
    autostart_value = LaunchConfiguration("autostart").perform(context)
    control_service_enabled_value = LaunchConfiguration("control_service_enabled").perform(context)
    map_qos_durability = LaunchConfiguration("map_qos_durability")
    map_qos_autodetect_on_startup = LaunchConfiguration("map_qos_autodetect_on_startup")
    map_qos_autodetect_timeout_s = LaunchConfiguration("map_qos_autodetect_timeout_s")
    costmap_qos_reliability = LaunchConfiguration("costmap_qos_reliability")

    frontier_overrides: dict[str, Any] = {
        "use_sim_time": use_sim_time,
        "map_qos_durability": map_qos_durability,
        "map_qos_autodetect_on_startup": map_qos_autodetect_on_startup,
        "map_qos_autodetect_timeout_s": map_qos_autodetect_timeout_s,
        "costmap_qos_reliability": costmap_qos_reliability,
    }
    autostart_override = _parse_optional_bool(autostart_value, "autostart")
    if autostart_override is not None:
        frontier_overrides["autostart"] = autostart_override

    control_service_enabled_override = _parse_optional_bool(
        control_service_enabled_value,
        "control_service_enabled",
    )
    if control_service_enabled_override is not None:
        frontier_overrides["control_service_enabled"] = control_service_enabled_override

    frontier_node = Node(
        package="frontier_exploration_ros2",
        executable="frontier_explorer",
        name="frontier_explorer",
        namespace=namespace,
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            params_file,
            frontier_overrides,
        ],
    )

    return [frontier_node]


def generate_launch_description():
    pkg_share = get_package_share_directory("frontier_exploration_ros2")
    default_params = (Path(pkg_share) / "config" / "params.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Optional namespace for the frontier explorer node.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=str(default_params),
                description="Parameter file for frontier_explorer.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time.",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="Log level (debug, info, warn, error, fatal).",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="",
                description="Optional override for explorer autostart. Leave empty to use the parameter file.",
            ),
            DeclareLaunchArgument(
                "control_service_enabled",
                default_value="",
                description=(
                    "Optional override for frontier control service availability. "
                    "Leave empty to use the parameter file."
                ),
            ),
            DeclareLaunchArgument(
                "map_qos_durability",
                default_value="transient_local",
                description="Map durability: transient_local | volatile | system_default.",
            ),
            DeclareLaunchArgument(
                "map_qos_autodetect_on_startup",
                default_value="false",
                description="Enable startup-only map durability autodetect.",
            ),
            DeclareLaunchArgument(
                "map_qos_autodetect_timeout_s",
                default_value="2.0",
                description="Autodetect timeout per attempt in seconds.",
            ),
            DeclareLaunchArgument(
                "costmap_qos_reliability",
                default_value="reliable",
                description="Costmap reliability: reliable | best_effort | system_default.",
            ),
            OpaqueFunction(function=_create_frontier_actions),
        ]
    )
