"""模擬車＋世界 —— 取代實車的 sensors_cc.launch.py

提供跟實車一模一樣的介面（話題、frame、型別），導航鏈不用改任何東西：
    /scan（frame laser）、/odom_combined、TF odom_combined -> base_footprint -> {base_link, laser}
    吃 /cmd_vel（經 akm_realism 加上實車的死區／不對稱／機械偏移）
另外發 /clock，所以導航那邊要 use_sim_time:=true。

用法（通常不直接用，sim_explore.launch.py 會帶它起來）：
    ros2 launch smartnav_sim sim_robot.launch.py                     # 瓶頸測試場，有視窗
    ros2 launch smartnav_sim sim_robot.launch.py gui:=false          # 不開視窗（最省資源）
    ros2 launch smartnav_sim sim_robot.launch.py map_yaml:=<地圖.yaml> # 拿實車地圖當場地

參數：
    world           worlds/ 底下的檔名或絕對路徑（預設 bottlenecks.sdf）
    map_yaml        給了就當場用 pgm_to_world 轉（蓋過 world），起點自動挑
    gui             false = 只跑模擬伺服器（--headless-rendering，光達照樣有資料）
    render_engine   ogre2（預設）／ogre —— WSL 開視窗崩潰時改 ogre
    realism         false = 關掉死區與不對稱半徑（理想車），機械偏移保留
    x y yaw         起點；x、y 預設 auto（世界檔用 0,0；地圖檔用 pgm_to_world 挑的點）
    lidar_x_actual  模擬光達實際 x（TF 仍是 0.0887）—— 用來看 TF 錯位的影響
"""

import os
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

PKG = "smartnav_sim"
MODEL_NAME = "smartnav_akm"


def _truthy(s: str) -> bool:
    return s.strip().lower() in ("true", "1", "yes", "on")


def _setup(context):
    share = get_package_share_directory(PKG)

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    actions = []
    x, y, yaw = arg("x"), arg("y"), arg("yaw")
    map_yaml = arg("map_yaml")
    if map_yaml:
        # 當場轉：改了地圖不用另外跑工具
        from smartnav_sim.pgm_to_world import convert
        sdf, summary, (sx, sy) = convert(map_yaml)
        stem = os.path.splitext(os.path.basename(map_yaml))[0]
        world = os.path.join(tempfile.gettempdir(), f"smartnav_sim_{stem}.sdf")
        with open(world, "w", encoding="utf-8", newline="\n") as f:
            f.write(sdf)
        actions.append(LogInfo(msg=f"[smartnav_sim] {summary}\n  → {world}"))
        if x == "auto":
            x = str(sx)
        if y == "auto":
            y = str(sy)
    else:
        world = arg("world")
        if not os.path.isabs(world):
            world = os.path.join(share, "worlds", world)
    x = "0.0" if x == "auto" else x
    y = "0.0" if y == "auto" else y

    gz_args = f"-r -v2 {world}"
    if not _truthy(arg("gui")):
        # 不開視窗，但 gpu_lidar 還是要算圖 —— --headless-rendering 用 EGL 在背景畫
        gz_args += " -s --headless-rendering"
    engine = arg("render_engine")
    if engine:
        gz_args += f" --render-engine {engine}"

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items(),
    )

    mappings = {"lidar_x_actual": arg("lidar_x_actual") or "same"}
    robot_description = xacro.process_file(
        os.path.join(share, "urdf", "senior_akm_sim.urdf.xacro"), mappings=mappings).toxml()

    # 靜態 TF base_footprint -> base_link / laser（跟實車 robot_mode_description 發的一樣）。
    # 輪子與轉向關節沒有 /joint_states，所以不會發動態 TF —— 實車也把 joint_state_publisher 關了
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", MODEL_NAME,
                   "-x", x, "-y", y, "-z", "0.02", "-Y", yaw],
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="smartnav_sim_bridge",
        output="screen",
        parameters=[{"config_file": os.path.join(share, "config", "bridge.yaml"), "use_sim_time": True}],
    )
    realism = Node(
        package=PKG,
        executable="akm_realism",
        name="akm_realism_node",
        output="screen",
        parameters=[{"use_sim_time": True, "realism": _truthy(arg("realism"))}],
    )
    actions.append(LogInfo(msg=f"[smartnav_sim] 世界 {world}、起點 ({x}, {y}, yaw {yaw})"))
    return actions + [gz, rsp, spawn, bridge, realism]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="bottlenecks.sdf"),
        DeclareLaunchArgument("map_yaml", default_value=""),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("render_engine", default_value=""),
        DeclareLaunchArgument("realism", default_value="true"),
        DeclareLaunchArgument("x", default_value="auto"),
        DeclareLaunchArgument("y", default_value="auto"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("lidar_x_actual", default_value=""),
        OpaqueFunction(function=_setup),
    ])
