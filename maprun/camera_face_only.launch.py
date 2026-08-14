#!/usr/bin/env python3
"""只開彩色的相機 launch —— 給人臉辨識用。

★ 為什麼要這個（2026-08-14 實測）
原廠 `wheeltec_camera.launch.py` 開的是完整功能：彩色 + 深度 + IR + 點雲 + d2c viewer，
外加**兩個** republish（彩色與深度各一）。實測 `astra_camera_node` 吃到 **135.9%**，
整台 Pi 4（4 核心）load average 衝到 **51**，後果是：

    人臉向量只發得出 0.3 Hz -> 註冊採樣逾時 -> 剛註冊好的使用者被回滾刪掉
    ros2 topic hz / topic info 全部逾時（看起來像沒有資料，其實是 CLI 排不到）

而**人臉辨識完全不用深度** —— `face_embedding` 只訂 `/camera/color/image_raw/compressed`，
InsightFace buffalo_sc 是純 RGB 模型。深度是導航避障（`depth_obstacle_cc`）在用的，
而故事線本來就是串行的：認人時車不動、帶位時不認人。

原廠 launch 把 `enable_d2c_viewer` 硬寫成 True 且不透傳 `enable_depth`，所以只能自己寫。

用法：
    ros2 launch ~/maprun/camera_face_only.launch.py
或透過 camera_manager_cc（它會連同子行程一起收乾淨）：
    ros2 run smartnav_navigation_cc camera_manager_cc --ros-args \\
      -p camera_launch_cmd:="ros2 launch /home/user/maprun/camera_face_only.launch.py"

★ 要跑導航避障時**不要用這支**，回去用 camera_obstacle_cc.launch.py（那支是只開深度）。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    astra_launch = os.path.join(
        get_package_share_directory("astra_camera"), "launch", "astra.launch.xml"
    )

    camera = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(astra_launch),
        launch_arguments=[
            # === 只留彩色 ===
            ("enable_color", "true"),
            ("enable_depth", "false"),
            ("enable_ir", "false"),
            ("enable_point_cloud", "false"),
            ("enable_colored_point_cloud", "false"),
            ("depth_registration", "false"),
            # d2c viewer 需要深度，關掉深度就不能開
            ("enable_d2c_viewer", "false"),
        ],
    )

    # face_embedding 訂的是 compressed，驅動本身不發，要 republish
    compress_color = Node(
        package="image_transport",
        executable="republish",
        name="republish_color_compressed",
        arguments=["raw", "compressed"],
        remappings=[
            ("in", "/camera/color/image_raw"),
            ("out/compressed", "/camera/color/image_raw/compressed"),
        ],
        # ★ 這兩個參數照抄原廠 wheeltec_camera.launch.py —— 少了它們
        #   republish 的 out_transport 會是空的，壓縮話題不會出現
        parameters=[{"format": "jpeg", "jpeg_quality": 85}],
    )

    return LaunchDescription([camera, compress_color])
