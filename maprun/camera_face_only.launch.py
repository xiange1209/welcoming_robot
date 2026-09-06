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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    astra_launch = os.path.join(
        get_package_share_directory("astra_camera"), "launch", "astra.launch.xml"
    )

    # ★★ 2026-09-06：把相機幀率降下來 —— 這是本專案 CPU 帳上最划算的一刀 ★★
    #
    # 現況（全部查證過，不是推論）：
    #   astra.launch.xml:16   color_fps 預設 **30**
    #   下游沒有任何人需要 30：
    #     hmi_server   video_fps 預設 **6**，超過的直接丟掉（_should_process_frame）
    #     face_embedding 單幀推論要 1.92 核心秒 -> 實際吃不到 **0.5 fps**
    #   也就是說 astra_camera_node 與下面那個 republish
    #   **有五分之四的工作量是產生出來馬上被丟掉的**。
    #
    # 這兩支的實測價碼（face_embedding_node.py 的 CPU 預算註解）：
    #   相機 + republish 約 52% —— 而且是「隨幀率線性」的那種成本
    #   （擷取、DDS 搬 900 KB 原始幀、JPEG 編碼，每一項都是每幀一次）
    #
    # ★ 為什麼降 fps 是安全的（查了廠商驅動原始碼才敢動）：
    #   astra.launch.xml **沒有**設 use_uvc_camera，而
    #   ob_camera_node_factory.cpp:69 的預設是 **false** -> 走 OpenNI 路徑。
    #   OpenNI 路徑在 ob_camera_node.cpp:191 遇到不支援的模式會**印警告後
    #   退回同解析度的支援模式**，相機照樣起得來。
    #   （UVC 路徑就不是了：uvc_camera_driver.cpp:186 失敗會直接 uvc_close
    #     把裝置關掉，那條路降 fps 會變成完全沒有相機。這台不走那條。）
    #
    # ★ 預設值選 15 不選 6：640x480@15 是 Astra 的標準模式，幾乎一定支援；
    #   6 不一定在支援清單裡，落到 fallback 就會**悄悄變回 30**。
    #   到現場想再壓，用 color_fps:=6 試，然後看啟動 log 有沒有
    #   「Video mode ... is not supported」——有的話就是沒吃到，退回 15。
    color_fps = LaunchConfiguration("color_fps")

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
            ("color_fps", color_fps),
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
        # ★ format 照抄原廠 wheeltec_camera.launch.py —— 少了它
        #   republish 的 out_transport 會是空的，壓縮話題不會出現
        #
        # ★★ 2026-09-06 查到的一件事：**hmi_server 的 video_quality 是死參數** ★★
        #   HMI 預設 video_width=640，而這台相機出來就是 640 ——
        #   _decide_passthrough() 判定尺寸相符就走「直接轉發位元組」，
        #   完全不重新編碼。所以平板實際收到的品質是**這裡的 jpeg_quality**，
        #   不是 hmi_server 的 video_quality=55。改那個沒有任何效果。
        #   （這是對的設計，重編碼只會多一次有損又多花 CPU；但參數名字會騙人。）
        #
        # 想再省 CPU 與 WiFi 就調這個。85 -> 70 大約省下三成位元組。
        # ⚠ 預設留 85 不動：這條話題**同時餵給 face_embedding**，
        #   壓縮強度會影響人臉向量。發表前不要為了省 CPU 動辨識輸入，
        #   要動就先跑一次註冊＋辨識確認認得出來。
        #
        # ★ 一定要包 ParameterValue(..., value_type=int)：LaunchConfiguration
        #   展開出來是**字串**，直接塞進參數字典 republish 會收到字串型別的
        #   jpeg_quality 而拋 InvalidParameterTypeException。
        parameters=[{
            "format": "jpeg",
            "jpeg_quality": ParameterValue(LaunchConfiguration("jpeg_quality"), value_type=int),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "color_fps", default_value="15",
            description="相機彩色幀率。下游最高只用 6（HMI），原廠預設 30 有 4/5 是白做的"),
        DeclareLaunchArgument(
            "jpeg_quality", default_value="85",
            description="republish 的 JPEG 品質。★ 這條同時餵人臉辨識，改動前要重測辨識"),
        camera,
        compress_color,
    ])
