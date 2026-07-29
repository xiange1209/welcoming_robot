"""深度相機避障模式 (_cc)

## 為什麼需要

雷達裝在離地 0.11 m，**比它矮的東西完全掃不到**：地上的雜物、電線、
地毯捲邊、椅腳橫桿。實測車子就是這樣撞上去卡住的 —— costmap 上一片空白，
規劃器與 collision_monitor 都認為前方淨空。

深度相機看得到這些東西，所以用它補上雷達的視野死角。

## 為什麼不直接開原廠的相機 launch

原廠 `wheeltec_camera.launch.py` 開的是**完整功能**：
彩色 640x480@30fps + 深度 640x480@30fps + IR + 點雲 + JPEG 壓縮轉發。
實測吃掉約 110% CPU（`astra_camera_node` 92% + `republish` 17%），
load average 一度衝到 52，導航直接失敗。

但**避障只需要深度**。這支 launch 把不需要的全部關掉：

    彩色影像    關閉   （避障用不到；人臉辨識另外開，見 camera_manager_cc）
    IR 影像     關閉
    彩色點雲    關閉   （只要幾何，不要顏色）
    深度解析度  320x240（原本 640x480，資料量 1/4）
    驅動點雲    關閉   （改用 depth_obstacle_cc，見下）
    JPEG 壓縮   不啟動 （那是給 HMI 網頁看的）

## 幀率降不下來，所以要改降下游

一開始這裡是設 `depth_fps:=6`，但**沒有生效**。Astra 驅動收到硬體不支援的
幀率時不會報錯，而是靜默回退成預設值 —— log 裡這三行就是證據：

    [WARN] Video mode Resolution :320x240@6Hz
    format PIXEL_FORMAT_DEPTH_1_MM is not supported.
    [WARN] Default video mode Resolution :320x240@30Hz     <- 實際跑這個

彩色更誇張：要 5 Hz，回退成 **60 Hz**。所以「降幀率省 CPU」這條路是死的。

實測 Pi4 上 `astra_camera_node` 的成本：

    深度 + 彩色 + 點雲      70.6%    load 37.0
    深度 + 點雲（關彩色）    61.8%    load 30.1
    深度、不含點雲          28.2%    load 22.6

**點雲生成一項就吃掉 34 個百分點**。原因是驅動每秒固定把 30 張深度圖
各轉成 76800 點的 XYZ 雲（27 MB/s），而避障根本用不到這個量。

因此改成：驅動只出深度圖，點雲由 `depth_obstacle_cc` 自己生 ——
降頻到 5 Hz、每 4 個像素取一點，資料量約為驅動點雲的 1/100。

## 點雲如何進到 costmap

`nav2_senior_akm_cc.yaml` 的 costmap 加入第二個 observation source
（`camera_cloud`，型別 PointCloud2，topic `/camera/obstacle_points`），
並用 `min_obstacle_height` / `max_obstacle_height` 只取**地面以上、
車高以下**的點 —— 地板本身不能被當成障礙，天花板與高處的東西也不必理會。

用法：
    ros2 launch smartnav_navigation_cc camera_obstacle_cc.launch.py

要人臉辨識用的彩色影像時，改用 camera_manager_cc（會開完整功能）。
兩者不要同時啟動 —— 相機硬體只能被一個行程開啟。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    astra_share = get_package_share_directory("astra_camera")
    astra_launch = os.path.join(astra_share, "launch", "astra.launch.xml")

    depth_width = LaunchConfiguration("depth_width")
    depth_height = LaunchConfiguration("depth_height")
    depth_fps = LaunchConfiguration("depth_fps")
    with_color = LaunchConfiguration("with_color")
    color_width = LaunchConfiguration("color_width")
    color_height = LaunchConfiguration("color_height")
    color_fps = LaunchConfiguration("color_fps")
    cloud_hz = LaunchConfiguration("cloud_hz")
    pixel_step = LaunchConfiguration("pixel_step")

    camera = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(astra_launch),
        launch_arguments=[
            # === 只留深度圖；點雲改由 depth_obstacle_cc 自己生 ===
            ("enable_depth", "true"),
            # 驅動的點雲一定要關 —— 它是 astra_camera_node 最大的單一成本
            # （61.8% -> 28.2%），而且幀率降不下來（見檔頭說明）。
            ("enable_point_cloud", "false"),
            # 彩色：避障用不到，但 HMI 的畫面需要它。
            # 預設關閉；要在 HMI 上看畫面就帶 with_color:=true，
            # 並且用很低的解析度與幀率 (320x240@5fps) —— 只是要「看得到」，
            # 不是要做人臉辨識 (那個另外用 camera_manager_cc 開完整規格)。
            ("enable_color", with_color),
            ("color_width", color_width),
            ("color_height", color_height),
            ("color_fps", color_fps),
            ("enable_ir", "false"),
            ("enable_colored_point_cloud", "false"),
            # d2c viewer 是給人看的除錯畫面
            ("enable_d2c_viewer", "false"),
            # === 降解析度與幀率 ===
            ("depth_width", depth_width),
            ("depth_height", depth_height),
            ("depth_fps", depth_fps),
        ],
    )

    # 深度圖 -> 稀疏障礙點雲。降頻 + 降採樣後資料量約為驅動點雲的 1/100。
    depth_obstacle = Node(
        package="smartnav_navigation_cc",
        executable="depth_obstacle_cc",
        name="depth_obstacle_cc",
        output="screen",
        parameters=[
            {
                "depth_topic": "/camera/depth/image_raw",
                "info_topic": "/camera/depth/camera_info",
                "output_topic": "/camera/obstacle_points",
                "target_hz": cloud_hz,
                "pixel_step": pixel_step,
                "min_range": 0.25,
                "max_range": 2.5,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("depth_width", default_value="320"),
            DeclareLaunchArgument("depth_height", default_value="240"),
            # 硬體只吃 30；設別的值驅動會靜默回退成 30，等於白設。
            # 真正的降頻在 depth_obstacle_cc 的 cloud_hz。
            DeclareLaunchArgument("depth_fps", default_value="30"),
            # HMI 畫面用。預設關閉以省 CPU；要看畫面就 with_color:=true。
            # 注意彩色也一樣會靜默回退 —— 要 5 Hz 實際會跑 60 Hz。
            DeclareLaunchArgument("with_color", default_value="false"),
            DeclareLaunchArgument("color_width", default_value="320"),
            DeclareLaunchArgument("color_height", default_value="240"),
            DeclareLaunchArgument("color_fps", default_value="5"),
            # 避障用的點雲更新率：車速 0.25 m/s，5 Hz 時每幀間隔 5 公分。
            DeclareLaunchArgument("cloud_hz", default_value="5.0"),
            DeclareLaunchArgument("pixel_step", default_value="4"),
            LogInfo(
                msg=(
                    "\n===== 深度相機避障模式 =====\n"
                    "驅動：只開深度 320x240（彩色/IR/驅動點雲全關）\n"
                    "點雲：depth_obstacle_cc 降頻 5 Hz、每 4 像素取一點\n"
                    "目的：補上雷達 (裝在 0.11 m 高) 看不到的低矮障礙\n"
                    "進 costmap 的 topic 是 /camera/obstacle_points\n"
                    "==========================\n"
                )
            ),
            camera,
            depth_obstacle,
        ]
    )
