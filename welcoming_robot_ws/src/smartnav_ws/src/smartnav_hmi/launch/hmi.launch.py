"""HMI 啟動檔

單一節點就提供完整平板介面：網頁＋影像串流＋地圖＋狀態推播＋所有服務的 HTTP 閘道，
不需要另外啟動 rosbridge 或 web_video_server。

用法：
    ros2 launch smartnav_hmi hmi.launch.py
    ros2 launch smartnav_hmi hmi.launch.py \\
        image_capture_topic:=/camera/color/image_raw robot_frame:=base_footprint
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# (參數名, 預設值, 型別, 說明)——集中一份，宣告與傳遞都從這裡展開，
# 免得新增參數時只改了一邊（舊版就漏過一次）
ARGS = [
    ("port", "8080", int, "HTTP 連接埠"),
    ("host", "0.0.0.0", str, "HTTP 綁定位址"),
    ("image_capture_topic", "image_raw", str, "相機影像話題。實車 astra 相機請設 /camera/color/image_raw"),
    ("image_transport", "auto", str, "auto／raw／compressed。auto 依話題是否以 /compressed 結尾判斷"),
    ("user_identity_topic", "user_identity", str, "身份辨識結果話題（由 user_auth_node 發布）"),
    ("map_topic", "map", str, "佔據柵格地圖話題（slam_toolbox 或 map_server 發布）"),
    ("map_frame", "map", str, "地圖座標系名稱"),
    ("robot_frame", "base_link", str, "機器人本體座標系。WHEELTEC 鏈可能要設 base_footprint"),
    ("enable_map", "true", bool, "是否啟用地圖功能。純迎賓展示設 false 可省下 Pi 的 CPU"),
    ("map_render_interval", "2.0", float, "地圖最快多久重繪一次（秒）"),
    ("pose_update_rate", "2.0", float, "查詢機器人 TF 位姿的頻率（Hz）"),
    ("video_fps", "12.0", float, "MJPEG 串流上限幀率"),
    ("video_quality", "70", int, "JPEG 品質 1-100"),
    ("video_width", "640", int, "輸出寬度，0 為不縮放"),
    ("service_timeout", "8.0", float, "呼叫 ROS 服務的等待秒數"),
]


def generate_launch_description():
    declarations = [
        DeclareLaunchArgument(name, default_value=default, description=desc)
        for name, default, _type, desc in ARGS
    ]
    parameters = {
        name: ParameterValue(LaunchConfiguration(name), value_type=type_)
        for name, _default, type_, _desc in ARGS
    }

    return LaunchDescription(
        declarations
        + [
            Node(
                package="smartnav_hmi",
                executable="hmi_server",
                name="hmi_server_node",
                output="screen",
                parameters=[parameters],
            ),
        ]
    )
