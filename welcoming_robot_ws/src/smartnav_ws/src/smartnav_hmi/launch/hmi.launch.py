"""HMI 啟動檔

單一節點就提供完整平板介面：網頁＋影像串流＋地圖＋狀態推播＋所有服務的 HTTP 閘道，
不需要另外啟動 rosbridge 或 web_video_server。

用法：
    ros2 launch smartnav_hmi hmi.launch.py
    ros2 launch smartnav_hmi hmi.launch.py \\
        image_capture_topic:=/camera/color/image_raw/compressed robot_frame:=base_footprint
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
    # ★★ 2026-08-20：預設從 "image_raw" 改成實際存在的話題 ★★
    #   本檔的 ARGS 是**無條件**展開成參數傳給 Node 的（見下方 Node(parameters=...)），
    #   所以不帶參數啟動時一定會蓋掉節點自己正確的預設值
    #   （hmi_server_node.py:766 = /camera/color/image_raw/compressed）。
    #   而全 workspace **沒有任何節點發布 /image_raw** ——
    #   astra 發的是 /camera/color/image_raw，compressed 那條由廠商
    #   wheeltec_camera.launch.py 的 image_transport/republish 發。
    #   後果：平板顯示「NO CAMERA SIGNAL」佔位圖，迎賓頁、註冊預覽、
    #   「從畫面拍照」全掛，段一起手就停住。
    #   ★ 選 compressed 而非 raw：Pi4 上原始幀 640x480x3 = 900 KB，
    #     壓縮那條省非常多，而 HMI 只是要顯示。
    ("image_capture_topic", "/camera/color/image_raw/compressed", str,
     "相機影像話題（壓縮那條由廠商 republish 發，Pi4 上比 raw 省很多）"),
    ("image_transport", "auto", str, "auto／raw／compressed。auto 依話題是否以 /compressed 結尾判斷"),
    ("user_identity_topic", "user_identity", str, "身份辨識結果話題（由 user_auth_node 發布）"),
    ("map_topic", "map", str, "佔據柵格地圖話題（slam_toolbox 或 map_server 發布）"),
    ("map_frame", "map", str, "地圖座標系名稱"),
    # base_footprint 而不是 base_link（2026-08-01 改）：
    # 這台車的整條鏈路——EKF 輸出、costmap 的 robot_base_frame、
    # path_teach_cc、waypoint_service_cc——全部用 base_footprint。
    # 預設留 base_link 的話，忘了帶參數就會查不到 TF，地圖上的車子圖示
    # 與位姿全部靜默失效，而畫面上看不出哪裡錯了。
    ("robot_frame", "base_footprint", str, "機器人本體座標系。這台車全鏈路用 base_footprint"),
    ("enable_map", "true", bool, "是否啟用地圖功能。純迎賓展示設 false 可省下 Pi 的 CPU"),
    ("map_render_interval", "2.0", float, "地圖最快多久重繪一次（秒）"),
    ("pose_update_rate", "2.0", float, "查詢機器人 TF 位姿的頻率（Hz）"),
    ("video_fps", "6.0", float, "MJPEG 串流上限幀率"),
    ("video_quality", "55", int, "JPEG 品質 1-100"),
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
