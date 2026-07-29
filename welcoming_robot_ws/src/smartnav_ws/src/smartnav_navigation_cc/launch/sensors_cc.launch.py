"""底盤 + 雷達 + IMU 零偏補償 (_cc)

取代 `turn_on_wheeltec_robot wheeltec_sensors.launch.py`，差別有兩點：

  1. **在 IMU 與 EKF 之間插入零偏補償**。
     這台車的陀螺儀靜止時就有 +4.54 度/分鐘的零偏，會被 EKF 積分進
     odom 的 yaw，讓車子靜置越久位姿偏得越多 (見 imu_bias_corrector_cc_node)。
     EKF 讀哪個 IMU 話題是 ekf.yaml 的 imu0 參數決定的，這裡用參數覆寫
     指到補償後的話題，**不需要改動 turn_on_wheeltec_robot 的原廠設定檔**。

  2. **相機預設不啟動**。
     相機相關行程實測吃掉約 110% CPU，是導航失敗的主因。
     需要人臉辨識時由 camera_manager_cc 按階段開啟。
     要一起啟動相機就帶 with_camera:=true。

用法：
    ros2 launch smartnav_navigation_cc sensors_cc.launch.py
    ros2 launch smartnav_navigation_cc sensors_cc.launch.py with_camera:=true

**校準期間車子必須靜止** —— 啟動後前幾秒會取樣算零偏，那時移動車子會把
真實旋轉當成零偏記下來。之後可隨時呼叫 /recalibrate_imu_bias 重新校準。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "smartnav_navigation_cc"


def generate_launch_description():
    wheeltec_share = get_package_share_directory("turn_on_wheeltec_robot")
    launch_dir = os.path.join(wheeltec_share, "launch")
    ekf_config = os.path.join(wheeltec_share, "config", "ekf.yaml")

    with_camera = LaunchConfiguration("with_camera")
    unbiased_topic = LaunchConfiguration("imu_output_topic")

    # 底盤序列埠 (發布 /imu/data_raw、輪速里程計)
    base_serial = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "base_serial.launch.py")),
    )
    # 雷達
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "wheeltec_lidar.launch.py")),
    )
    # URDF / robot_state_publisher
    robot_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "robot_mode_description.launch.py")),
    )
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "wheeltec_camera.launch.py")),
        condition=IfCondition(with_camera),
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )

    imu_corrector = Node(
        package=PKG,
        executable="imu_bias_corrector_cc",
        name="imu_bias_corrector_cc_node",
        output="screen",
        parameters=[{"input_topic": "/imu/data_raw", "output_topic": unbiased_topic}],
    )

    # EKF：用參數覆寫把 imu0 指到補償後的話題。
    # 其餘設定全部沿用原廠 ekf.yaml，remap 也跟原廠一致 (odometry/filtered -> odom_combined)。
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_config,
            {
                "imu0": unbiased_topic,
                # 原廠 imu0_config 的第 6 個 (絕對 yaw) 是 true，也就是把 IMU
                # 自己算出來的 orientation.yaw 當成絕對姿態測量餵給 EKF。
                #
                # 這台車沒有磁力計，IMU 的 yaw 是它內部積分角速度得到的，
                # **本身就已經累積了陀螺儀零偏的漂移**。只補償 angular_velocity
                # 完全沒用 —— EKF 照樣從 orientation.yaw 拿到漂掉的值，
                # 實測補償前後 odom 都是 60 秒漂約 +5 度，一模一樣。
                #
                # 這裡把絕對 yaw 關掉 (索引 5 -> false)，只保留角速度
                # (索引 11 = yaw_vel，維持 true)。角速度已經扣掉零偏，
                # EKF 自己積分就不會漂。這也是無磁力計 IMU 的標準做法：
                # 相對角速度可信，絕對航向不可信。
                "imu0_config": [
                    False, False, False,   # x, y, z 位置
                    False, False, False,   # roll, pitch, yaw   <- yaw 由 true 改為 false
                    False, False, False,   # x, y, z 線速度
                    False, False, True,    # roll_vel, pitch_vel, yaw_vel
                    False, False, False,   # x, y, z 加速度
                ],
            },
        ],
        remappings=[("/odometry/filtered", "odom_combined")],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("with_camera", default_value="false"),
            DeclareLaunchArgument("imu_output_topic", default_value="/imu/data_unbiased"),
            LogInfo(
                msg=(
                    "\n===== 感測器啟動 (含 IMU 零偏補償) =====\n"
                    "★ 校準期間請保持車子靜止，否則零偏會算錯 ★\n"
                    "相機預設不啟動 (省 110% CPU)，需要時用 with_camera:=true\n"
                    "======================================\n"
                )
            ),
            base_serial,
            joint_state_publisher,
            robot_desc,
            imu_corrector,
            ekf,
            lidar,
            camera,
        ]
    )
