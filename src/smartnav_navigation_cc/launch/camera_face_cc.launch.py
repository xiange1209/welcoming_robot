"""只開彩色相機與 JPEG republish，供人臉辨識使用。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    astra = os.path.join(get_package_share_directory("astra_camera"), "launch", "astra.launch.xml")
    camera = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(astra),
        launch_arguments={
            "enable_color": "true", "enable_depth": "false", "enable_ir": "false",
            "enable_point_cloud": "false", "enable_colored_point_cloud": "false",
            "depth_registration": "false", "enable_d2c_viewer": "false",
            "color_fps": LaunchConfiguration("color_fps"),
        }.items(),
    )
    republish = Node(
        package="image_transport", executable="republish", name="republish_color_compressed",
        arguments=["raw", "compressed"],
        remappings=[("in", "/camera/color/image_raw"), ("out/compressed", "/camera/color/image_raw/compressed")],
        parameters=[{"format": "jpeg", "jpeg_quality": ParameterValue(LaunchConfiguration("jpeg_quality"), value_type=int)}],
    )
    return LaunchDescription([
        DeclareLaunchArgument("color_fps", default_value="15"),
        DeclareLaunchArgument("jpeg_quality", default_value="85"),
        camera,
        republish,
    ])
