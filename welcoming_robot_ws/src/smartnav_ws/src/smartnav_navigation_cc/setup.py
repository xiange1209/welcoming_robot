import os
import glob
from setuptools import find_packages, setup

package_name = "smartnav_navigation_cc"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob.glob(os.path.join("launch", "*.py"))),
        (os.path.join("share", package_name, "config"), glob.glob(os.path.join("config", "*"))),
        (
            os.path.join("share", package_name, "behavior_trees"),
            glob.glob(os.path.join("behavior_trees", "*.xml")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="104887107+xiange1209@users.noreply.github.com",
    description="SmartNav 導航功能包 (_cc 重寫版) - wheeltec senior_akm 阿克曼底盤的建圖與導航",
    license="MIT",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "map_service_cc = smartnav_navigation_cc.map_service_cc_node:main",
            "waypoint_service_cc = smartnav_navigation_cc.waypoint_service_cc_node:main",
            "navigation_action_cc = smartnav_navigation_cc.navigation_action_cc_node:main",
            "fake_robot_cc = smartnav_navigation_cc.fake_robot_cc_node:main",
            "camera_manager_cc = smartnav_navigation_cc.camera_manager_cc_node:main",
            "steering_trim_cc = smartnav_navigation_cc.steering_trim_cc_node:main",
            "imu_bias_corrector_cc = smartnav_navigation_cc.imu_bias_corrector_cc_node:main",
            "stuck_detector_cc = smartnav_navigation_cc.stuck_detector_cc_node:main",
            "depth_obstacle_cc = smartnav_navigation_cc.depth_obstacle_cc_node:main",
        ],
    },
)
