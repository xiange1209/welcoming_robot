import os
import glob
from setuptools import find_packages, setup

package_name = "smartnav_navigation"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob.glob(os.path.join("launch", "*.py"))),
        (os.path.join("share", package_name, "config"), glob.glob(os.path.join("config", "*"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="swient",
    maintainer_email="swient@todo.todo",
    description="SmartNav 移動導航功能包 - 地圖服務、路徑規劃、導航控制",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "map_service = smartnav_navigation.map_service_node:main",
            "waypoint_service = smartnav_navigation.waypoint_service_node:main",
            "navigation_action = smartnav_navigation.navigation_action_node:main",
        ],
    },
)
