import os
import glob
from setuptools import find_packages, setup

package_name = "smartnav_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob.glob(os.path.join("launch", "*.py"))),
        (os.path.join("share", package_name, "config"), glob.glob(os.path.join("config", "*"))),
        (os.path.join("share", package_name, "urdf"), glob.glob(os.path.join("urdf", "*"))),
        (os.path.join("share", package_name, "worlds"), glob.glob(os.path.join("worlds", "*.sdf"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xiange1209",
    maintainer_email="104887107+xiange1209@users.noreply.github.com",
    description="SmartNav Gazebo 模擬 —— senior_akm 阿克曼車＋N10 光達（只在筆電 WSL 跑，不上車）",
    license="MIT",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "akm_realism = smartnav_sim.akm_realism_node:main",
            "pgm_to_world = smartnav_sim.pgm_to_world:main",
            "map_area = smartnav_sim.map_area:main",
        ],
    },
)
