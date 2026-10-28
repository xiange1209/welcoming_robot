import os
import glob
from setuptools import find_packages, setup

package_name = "smartnav_hmi"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob.glob(os.path.join("launch", "*.py"))),
        (os.path.join("share", package_name, "config"), glob.glob(os.path.join("config", ".env"))),
        *[
            (os.path.join("share", package_name, os.path.dirname(f)), [f])
            for f in glob.glob(os.path.join("frontend", "dist", "**"), recursive=True)
            if os.path.isfile(f)
        ],
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="swient",
    maintainer_email="swient@todo.todo",
    description="SmartNav 人機介面：影像串流、管理者的地圖與使用者管理、迎賓頁、系統控制與緊急停止（導航一律經 nav_arbiter）",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "hmi_server = smartnav_hmi.hmi_server_node:main",
            "system_control = smartnav_hmi.system_control_cli:main",
        ],
    },
)
