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
        (os.path.join("share", package_name, "web"), glob.glob(os.path.join("web", "*.html"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="swient",
    maintainer_email="swient@todo.todo",
    description="SmartNav 人機介面功能包 - 影像串流、地圖點擊建點/導航、使用者管理、對話流",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "hmi_server = smartnav_hmi.hmi_server_node:main",
        ],
    },
)
