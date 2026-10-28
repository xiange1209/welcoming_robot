import os
import glob
from setuptools import find_packages, setup

package_name = "smartnav_llm"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob.glob(os.path.join("config", "*.txt"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="swient",
    maintainer_email="swient@todo.todo",
    description="SmartNav 迎賓對話：聽懂客人的話並以 Ollama 模型回話，需要動作時向 nav_arbiter／bank_reception 提出請求",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "llm_service = smartnav_llm.llm_service_node:main",
        ],
    },
)
