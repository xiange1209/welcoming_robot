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
        (os.path.join("share", package_name, "knowledge"), glob.glob(os.path.join("knowledge", "*.md"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="swient",
    maintainer_email="swient@todo.todo",
    description="SmartNav LLM 功能包 - 聊天對話、指令解析",
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
