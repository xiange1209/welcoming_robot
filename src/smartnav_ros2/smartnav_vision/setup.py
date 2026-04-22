from setuptools import find_packages, setup

package_name = "smartnav_vision"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
        "insightface>=0.7.3",
        "opencv-python>=4.11.0",
        "numpy>=1.26.4",
        "onnxruntime>=1.24.4",
    ],
    zip_safe=True,
    maintainer="swient",
    maintainer_email="swient@todo.todo",
    description="SmartNav 視覺核心功能包 - 人臉檢測、特徵提取與識別",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "face_recognition = smartnav_vision.face_recognition_node:main",
            "face_registration = smartnav_vision.face_registration_node:main",
        ],
    },
)
