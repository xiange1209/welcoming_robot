import os
import glob
from setuptools import find_packages, setup

package_name = "smartnav_audio"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        *[
            (os.path.join("share", package_name, os.path.dirname(f)), [f])
            for f in glob.glob("models/**", recursive=True)
            if os.path.isfile(f)
        ],
    ],
    install_requires=[
        "setuptools",
        "soxr>=1.1.0",
        "sounddevice>=0.5.5",
        "pypinyin>=0.55.0",
        "opencc>=1.3.1",
        "sherpa-onnx>=1.13.0",
    ],
    zip_safe=True,
    maintainer="swient",
    maintainer_email="swient@todo.todo",
    description="SmartNav 音訊處理功能包 - 語音喚醒、語音辨識、文字轉語音",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "voice_trigger = smartnav_audio.voice_trigger_node:main",
            "voice_playback = smartnav_audio.voice_playback_node:main",
            "speech_recognizer = smartnav_audio.speech_recognizer_node:main",
            "speech_synthesizer = smartnav_audio.speech_synthesizer_node:main",
        ],
    },
)
