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
        # ★ 2026-08-14：config/ 原本沒被安裝，所以放在裡面的檔案 colcon build
        #   之後**不會出現在 share/ 底下**，節點自然找不到 —— 而且不會報錯，
        #   只會安靜地少一個功能（熱詞）。加上 hotwords.txt 時踩到，補上規則。
        *[
            (os.path.join("share", package_name, os.path.dirname(f)), [f])
            for f in glob.glob("config/**", recursive=True)
            if os.path.isfile(f)
        ],
    ],
    install_requires=["setuptools"],
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
