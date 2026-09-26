"""受管 ASR chain：先確認 Astra S 可開啟，再起 VAD 與辨識器。"""

from __future__ import annotations

import re
import subprocess
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def _astra_card() -> str | None:
    result = subprocess.run(["arecord", "-l"], capture_output=True, text=True, timeout=2, check=False)
    match = re.search(r"^card (\d+):.*ASTRA S", result.stdout, re.IGNORECASE | re.MULTILINE)
    return match.group(1) if match else None


def _device_index() -> int | None:
    import sounddevice as sd

    for index, device in enumerate(sd.query_devices()):
        if device["max_input_channels"] > 0 and "ASTRA" in device["name"].upper():
            return index
    return None


def _preflight(context):
    gain = context.perform_substitution(LaunchConfiguration("mic_gain"))
    last_error = "Astra S 錄音裝置不存在"
    for delay in (1, 2, 4):
        try:
            card = _astra_card()
            if not card:
                raise RuntimeError("找不到 Astra S ALSA card")
            # -d 1 讓 probe 最多持有裝置一秒；timeout 防止驅動卡死。
            probe = subprocess.run(
                [
                    "arecord",
                    "-D",
                    f"hw:{card},0",
                    "-d",
                    "1",
                    "-t",
                    "raw",
                    "-f",
                    "S16_LE",
                    "-r",
                    "16000",
                    "-c",
                    "2",
                    "/dev/null",
                ],
                capture_output=True,
                text=True,
                timeout=2,
                check=False,
            )
            if probe.returncode:
                raise RuntimeError((probe.stderr or "Astra S 裝置忙碌").strip())
            time.sleep(0.25)  # ALSA close 後讓驅動完成釋放。
            for control in ("Mic,0", "Mic,1"):
                changed = subprocess.run(
                    ["amixer", "-c", card, "sset", control, gain, "cap"],
                    capture_output=True,
                    text=True,
                    timeout=2,
                    check=False,
                )
                if changed.returncode:
                    raise RuntimeError((changed.stderr or f"無法設定 {control}").strip())
            index = _device_index()
            if index is None:
                raise RuntimeError("sounddevice 找不到 Astra S")
            recognizer = Node(
                package="smartnav_audio",
                executable="speech_recognizer",
                name="speech_recognizer_node",
                output="screen",
                parameters=[{"num_threads": 2}],
            )
            trigger = Node(
                package="smartnav_audio",
                executable="voice_trigger",
                name="voice_trigger_node",
                output="screen",
                parameters=[
                    {
                        "device": index,
                        "device_name_match": "ASTRA",
                        "device_open_retries": 3,
                        "vad_num_threads": 1,
                        "mic_mode": context.perform_substitution(LaunchConfiguration("mic_mode")),
                    }
                ],
            )
            stop_trigger = RegisterEventHandler(
                OnProcessExit(
                    target_action=trigger,
                    on_exit=[EmitEvent(event=Shutdown(reason="voice_trigger 已退出，關閉 ASR chain"))],
                )
            )
            stop_recognizer = RegisterEventHandler(
                OnProcessExit(
                    target_action=recognizer,
                    on_exit=[EmitEvent(event=Shutdown(reason="speech_recognizer 已退出，關閉 ASR chain"))],
                )
            )
            return [
                LogInfo(msg=f"ASR 使用 Astra S ALSA card={card}, sounddevice={index}"),
                recognizer,
                trigger,
                stop_trigger,
                stop_recognizer,
            ]
        except (OSError, RuntimeError, subprocess.TimeoutExpired, ImportError) as exc:
            last_error = str(exc)
            time.sleep(delay)
    raise RuntimeError(f"ASR preflight 失敗（已重試 3 次）：{last_error}")


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("mic_gain", default_value=EnvironmentVariable("ASR_MIC_GAIN", default_value="80")),
            DeclareLaunchArgument("mic_mode", default_value=EnvironmentVariable("ASR_MIC_MODE", default_value="left")),
            OpaqueFunction(function=_preflight),
        ]
    )
