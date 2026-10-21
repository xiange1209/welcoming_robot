"""受管 ASR chain：先確認 Astra S 可開啟，再起 VAD 與辨識器。"""

from __future__ import annotations

import os
import re
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
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


def _hotwords_file(value: str) -> str:
    """★ 2026-10-21：熱詞檔路徑。有給就用（展開 ~）；留空 = 套件 share 下的 config/hotwords.txt，檔案存在才用。

    不存在就回空字串 —— 節點收到空字串會照舊自己從模型目錄往上找（speech_recognizer_node 的退路）。
    """
    value = (value or "").strip()
    if value:
        return os.path.expanduser(value)
    try:
        cand = os.path.join(get_package_share_directory("smartnav_audio"), "config", "hotwords.txt")
    except Exception:  # noqa: BLE001  套件找不到 -> 交給節點自己找
        return ""
    return cand if os.path.isfile(cand) else ""


def _truthy(value: str) -> bool:
    """true／1／yes／on（不分大小寫）算開，其他都算關。"""
    return (value or "").strip().lower() in ("1", "true", "yes", "on")


def _preflight(context):
    gain = context.perform_substitution(LaunchConfiguration("mic_gain"))
    # ★ 2026-10-21：四個新參數（理由寫在 generate_launch_description）。重試迴圈外算一次就好。
    hotwords_file = _hotwords_file(context.perform_substitution(LaunchConfiguration("hotwords_file")))
    mute_during_tts = _truthy(context.perform_substitution(LaunchConfiguration("mute_during_tts")))
    debug_wav_dir = context.perform_substitution(LaunchConfiguration("debug_wav_dir")).strip()
    if debug_wav_dir:
        debug_wav_dir = os.path.expanduser(debug_wav_dir)
    audio_latency = context.perform_substitution(LaunchConfiguration("audio_latency")).strip() or "high"
    try:
        # 秒數。launch_ros 本來就會把數字字串轉成 double，這裡寫明白；節點兩種型別都收（dynamic_typing）
        audio_latency = float(audio_latency)
    except ValueError:
        pass
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
                parameters=[
                    {
                        "num_threads": 2,
                        "hotwords_file": hotwords_file,
                        "mute_during_tts": mute_during_tts,
                        "debug_wav_dir": debug_wav_dir,
                    }
                ],
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
                        "audio_latency": audio_latency,
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
                LogInfo(
                    msg=(
                        f"ASR 參數：hotwords_file={hotwords_file or '（空，交給節點自己找）'}"
                        f"、mute_during_tts={mute_during_tts}"
                        f"、debug_wav_dir={debug_wav_dir or '（關）'}"
                        f"、audio_latency={audio_latency}"
                    )
                ),
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
            # ★ 2026-09-29：80 → 60。amixer 檔位 0~120、實測 1.38 dB/格；9/17 新實驗室 66 就削波
            #   581 點，60 才正確觸發（驗證結果_20260917.md §8-1，run_asr_cc.sh 的預設也是 60）。
            DeclareLaunchArgument("mic_gain", default_value=EnvironmentVariable("ASR_MIC_GAIN", default_value="60")),
            DeclareLaunchArgument("mic_mode", default_value=EnvironmentVariable("ASR_MIC_MODE", default_value="left")),
            # ★ 2026-10-21：熱詞檔。留空 = 套件 share 下的 config/hotwords.txt（setup.py 會裝進去；存在才用，
            #   否則傳空字串、由節點自己往模型目錄找）。節點原本只會從模型目錄往上推算，
            #   模型搬到 ~/models/asr 後推不到 —— 10/20 是靠手動符號連結 ~/models/asr/hotwords.txt 才載入。
            #   有熱詞時熱詞句 CER 55.9% -> 32.4%（各 10 句，區間重疊，只能算方向）。
            DeclareLaunchArgument(
                "hotwords_file", default_value=EnvironmentVariable("ASR_HOTWORDS_FILE", default_value="")
            ),
            # ★ 2026-10-21：機器人講話（平板朗讀）時丟掉收音，預設開。
            #   取捨：客人沒辦法在機器人講話時插話，換來機器人不會聽到自己。
            #   10/20 實車：平板朗讀被收進來，文字層回音過濾 0/2（辨識太亂、比對不上），LLM 自己跟自己對話。
            #   平板朗讀起訖早就會回報（tts.ts -> /api/tts_state -> playback_status），只是節點預設沒拿來靜音。
            #   要恢復「客人可插話」：ASR_MUTE_DURING_TTS=false（true／1／yes／on 以外都算關）。
            DeclareLaunchArgument(
                "mute_during_tts", default_value=EnvironmentVariable("ASR_MUTE_DURING_TTS", default_value="true")
            ),
            # ★ 2026-10-21：除錯錄音目錄（有值時展開 ~），空 = 關。每段送進辨識器的音訊存成 WAV，
            #   查 10/20 的疊字與句首遺失用；目錄裡滿 1000 個就停（不刪檔）。
            DeclareLaunchArgument(
                "debug_wav_dir", default_value=EnvironmentVariable("ASR_DEBUG_WAV_DIR", default_value="")
            ),
            # ★ 2026-10-21：收音緩衝 low／high／秒數（例 0.2）。原本寫死 low，10/20 input overflow 全開 74 次/分。
            DeclareLaunchArgument(
                "audio_latency", default_value=EnvironmentVariable("ASR_AUDIO_LATENCY", default_value="high")
            ),
            OpaqueFunction(function=_preflight),
        ]
    )
