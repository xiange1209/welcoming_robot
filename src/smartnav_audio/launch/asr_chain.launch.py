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
    # ★ 2026-11-10：觸發後「不判斷靜音」的初始等待（毫秒），理由見 generate_launch_description。
    initial_wait_ms = int(float(context.perform_substitution(LaunchConfiguration("command_initial_wait_ms")) or 1000))
    # ★ 2026-11-25：收音距離閘門開關（展場人多、客人不一定站在車頭前，要關得掉），理由見 generate_launch_description。
    presence_gate = _truthy(context.perform_substitution(LaunchConfiguration("presence_gate")))
    def _nodes(index: int, desc: str) -> list:
        """建立 speech_recognizer 與 voice_trigger（index＝Astra S 的 sounddevice 編號；-1＝沒有本機麥克風）"""
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
                    "command_initial_wait_timeout": initial_wait_ms,
                    "vad_threshold": _vad_threshold_for_input(),
                    "presence_gate_enable": presence_gate,
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
            LogInfo(msg=f"ASR 使用 {desc}"),
            LogInfo(
                msg=(
                    f"ASR 參數：hotwords_file={hotwords_file or '（空，交給節點自己找）'}"
                    f"、mute_during_tts={mute_during_tts}"
                    f"、debug_wav_dir={debug_wav_dir or '（關）'}"
                    f"、audio_latency={audio_latency}"
                    f"、presence_gate={'開' if presence_gate else '關'}"
                )
            ),
            recognizer,
            trigger,
            stop_trigger,
            stop_recognizer,
        ]

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
            return _nodes(index, f"Astra S ALSA card={card}, sounddevice={index}")
        except (OSError, RuntimeError, subprocess.TimeoutExpired, ImportError) as exc:
            last_error = str(exc)
            time.sleep(delay)
    # ★ 2026-11-24：平板收音（預設）根本用不到 Astra 的麥克風，以前照樣要求 Astra 存在 →
    #   車子重開後相機沒抓到（Astra S 要手動插拔）時，語音輸入整條起不來（11/24 18:13～18:17 連 4 次失敗）。
    #   平板模式改成警告後照常啟動；之後在語音頁切到相機麥克風時，voice_trigger 自己會回報「開不起來」。
    if _voice_input_mode() == "browser":
        return [LogInfo(msg=f"⚠ 找不到 Astra S（{last_error}）；目前是平板收音，照常啟動（相機麥克風模式暫時不能用）")] + _nodes(
            -1, "平板收音（沒有 Astra S）")
    raise RuntimeError(f"ASR preflight 失敗（已重試 3 次）：{last_error}")


def _vad_threshold_for_input() -> float:
    """語音觸發門檻（Silero VAD 機率）：依啟動當下的收音來源決定。

    ★ 2026-11-24：平板收音開著自動增益，遠處、旁人的話也被放大到觸發；原本的 0.25 是為了相機麥克風
      聲音小才調低的（voice_trigger_node.py 的說明）。離線測試：乾淨人聲在 0.25 和 0.5 的語音比例幾乎一樣
      （82%→81%、95%→95%），擋掉的主要是雜音與悶、遠的人聲。
    依 ~/.smartnav/voice_settings.json 的 input：browser（平板，預設）→ 0.5；robot（相機麥克風）→ 0.25。
    在語音頁中途切換來源後，要把「語音輸入」停掉再開才會換門檻。環境變數 ASR_VAD_THRESHOLD 可直接指定。
    """
    env = os.environ.get("ASR_VAD_THRESHOLD", "").strip()
    if env:
        try:
            return float(env)
        except ValueError:
            pass
    return 0.25 if _voice_input_mode() == "robot" else 0.5


def _voice_input_mode() -> str:
    """~/.smartnav/voice_settings.json 的 input：browser（平板，預設）或 robot（相機麥克風）。"""
    try:
        import json
        from pathlib import Path
        data = json.loads((Path.home() / ".smartnav" / "voice_settings.json").read_text(encoding="utf-8"))
        return str(data.get("input", "browser"))
    except (OSError, ValueError):
        return "browser"


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
            # ★ 2026-10-27：預設改 0.2 秒。這個麥克風（Astra S）的 "high" 實際只有 32 ms（log：latency='high' -> 實際 32.0 ms），
            #   等於沒加大：10/27 demo 全開時溢位 21~145 次/分。給秒數讓 PortAudio 多留緩衝；收音還是每 32 ms 送一塊，
            #   只有程式被卡住時才會用到多的緩衝。這個值開不起來時 voice_trigger 會自己退回 "high"。
            DeclareLaunchArgument(
                "audio_latency", default_value=EnvironmentVariable("ASR_AUDIO_LATENCY", default_value="0.2")
            ),
            # ★ 2026-11-10：voice_trigger 觸發後前 N 毫秒「不判斷靜音」（command_initial_wait_timeout），節點預設 3000。
            #   這條鏈是講話本身觸發（不是喚醒詞），3 秒等於強迫短句錄滿 3 秒：11/10 實車平板收音 10 句，
            #   短句「最後一次出字 → 最終結果」2.0～2.6 秒，超過 3 秒的長句只要 0.6～0.8 秒（＝500 ms 靜音＋辨識）。
            #   改 1000：短句也約 0.6～0.8 秒收尾。代價：說完第一個詞後停超過 0.5 秒會被切成兩句（3 秒以後本來就是這樣）。
            #   要恢復舊行為：ASR_COMMAND_INITIAL_WAIT_MS=3000。
            DeclareLaunchArgument(
                "command_initial_wait_ms",
                default_value=EnvironmentVariable("ASR_COMMAND_INITIAL_WAIT_MS", default_value="1000"),
            ),
            # ★ 2026-11-25：收音距離閘門（車頭前方 1 公尺有人才收音，presence_gate.py）。預設開。
            #   從平板啟動也要關得掉：smartnav-hmi 的 systemd drop-in 加 Environment=SMARTNAV_PRESENCE_GATE=0；
            #   不重啟就關：ros2 param set /voice_trigger_node presence_gate_enable false。
            DeclareLaunchArgument(
                "presence_gate", default_value=EnvironmentVariable("SMARTNAV_PRESENCE_GATE", default_value="1")
            ),
            OpaqueFunction(function=_preflight),
        ]
    )
