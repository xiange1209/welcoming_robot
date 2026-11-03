"""語音輸出／輸入的全系統設定（★ 2026-11-03）。

  output  browser = 平板用瀏覽器 speechSynthesis 唸（原本唯一的方式）
          robot   = 機器人上的 speech_synthesizer（sherpa-onnx）＋ voice_playback（耳機孔喇叭）
  input   browser = （預設）平板麥克風錄音 -> /ws/mic -> remote_mic_audio -> voice_trigger -> speech_recognizer
                    ★ 辨識一律在機器人上做（使用者決定：客人的錄音不送到 Google／Apple 雲端）
          robot   = 機器人麥克風 -> voice_trigger（VAD）-> speech_recognizer（改版前的方式）

  robot_tts  （★ 2026-11-03）output=robot 時的「機器人朗讀」開關，像平板的朗讀鈕一樣可以關。
             關 = speech_synthesizer 不合成、voice_playback 立刻停掉正在播的。

存在 ~/.smartnav/voice_settings.json，所以每台平板看到的是同一個設定、重開機也記得。
HMI 啟動時與每次變更時，用 latched 話題 voice_output_mode / voice_input_mode 廣播，
speech_synthesizer 與 voice_trigger 依此決定要不要動作。
"""

import json
import threading
from pathlib import Path
from typing import Dict, Optional

MODES = ("browser", "robot")
# ★ 2026-11-03 使用者要求：語音輸入預設＝平板錄音（車上目前沒有麥克風）。輸出預設仍是平板朗讀
DEFAULTS = {"output": "browser", "input": "browser", "robot_tts": True}


class VoiceSettings:
    def __init__(self, path: Optional[Path] = None, logger=None):
        self.path = path or (Path.home() / ".smartnav" / "voice_settings.json")
        self._logger = logger
        self._lock = threading.Lock()
        self._data: Dict[str, str] = dict(DEFAULTS)
        self._load()

    def _load(self) -> None:
        try:
            raw = json.loads(self.path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return
        except Exception as exc:  # noqa: BLE001 —— 壞檔不能讓 HMI 起不來，退回預設
            if self._logger:
                self._logger.warn(f"語音設定檔讀取失敗，改用預設值：{exc}")
            return
        for key in ("output", "input"):
            if raw.get(key) in MODES:
                self._data[key] = raw[key]
        if isinstance(raw.get("robot_tts"), bool):
            self._data["robot_tts"] = raw["robot_tts"]

    def get(self) -> Dict[str, str]:
        with self._lock:
            return dict(self._data)

    def update(self, output: Optional[str] = None, input_: Optional[str] = None,
               robot_tts: Optional[bool] = None) -> Dict[str, object]:
        """更新並寫檔。值不合法就丟 ValueError（呼叫端回 400）"""
        for value in (output, input_):
            if value is not None and value not in MODES:
                raise ValueError(f"模式只能是 {' / '.join(MODES)}，收到 {value!r}")
        with self._lock:
            if output is not None:
                self._data["output"] = output
            if input_ is not None:
                self._data["input"] = input_
            if robot_tts is not None:
                self._data["robot_tts"] = bool(robot_tts)
            data = dict(self._data)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(data, ensure_ascii=False, indent=1), encoding="utf-8")
        tmp.replace(self.path)                        # 原子替換：寫到一半斷電也不會留下半個檔
        return data
