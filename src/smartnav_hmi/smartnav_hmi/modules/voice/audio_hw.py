"""機器人本機的音訊硬體偵測（★ 2026-11-03，語音頁切到「機器人麥克風／喇叭」時提示用）。

只讀 /proc/asound（便宜、不開裝置、不會搶走正在錄音的麥克風）。

★ 喇叭偵測的極限：Pi 的 3.5 mm 耳機孔在 ALSA 裡**永遠存在**，有沒有插喇叭硬體上量不到。
  所以喇叭分三級：USB 音效卡（偵測得到）／只有耳機孔（無法確認）／完全沒有播放裝置。
  HDMI 不算喇叭（接螢幕才有聲音，而且迎賓機不接螢幕）。
"""

import re
from typing import Dict, List


def _read(path: str) -> str:
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as fp:
            return fp.read()
    except OSError:
        return ""


def _cards() -> Dict[int, str]:
    """卡號 -> 「驅動 - 名稱」"""
    out = {}
    for line in _read("/proc/asound/cards").splitlines():
        m = re.match(r"\s*(\d+)\s+\[[^\]]*\]:\s*(.+)$", line)
        if m:
            out[int(m.group(1))] = m.group(2).strip()
    return out


def probe() -> Dict[str, object]:
    cards = _cards()
    capture: List[str] = []
    playback_usb: List[str] = []
    playback_jack: List[str] = []
    for line in _read("/proc/asound/pcm").splitlines():
        m = re.match(r"(\d+)-(\d+):\s*([^:]*):\s*([^:]*):(.*)$", line)
        if not m:
            continue
        card = int(m.group(1))
        desc = cards.get(card, m.group(3).strip())
        rest = m.group(5)
        name = f"{m.group(4).strip() or m.group(3).strip()}（卡 {card}）"
        if "capture" in rest:
            capture.append(name)
        if "playback" in rest:
            low = desc.lower()
            if "usb" in low:
                playback_usb.append(name)
            elif "hdmi" in low:
                pass
            else:
                playback_jack.append(name)

    if playback_usb:
        speaker, speaker_detail = "usb", "USB 音效裝置：" + "、".join(playback_usb)
    elif playback_jack:
        speaker, speaker_detail = "jack", "只有耳機孔（" + "、".join(playback_jack) + "）——硬體上無法確認有沒有插喇叭"
    else:
        speaker, speaker_detail = "none", "沒有任何播放裝置"
    return {
        "mic": bool(capture),
        "mic_detail": "、".join(capture) if capture else "沒有任何錄音裝置（/proc/asound 沒有 capture）",
        "speaker": speaker,
        "speaker_detail": speaker_detail,
    }
