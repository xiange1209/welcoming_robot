"""影像編解碼小工具。"""

import base64
import binascii

import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage

from ...core.constants import MAX_PHOTO_BYTES


def decode_photo(payload: str) -> CompressedImage:
    """把 base64 或 data URL 字串轉成 CompressedImage

    照片走 JSON+base64 而不是 multipart，是為了不必在 Pi 上多裝 python-multipart。
    """
    data = payload.split(",", 1)[1] if payload.startswith("data:") else payload
    try:
        raw = base64.b64decode(data, validate=True)
    except (binascii.Error, ValueError):
        raise ValueError("照片不是有效的 base64 資料")
    if not raw:
        raise ValueError("照片內容是空的")
    if len(raw) > MAX_PHOTO_BYTES:
        raise ValueError(f"單張照片超過 {MAX_PHOTO_BYTES // (1024 * 1024)} MB 上限")

    msg = CompressedImage()
    msg.format = "jpeg"
    msg.data = raw
    return msg


def placeholder_jpeg() -> bytes:
    """相機還沒有資料時顯示的佔位圖——比讓 <img> 一直轉圈好判讀"""
    img = np.full((360, 640, 3), 30, dtype=np.uint8)
    cv2.putText(img, "NO CAMERA SIGNAL", (120, 190), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (90, 90, 200), 2)
    ok, buf = cv2.imencode(".jpg", img)
    return buf.tobytes() if ok else b""
