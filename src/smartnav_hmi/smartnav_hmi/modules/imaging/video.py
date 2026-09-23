"""相機影像：解碼/編碼、按需訂閱、MJPEG 串流。

從 hmi_server_node.py 搬出來，把影像相關的所有狀態與方法集中成一個元件。
仍然吃 `node`（建立/銷毀訂閱要用到 create_subscription/destroy_subscription，
這是 rclpy Node 的方法，見 core/README 同一套取捨）。
"""

import asyncio
import threading
import time
from typing import List, Optional

import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image

from .imaging import placeholder_jpeg


class VideoStreamer:
    # 沒有相機影像時，佔位圖的重送間隔（秒）
    PLACEHOLDER_INTERVAL_SEC = 1.0

    def __init__(
        self,
        node,
        state,
        *,
        image_topic: str,
        image_compressed: bool,
        sensor_qos,
        video_fps: float,
        video_quality: int,
        video_width: int,
        video_idle_ttl: float,
    ):
        self._node = node
        self.state = state
        self.image_topic = image_topic
        self.image_compressed = image_compressed
        self._sensor_qos = sensor_qos
        self.video_fps = video_fps
        self.video_quality = video_quality
        self.video_width = video_width
        self.video_idle_ttl = video_idle_ttl

        self.bridge = CvBridge()
        self._frame_lock = threading.Lock()
        self._latest_jpeg: Optional[bytes] = None
        self._last_encode_time = 0.0
        self._frame_times: List[float] = []
        # 壓縮影像是否能直接轉發位元組（None = 還沒判斷過，用第一張影格決定）
        self._passthrough: Optional[bool] = None
        self._image_sub = None

        # /video 串流數與「剛剛有人要影格」的時間戳，各自有一把鎖，
        # 跟節點的用戶端在場追蹤（ws/http）分開，理由是這兩個量只跟影像有關。
        self._client_lock = threading.Lock()
        self._video_viewers = 0
        self._last_frame_req_at = 0.0

    # ── 用戶端在場（影像專屬部分）────────────────────────

    def note_interest(self) -> None:
        """記下「剛剛有人要影格」（/api/frame.jpg 拍照用）"""
        with self._client_lock:
            self._last_frame_req_at = time.monotonic()

    def wants_stream(self) -> bool:
        """要不要維持影像訂閱（還要再跟 clients_present() 交叉驗證，見 hmi_server_node._video_wanted）"""
        with self._client_lock:
            if self._video_viewers > 0:
                return True
            return (time.monotonic() - self._last_frame_req_at) < self.video_idle_ttl

    def latest_jpeg(self) -> Optional[bytes]:
        with self._frame_lock:
            return self._latest_jpeg

    # ── 影格處理 ──────────────────────────────────────────

    def _should_process_frame(self) -> Optional[float]:
        """依 video_fps 節流。要處理這一影格時回傳當下時間，否則回 None"""
        now = time.monotonic()
        if now - self._last_encode_time < 1.0 / max(self.video_fps, 1.0):
            return None
        self._last_encode_time = now
        return now

    def _encode_frame(self, frame: np.ndarray) -> Optional[bytes]:
        """縮放（如有需要）並編成 JPEG"""
        if self.video_width and frame.shape[1] != self.video_width:
            scale = self.video_width / frame.shape[1]
            frame = cv2.resize(frame, (self.video_width, int(frame.shape[0] * scale)))
        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.video_quality])
        return buf.tobytes() if ok else None

    def _publish_frame(self, payload: bytes, now: float) -> None:
        """存下最新影格並更新幀率統計（只留最近兩秒）"""
        with self._frame_lock:
            self._latest_jpeg = payload

        self._frame_times.append(now)
        self._frame_times = [t for t in self._frame_times if now - t <= 2.0]
        if len(self._frame_times) >= 2:
            span = self._frame_times[-1] - self._frame_times[0]
            if span > 0:
                self.state.set_system(camera_fps=round((len(self._frame_times) - 1) / span, 1))

    def image_cb(self, msg: Image) -> None:
        """原始影像（sensor_msgs/Image）：解成 BGR → 縮放 → 編 JPEG"""
        now = self._should_process_frame()
        if now is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            payload = self._encode_frame(frame)
        except Exception as e:  # 影像壞掉不該弄垮整個 HMI
            self._node.get_logger().warn(f"影像轉換失敗: {e}", throttle_duration_sec=5.0)
            return
        if payload is not None:
            self._publish_frame(payload, now)

    def compressed_image_cb(self, msg: CompressedImage) -> None:
        """壓縮影像（sensor_msgs/CompressedImage）

        上游已經是 JPEG 而且尺寸就是我們要輸出的尺寸時，**直接把位元組轉發出去**——
        解碼再重新編碼一次只是白白浪費 Pi 的 CPU，還會多一次有損壓縮。
        能不能走這條捷徑用第一張影格判斷一次就好，相機解析度不會中途改變。
        """
        now = self._should_process_frame()
        if now is None:
            return

        try:
            if self._passthrough is None:
                self._passthrough = self._decide_passthrough(msg)

            if self._passthrough:
                payload = bytes(msg.data)
            else:
                frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    self._node.get_logger().warn("壓縮影像解碼失敗", throttle_duration_sec=5.0)
                    return
                payload = self._encode_frame(frame)
        except Exception as e:
            self._node.get_logger().warn(f"壓縮影像處理失敗: {e}", throttle_duration_sec=5.0)
            return

        if payload is not None:
            self._publish_frame(payload, now)

    def _decide_passthrough(self, msg: CompressedImage) -> bool:
        """用第一張影格決定壓縮影像能否直接轉發

        兩個條件都要成立：
          ・格式是 JPEG——MJPEG 串流每個分段都宣告 image/jpeg，塞 PNG 進去瀏覽器不吃
          ・尺寸已經等於 video_width（或設定為不縮放）
        """
        logger = self._node.get_logger()
        fmt = (msg.format or "").lower()
        if "jpeg" not in fmt and "jpg" not in fmt:
            logger.info(f"壓縮格式為「{msg.format}」非 JPEG，將解碼後重新編碼")
            return False

        if not self.video_width:
            logger.info("壓縮影像直接轉發（video_width=0，不縮放）——省下解碼與重新編碼")
            return True

        frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            logger.warn("壓縮影像解碼失敗，改走解碼路徑")
            return False

        if frame.shape[1] == self.video_width:
            logger.info(f"壓縮影像直接轉發（來源已是 {self.video_width}px 寬）——省下解碼與重新編碼")
            return True

        logger.info(
            f"來源寬度 {frame.shape[1]}px 與 video_width={self.video_width} 不同，需解碼縮放。"
            f"想走零解碼捷徑可設 video_width:=0"
        )
        return False

    # ── 訂閱按需開關 ──────────────────────────────────────

    def acquire(self) -> None:
        if self._image_sub is not None:
            return
        node = self._node
        if self.image_compressed:
            self._image_sub = node.create_subscription(
                CompressedImage,
                self.image_topic,
                self.compressed_image_cb,
                self._sensor_qos,
                callback_group=node._cb,
            )
        else:
            self._image_sub = node.create_subscription(
                Image,
                self.image_topic,
                self.image_cb,
                self._sensor_qos,
                callback_group=node._cb,
            )

    def release(self) -> None:
        if self._image_sub is None:
            return
        self._node.destroy_subscription(self._image_sub)
        self._image_sub = None
        # ★ 一定要把快取的影格丟掉。留著的話 /api/frame.jpg 會回一張
        #   不知道多久以前的照片 —— 人臉註冊拿它當樣本就會註冊到錯的人。
        with self._frame_lock:
            self._latest_jpeg = None
        self._frame_times = []
        self.state.set_system(camera_fps=0.0)

    # ── MJPEG ─────────────────────────────────────────────

    async def mjpeg_generator(self, request=None):
        """產生 multipart MJPEG 串流

        相機關掉時（迎賓機目前的常態）原本照樣以 video_fps=12 一直重送同一張
        「NO CAMERA SIGNAL」佔位圖——每秒 12 次穿過 starlette 的 chunked 編碼，
        內容還完全一樣。這裡改成沒有真影格時降到 1 fps：畫面上看不出差別
        （本來就是靜態圖），HTTP 執行緒的工作量降到十二分之一。

        ★ 串流的存在本身現在是一個訊號——它開著，影像訂閱才會建立
          （見 hmi_server_node._client_tick）。所以進出都要記帳，而且要用
          try/finally，不然對方一斷線就永遠漏掉一次減量，計數只增不減。
          乾淨的斷線不必自己偵測：starlette 的 StreamingResponse 內部就有一條
          listen_for_disconnect，收到 http.disconnect 會把產生器取消掉，
          finally 照樣會執行。
        """
        interval = 1.0 / max(self.video_fps, 1.0)
        placeholder = placeholder_jpeg()
        with self._client_lock:
            self._video_viewers += 1
        try:
            while True:
                with self._frame_lock:
                    frame = self._latest_jpeg
                payload = frame if frame is not None else placeholder
                yield b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                yield str(len(payload)).encode()
                yield b"\r\n\r\n"
                yield payload
                yield b"\r\n"
                await asyncio.sleep(interval if frame is not None else self.PLACEHOLDER_INTERVAL_SEC)
        finally:
            with self._client_lock:
                self._video_viewers = max(0, self._video_viewers - 1)
