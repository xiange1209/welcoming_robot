"""佔據柵格 -> PNG 渲染。

從 hmi_server_node.py 搬出來：_map_cb 只存最新一則未渲染的 OccupancyGrid，
真正的編碼交給 render_if_needed()（由 PoseTracker 的位姿計時器帶動，
只在有人在看地圖時才做）。
"""

import threading
import time
from typing import Optional

import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid

from .geometry import quaternion_to_yaw


class MapRenderer:
    def __init__(self, node, state, map_render_interval: float):
        self._node = node
        self.state = state
        self.map_render_interval = map_render_interval

        self._map_lock = threading.Lock()
        self._map_png: Optional[bytes] = None
        self._map_version = 0
        self._last_map_render = 0.0
        # 最新一則還沒渲染的 OccupancyGrid。/map 的回呼只把訊息「放這裡」，
        # 真正的 PNG 編碼延到 render_if_needed()（有人在看地圖時才做）。
        self._pending_grid: Optional[OccupancyGrid] = None

    def latest_png(self) -> Optional[bytes]:
        with self._map_lock:
            return self._map_png

    def on_grid(self, msg: OccupancyGrid) -> None:
        """收下最新的佔據柵格，但**不在這裡渲染**

        原本這支直接做 numpy 轉換 + cv2.imencode，等於在 ROS 執行緒上同步跑數十
        毫秒的影像編碼。/map 是 latched 話題，重點是「最後一則」而不是每一則，
        所以這裡只留參考（不複製，成本接近零），實際渲染交給 render_if_needed()：
        有人在看地圖時才做，而且照樣受 map_render_interval 節流。

        這樣建圖模式下 slam_toolbox 每 3 秒重發整張地圖也不再有 CPU 尖峰。
        """
        self._pending_grid = msg

    def render_if_needed(self) -> None:
        """有人在看地圖時，把最新的柵格渲染成 PNG

        由 PoseTracker 的位姿計時器呼叫（跟位姿同一個節拍），沒有新地圖或
        還沒到節流間隔就直接返回，正常情況下是一個 if 就結束。
        """
        msg = self._pending_grid
        if msg is None:
            return

        now = time.monotonic()
        if now - self._last_map_render < self.map_render_interval:
            return
        self._last_map_render = now

        try:
            info = msg.info
            width, height = int(info.width), int(info.height)
            if width <= 0 or height <= 0:
                return

            grid = np.asarray(msg.data, dtype=np.int8).reshape(height, width)

            # -1（未知）→ 中灰；0~100（佔據機率）→ 白到黑的連續灰階
            img = np.full((height, width), 127, dtype=np.uint8)
            known = grid >= 0
            img[known] = (255 - grid[known].astype(np.int16) * 255 // 100).astype(np.uint8)

            # ROS 柵格的 row 0 在地圖「下方」，影像的 row 0 在上方，所以要上下翻轉
            img = np.flipud(img)

            ok, buf = cv2.imencode(".png", img)
            if not ok:
                return

            with self._map_lock:
                self._map_png = buf.tobytes()
                self._map_version += 1
                version = self._map_version

            origin = info.origin
            self.state.set_map_meta(
                {
                    "width": width,
                    "height": height,
                    "resolution": float(info.resolution),
                    "origin_x": float(origin.position.x),
                    "origin_y": float(origin.position.y),
                    "origin_yaw": quaternion_to_yaw(
                        origin.orientation.x, origin.orientation.y, origin.orientation.z, origin.orientation.w
                    ),
                    "version": version,
                }
            )
            # 渲染成功才丟掉來源。/map 是 latched 話題，map_server 載入後可能
            # 一整天都不會再發第二則——中途失敗就把它清掉的話，這張地圖
            # 就永遠畫不出來了。留著，下一個間隔會再試一次。
            self._pending_grid = None
        except Exception as e:
            self._node.get_logger().warn(f"地圖渲染失敗: {e}", throttle_duration_sec=10.0)
