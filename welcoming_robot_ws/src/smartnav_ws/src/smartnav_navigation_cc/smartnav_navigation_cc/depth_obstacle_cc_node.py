"""深度圖 -> 稀疏障礙點雲 (_cc)

## 為什麼不直接用驅動的點雲

`astra_camera` 的 `enable_point_cloud:=true` 會把每一張深度圖轉成完整的
XYZ 點雲再發佈。實測在 Pi4 上：

    深度 320x240 + 點雲   astra_camera_node  61.8%   load 30.1
    深度 320x240 不含點雲  astra_camera_node  28.2%   load 22.6

**光是點雲生成就吃掉 34 個百分點的 CPU**，而且完全省不掉 ——
Astra S 的深度硬體最低就是 30 fps（driver 收到不支援的 fps 會靜默回退：
log 裡的 "Video mode 320x240@6Hz ... not supported / Default video mode 320x240@30Hz"
就是這麼來的），所以驅動每秒固定要轉 30 次 76800 點 = 27 MB/s。

避障根本不需要這個量。車速只有 0.25 m/s，5 Hz 更新時每幀之間才走 5 公分；
而障礙物「在不在」用降採樣後的點判斷就夠了，不需要每個像素。

## 這支節點做什麼

訂閱驅動的**深度圖**（150 KB/幀，比點雲便宜 6 倍），然後：

    降頻    每秒只處理 target_hz 幀（預設 5），其餘直接丟棄
    降採樣  每 pixel_step 個像素取一點（預設 4 -> 80x60 = 4800 個候選點）
    距離濾  只留 [min_range, max_range] 內的點，深度 0（無效值）丟掉

輸出 `/camera/obstacle_points`，資料量約為驅動點雲的 1/100。

高度過濾**不在這裡做** —— costmap 的 `min_obstacle_height` /
`max_obstacle_height` 已經在做了，而且它是 C++ 又能正確用 TF 轉到
全域座標，比在這裡用相機外參硬算可靠。這支只負責「少送一點資料」。

## 座標轉換

深度圖是 optical frame 慣例：X 向右、Y 向下、Z 向前。
反投影用針孔模型，內參從 camera_info 拿：

    z = depth
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

發佈時 frame_id 沿用深度圖自己的 header.frame_id
（`camera_depth_optical_frame`），下游自己去查 TF。
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


class DepthObstacleCC(Node):
    def __init__(self):
        super().__init__("depth_obstacle_cc")

        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("info_topic", "/camera/depth/camera_info")
        self.declare_parameter("output_topic", "/camera/obstacle_points")
        # 5 Hz：0.25 m/s 車速下每幀間隔 5 公分，避障足夠
        self.declare_parameter("target_hz", 5.0)
        # 每 4 個像素取一點 -> 320x240 變成 80x60
        self.declare_parameter("pixel_step", 4)
        # Astra S 近距離量不準（<0.4 m 常回傳 0 或雜訊）
        self.declare_parameter("min_range", 0.25)
        # 超過 2.5 m 的障礙交給雷達就好，相機的深度誤差在遠處會放大
        self.declare_parameter("max_range", 2.5)

        self.depth_topic = self.get_parameter("depth_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.target_hz = float(self.get_parameter("target_hz").value)
        self.step = int(self.get_parameter("pixel_step").value)
        self.min_range = float(self.get_parameter("min_range").value)
        self.max_range = float(self.get_parameter("max_range").value)

        self.fx = self.fy = self.cx = self.cy = None
        # 反投影用的像素網格，第一次拿到影像時才算得出來（要知道尺寸），
        # 之後尺寸不變就一直重用 —— 這是每幀省下來的主要成本。
        self._grid = None
        self._grid_shape = None
        self._last_emit = 0.0

        self.pub = self.create_publisher(PointCloud2, self.output_topic, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, self.get_parameter("info_topic").value, self.on_info, qos_profile_sensor_data
        )
        self.create_subscription(Image, self.depth_topic, self.on_depth, qos_profile_sensor_data)

        self._n_in = 0
        self._n_out = 0
        self.create_timer(10.0, self._report)

        self.get_logger().info(
            f"深度障礙點雲：{self.depth_topic} -> {self.output_topic}  "
            f"降頻 {self.target_hz} Hz、每 {self.step} 像素取一點、"
            f"距離 {self.min_range}~{self.max_range} m"
        )

    def on_info(self, msg: CameraInfo):
        if self.fx is None:
            self.fx, self.fy = msg.k[0], msg.k[4]
            self.cx, self.cy = msg.k[2], msg.k[5]
            self.get_logger().info(
                f"取得內參 fx={self.fx:.1f} fy={self.fy:.1f} cx={self.cx:.1f} cy={self.cy:.1f}"
            )

    def _ensure_grid(self, h, w):
        """建立降採樣後的像素座標網格（只算一次）。"""
        if self._grid_shape == (h, w):
            return
        vs = np.arange(0, h, self.step, dtype=np.float32)
        us = np.arange(0, w, self.step, dtype=np.float32)
        uu, vv = np.meshgrid(us, vs)
        # 先把 (u-cx)/fx 這種與深度無關的部分預先算好，
        # 每幀就只剩一次乘法。
        self._grid = (
            ((uu - self.cx) / self.fx).ravel(),
            ((vv - self.cy) / self.fy).ravel(),
        )
        self._grid_shape = (h, w)

    def on_depth(self, msg: Image):
        self._n_in += 1
        if self.fx is None:
            return

        # --- 降頻：還沒到下一幀的時間就直接丟掉 ---
        # 用訊息自己的時間戳而不是系統時間，這樣即使處理塞車也不會累積延遲。
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if now - self._last_emit < 1.0 / self.target_hz:
            return
        self._last_emit = now

        h, w = msg.height, msg.width
        self._ensure_grid(h, w)

        # --- 解析深度圖並降採樣 ---
        if msg.encoding in ("16UC1", "mono16"):
            depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
            z = depth[:: self.step, :: self.step].astype(np.float32) * 0.001  # mm -> m
        elif msg.encoding == "32FC1":
            depth = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w)
            z = depth[:: self.step, :: self.step].copy()
        else:
            self.get_logger().warn(f"不認得的深度編碼 {msg.encoding}", throttle_duration_sec=10.0)
            return
        z = z.ravel()

        # --- 距離過濾（深度 0 代表無效測量，會被 min_range 一起濾掉）---
        valid = (z >= self.min_range) & (z <= self.max_range) & np.isfinite(z)
        if not valid.any():
            return
        z = z[valid]
        x = self._grid[0][valid] * z
        y = self._grid[1][valid] * z

        cloud = np.empty((z.size, 3), dtype=np.float32)
        cloud[:, 0] = x
        cloud[:, 1] = y
        cloud[:, 2] = z

        self.pub.publish(self._to_msg(cloud, msg.header))
        self._n_out += 1

    def _to_msg(self, pts: np.ndarray, header) -> PointCloud2:
        out = PointCloud2()
        out.header = header
        out.height = 1
        out.width = pts.shape[0]
        out.is_dense = True
        out.is_bigendian = False
        out.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        out.point_step = 12
        out.row_step = 12 * pts.shape[0]
        out.data = pts.tobytes()
        return out

    def _report(self):
        self.get_logger().info(
            f"深度圖進 {self._n_in} 幀 / 點雲出 {self._n_out} 幀（{self.target_hz} Hz 目標）"
        )
        self._n_in = 0
        self._n_out = 0


def main(args=None):
    rclpy.init(args=args)
    node = DepthObstacleCC()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
