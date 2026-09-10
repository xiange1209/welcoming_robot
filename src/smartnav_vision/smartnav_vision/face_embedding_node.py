#!/usr/bin/env python3
"""人臉向量提取節點"""

import cv2
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import CompressedImage

from std_msgs.msg import Bool
from smartnav_msgs.msg import FaceEmbedding
from smartnav_msgs.srv import ExtractFaceEmbedding
from smartnav_vision.face_engine import FaceEngine


class FaceEmbeddingNode(Node):
    """人臉向量提取節點"""

    def __init__(self) -> None:
        """初始化人臉向量提取節點"""
        super().__init__("face_embedding_node")

        # 宣告參數
        # ★ 2026-08-10 修正：預設值原本是 "/camera/color/image_raw"，
        #   但這個節點訂的型別是 **CompressedImage**，而相機在那條話題上
        #   發的是 sensor_msgs/Image（astra_camera 的 ob_camera_node.cpp
        #   直接 create_publisher<Image>，沒有走 image_transport）。
        #   型別對不上 -> 訂閱建立成功、節點正常跑、log 也印「訂閱話題: ...」，
        #   但**一則影像都收不到**，/face_embedding 永遠沒有輸出。
        #   整條人臉線靜默卡死，不報錯。
        #
        #   CompressedImage 是 wheeltec_camera.launch.py 的 image_transport
        #   republish 節點另外發到 ".../compressed" 的。
        #   而且 user_auth_node 的預設值本來就是 compressed 那條——兩個節點
        #   原本連話題都不在一起，同步器也不可能對上。
        self.declare_parameter(
            "image_capture_topic",
            "/camera/color/image_raw/compressed",
            ParameterDescriptor(description="相機影像話題（CompressedImage）"),
        )
        self.declare_parameter(
            "face_embedding_topic",
            "face_embedding",
            ParameterDescriptor(description="人臉向量話題"),
        )
        self.declare_parameter(
            "model_name",
            "buffalo_sc",
            ParameterDescriptor(description="InsightFace 模型名稱"),
        )
        self.declare_parameter(
            "detect_threshold",
            0.5,
            ParameterDescriptor(description="偵測信心閾值 (0.0-1.0)"),
        )
        self.declare_parameter(
            "enable_gpu",
            True,
            ParameterDescriptor(description="啟用 GPU 加速"),
        )
        # ★★ 2026-08-24：這兩個一定要明確給，見 face_engine._init_model ★★
        self.declare_parameter(
            "det_size",
            640,
            ParameterDescriptor(
                description="偵測輸入邊長。不給的話 InsightFace 會跑 "
                            "[(128,128),(640,640)] 串聯，等於每幀偵測兩次"),
        )
        # ★★ 2026-08-27：依導航狀態自動降頻 ★★
        #
        # 這個節點原本**沒有任何跳幀或降頻機制**：相機 30 Hz 進來，
        # 它就一直追著算。單幀 CPU time 實測 1.92 核心秒，等於它會把
        # 拿得到的核心全部吃光。2026-08-27 實測：
        #     導航線 only                    142.7% / 400
        #     導航 + 相機 + 人臉             **400.0%，idle 0.0%，load 35.7**
        #     （連 `ros2 node list` 都排不到 CPU，看起來像節點掛了）
        #
        # ★ 但機器人不需要每秒認 15 次人。車子以 0.15 m/s 移動，
        #   每 3~5 秒認一次綽綽有餘。降頻之後：
        #     每 1 秒一幀 -> 人臉約 192% -> 全機約 387%  ✗ 太緊
        #     每 2 秒一幀 -> 人臉約 96%  -> 全機約 291%  ✓ 採用
        #     每 3 秒一幀 -> 人臉約 64%  -> 全機約 259%  ✓
        #
        # 兩段式：待機時全速（迎賓要反應快），導航中降頻（把核心讓給控制迴圈）。
        # 預設 idle = 0.0（不限制）所以**沒有導航時行為完全不變**，不會有回歸。
        self.declare_parameter(
            "process_interval_sec",
            0.0,
            ParameterDescriptor(description="待機時兩幀之間最少間隔幾秒。0 = 不限制"),
        )
        self.declare_parameter(
            "process_interval_navigating_sec",
            2.0,
            ParameterDescriptor(
                description="導航進行中兩幀之間最少間隔幾秒（訂 /navigation_active）。"
                            "skip_while_navigating 為 true 時這個值用不到"),
        )
        # ★★ 2026-08-27：導航中完全不推論（使用者確認的需求）★★
        #
        # 使用者的原話：「導航中可能要開相機顯示在 HMI，但是可以不用辨識。」
        # 這個組合最划算 —— 貴的是**推論**不是相機：
        #     人臉推論   ~190%（單幀 CPU time 1.92 核心秒）  <- 關掉
        #     相機+republish ~52%                           <- 留著給 HMI
        #     HMI 編碼（6 fps）~22%                          <- 留著
        # 導航中的帳：142.7（導航）+ 52 + 22 + 0 = **約 217% / 400** ✓
        #
        # 而這也符合使用者 7/28 自己定的流程：
        #   「待機（相機開，人臉辨識）→ 認出貴賓 → 對話 → 導航
        #     → **導航時相機可待機** → 到達 → 恢復辨識」
        # 帶位途中身份已經確定，不需要重複辨識。
        #
        # ★ 設 false 會退回用 process_interval_navigating_sec 降頻 ——
        #   如果之後要求「黑名單在車子移動時出現也要抓到」，就改用那條路。
        self.declare_parameter(
            "skip_while_navigating",
            True,
            ParameterDescriptor(description="導航進行中完全不做人臉推論"),
        )
        self.declare_parameter(
            "cpu_cores",
            2,
            ParameterDescriptor(
                description="人臉推論最多用幾顆核心（0 = 不限）。"
                            "Pi 4 只有 4 顆，不限的話會搶走導航控制迴圈"),
        )

        # 讀取與驗證參數
        image_capture_topic = self.get_parameter("image_capture_topic").get_parameter_value().string_value
        face_embedding_topic = self.get_parameter("face_embedding_topic").get_parameter_value().string_value
        model_name = self.get_parameter("model_name").get_parameter_value().string_value
        detect_threshold = self.get_parameter("detect_threshold").get_parameter_value().double_value
        enable_gpu = self.get_parameter("enable_gpu").get_parameter_value().bool_value
        det_size = self.get_parameter("det_size").get_parameter_value().integer_value
        cpu_cores = self.get_parameter("cpu_cores").get_parameter_value().integer_value
        self._interval_idle = float(
            self.get_parameter("process_interval_sec").get_parameter_value().double_value)
        self._interval_nav = float(
            self.get_parameter("process_interval_navigating_sec")
            .get_parameter_value().double_value)
        self._skip_when_nav = bool(
            self.get_parameter("skip_while_navigating").get_parameter_value().bool_value)
        self._navigating = False
        self._last_process = 0.0
        self._n_done = 0
        self._n_skip = 0
        self._last_report = time.monotonic()

        # 初始化臉部引擎
        try:
            self.face_engine = FaceEngine(
                model_name=model_name,
                det_thresh=detect_threshold,
                enable_gpu=enable_gpu,
                logger=self.get_logger(),
                det_size=det_size,
                cpu_cores=cpu_cores,
            )
        except Exception as e:
            self.get_logger().error(f"臉部引擎初始化失敗: {e}")
            raise

        # QoS 設定
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # 訂閱相機影像（導航中會整個退訂，見 _set_image_sub）
        self._image_topic = image_capture_topic
        self._image_qos = image_qos
        self.image_capture_sub = None
        self._set_image_sub(True)

        # ★ 導航狀態：導航中就降頻，把核心讓給控制迴圈
        #
        # ★★ 2026-08-27 晚間修正 ①：QoS 要跟發布端對上 ★★
        #
        # 發布端（navigation_action_cc_node.py:217-221）是
        # **TRANSIENT_LOCAL + RELIABLE + depth=1**，也就是 latched ——
        # 它在啟動時發一則 False，之後只在狀態改變時才發。
        #
        # 原本這裡用 `..., 10)` 的簡寫，等於 **VOLATILE**。
        # 兩者「相容」（發布端提供的耐久性比訂閱端要求的高，連得上），
        # 但 **VOLATILE 訂閱端拿不到 latched 的那一則舊訊息**。
        #
        # 後果：導航進行中重啟這個節點（或它比導航節點晚起來），
        # 就**永遠收不到那則 True**，於是全速跑推論 ——
        # 正好是這次要修掉的 400% CPU 飽和。
        #
        # 改成 TRANSIENT_LOCAL 之後，晚加入的訂閱端會立刻收到最後一則狀態。
        self._nav_topic = "/navigation_active"
        self.create_subscription(
            Bool, self._nav_topic, self._nav_active_cb,
            QoSProfile(depth=1,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))

        # ★★ 2026-08-27 晚間修正 ②：_navigating 卡在 True 的看門狗 ★★
        #
        # `/navigation_active` 是 on-change 發布的，而 `_navigating` 只在
        # 那個回呼裡被改。所以**導航節點在 True 的狀態下掛掉／被 SIGKILL**，
        # 就再也不會有人發 False ——
        #     `_navigating` 永遠是 True
        #  -> 影像訂閱永遠不會被重建
        #  -> **人臉辨識靜默永久失效，而且不會有任何錯誤訊息**
        #
        # 這不是理論風險：`stop_nav_cc.sh` 走的就是 TERM 之後補 KILL，
        # 而 8/27 當天導航節點被重啟了六次。
        #
        # 判斷依據用「發布端還在不在」而不是逾時 ——
        # on-change 話題上「很久沒訊息」是正常的，不能當成異常；
        # 但**發布端消失**是明確的異常。
        self._nav_watchdog_timer = self.create_timer(5.0, self._nav_watchdog)

        # 發布人臉向量
        self.face_embedding_pub = self.create_publisher(
            FaceEmbedding,
            face_embedding_topic,
            10,
        )

        # 提供給其他節點的抽特徵服務。模型只有這個行程載了一份，照片註冊
        # 走這裡就不必在 user_auth_node 再載一次（Pi 上記憶體很吃緊）。
        # 這個節點用單執行緒 executor，服務與影像回呼自然序列化，
        # 不必替 FaceEngine 另外上鎖。
        self.extract_srv = self.create_service(
            ExtractFaceEmbedding,
            "extract_face_embedding",
            self._extract_callback,
        )

        self.get_logger().info("✓ 人臉向量提取節點已初始化")
        self.get_logger().info(f"  訂閱話題: {image_capture_topic}")
        self.get_logger().info(f"  發佈話題: {face_embedding_topic}")
        self.get_logger().info(f"  模型: {model_name}")
        self.get_logger().info(f"  偵測信心閾值: {detect_threshold}")

    def _set_image_sub(self, on: bool) -> None:
        """開關影像訂閱。

        ★★ 為什麼是「退訂」而不是「收到再丟棄」★★

        2026-08-27 實測，導航中只是在回呼裡 return（收到才丟）：
            全機 303.0% / 400，idle 24.2%
        還是比預期高很多。原因有兩層，兩層都在**回呼被呼叫之前**就付掉了：

        1. rclpy 對每一則都要在 Python 端把 CompressedImage 反序列化
           （640x480 JPEG 約 85 KB，30 Hz）。
        2. ★ `image_transport` 的 republish 是**惰性**的 ——
           8/24 實測：訂閱者 0 個時 republish 只有 **0.6%**，
           一有訂閱者就跳到 **36.4%**，astra_camera_node 再 +16%。

        ★★ 2026-08-27 更正：上面第 2 點**對本專案不成立**。
           `user_auth_node` 也一直訂著同一條影像話題（message_filters，全程不退訂），
           所以「訂閱者 0 個」這個狀態在本專案**永遠不會發生** ——
           republish 那 36.4% 一分都省不到。

           退訂真正省下來的是**這個節點自己的**反序列化與推論，
           而那仍然是主要項：實測 303.0%（收到才丟）→ **212.3%**（退訂），
           省了 **91 個百分點**。結論方向不變，但理由要說對。

        HMI 那邊仍然訂著，所以平板照樣有畫面。
        """
        if on and self.image_capture_sub is None:
            self.image_capture_sub = self.create_subscription(
                CompressedImage,
                self._image_topic,
                self._image_capture_callback,
                qos_profile=self._image_qos,
            )
        elif not on and self.image_capture_sub is not None:
            self.destroy_subscription(self.image_capture_sub)
            self.image_capture_sub = None

    def _nav_active_cb(self, msg: Bool) -> None:
        if msg.data != self._navigating:
            self._navigating = msg.data
            if msg.data and self._skip_when_nav:
                self._set_image_sub(False)
                self.get_logger().info(
                    "導航開始 -> **退訂影像、完全停止人臉推論**"
                    "（相機仍開著，平板照樣有畫面）")
            elif msg.data:
                self.get_logger().info(
                    f"導航開始 -> 人臉處理間隔 {self._interval_nav:.1f} 秒")
            else:
                self._set_image_sub(True)
                # ★ 不重置的話，導航結束後第一幀會立刻補印一則統計，
                #   而裡面的數字是導航前的殘值（會誤導）。
                self._last_report = time.monotonic()
                self._n_done = self._n_skip = 0
                iv = self._interval_idle
                self.get_logger().info(
                    f"導航結束 -> 重新訂閱影像、恢復辨識，間隔 {iv:.1f} 秒"
                    f"{'（不限制）' if iv <= 0 else ''}")

    def _nav_watchdog(self) -> None:
        """導航狀態的看門狗（5 秒一次）。

        兩件事：
        1. **發布端消失就恢復辨識** —— 見 __init__ 裡的長註解。
           導航節點在 True 的狀態下掛掉，沒有人會再發 False，
           人臉辨識就靜默永久失效。
        2. **導航中印心跳** —— 退訂之後影像回呼完全不會被呼叫，
           那段 30 秒統計掛在處理路徑上所以也印不出來，
           節點在 log 上看起來像掛了。展示現場很容易誤判。
        """
        if not self._navigating:
            return

        try:
            n_pub = self.count_publishers(self._nav_topic)
        except Exception:  # noqa: BLE001
            return                      # 查不到就當作正常，不要自己嚇自己

        if n_pub == 0:
            # ★ 發布端不見了，而我們還以為在導航中 -> 一定是它掛了
            self._navigating = False
            self._set_image_sub(True)
            self._last_report = time.monotonic()
            self._n_done = self._n_skip = 0
            self.get_logger().warning(
                "⚠ /navigation_active 的發布端消失，但狀態還停在「導航中」"
                "（導航節點可能被 KILL 了）—— 已自動恢復影像訂閱與人臉辨識。"
                "★ 若導航其實還在跑，CPU 會回到飽和，請重啟導航節點。")
            return

        # 心跳：導航中每 30 秒報一次還活著
        now = time.monotonic()
        if now - self._last_report >= 30.0:
            self._last_report = now
            if self._skip_when_nav:
                self.get_logger().info(
                    "（導航中：已退訂影像、暫停人臉推論，節點正常）")
            else:
                self.get_logger().info(
                    f"（導航中：降頻 {self._interval_nav:.1f} 秒一幀，"
                    f"已處理 {self._n_done}／跳過 {self._n_skip}）")

    def _image_capture_callback(self, msg: CompressedImage) -> None:
        """處理相機幀並執行人臉向量提取"""
        # ★★ 降頻要在**解碼之前**判斷 ★★
        #   cv2.imdecode 一張 640x480 JPEG 在 Pi 4 上就要好幾毫秒，
        #   30 Hz 下光解碼就是可觀的浪費。先跳過才有意義。
        if self._navigating and self._skip_when_nav:
            self._n_skip += 1
            return
        interval = self._interval_nav if self._navigating else self._interval_idle
        now = time.monotonic()
        if interval > 0.0 and (now - self._last_process) < interval:
            self._n_skip += 1
            return
        self._last_process = now
        self._n_done += 1
        if now - self._last_report >= 30.0:
            tot = self._n_done + self._n_skip
            self.get_logger().info(
                f"人臉處理 {self._n_done}/{tot} 幀"
                f"（跳過 {self._n_skip}），目前間隔 {interval:.1f} 秒")
            self._n_done = self._n_skip = 0
            self._last_report = now
        try:
            # 轉換為 OpenCV 格式
            img_buf = np.frombuffer(msg.data, dtype=np.uint8)
            cv_image = cv2.imdecode(img_buf, cv2.IMREAD_COLOR)

            if cv_image is None:
                self.get_logger().warning("影像解碼失敗（cv_image 為 None）")
                return

            # 偵測與提取
            face = self.face_engine.detect_and_extract(cv_image)

            if face is not None:
                # 建立並發佈人臉向量訊息
                face_msg = FaceEmbedding()
                face_msg.header = msg.header
                # 明確轉 float：msg 欄位是 float32[4]，rosidl 會 assert 型別
                face_msg.bbox = [float(v) for v in face.bbox]
                face_msg.embedding = face.embedding.tolist()
                self.face_embedding_pub.publish(face_msg)
        except Exception as e:
            self.get_logger().error(f"處理影像時發生錯誤: {e}")

    def _extract_callback(self, request, response):
        """從單張影像抽特徵，供照片註冊使用"""
        try:
            buf = np.frombuffer(request.image.data, dtype=np.uint8)
            cv_image = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            if cv_image is None:
                response.success = False
                response.message = "影像解碼失敗，請確認是有效的 JPEG/PNG"
                return response

            face = self.face_engine.detect_and_extract(cv_image)
            if face is None:
                response.success = False
                response.message = "照片中沒有偵測到人臉"
                return response

            response.success = True
            response.message = "ok"
            response.bbox = [float(v) for v in face.bbox]
            response.embedding = face.embedding.tolist()
        except Exception as e:
            self.get_logger().error(f"抽取特徵失敗: {e}")
            response.success = False
            response.message = f"抽取特徵失敗: {e}"
        return response


def main(args=None):
    """人臉向量提取節點進入點"""
    rclpy.init(args=args)
    node = FaceEmbeddingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
