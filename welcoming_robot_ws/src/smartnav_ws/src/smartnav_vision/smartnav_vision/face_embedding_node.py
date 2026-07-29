#!/usr/bin/env python3
"""人臉向量提取節點"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import CompressedImage

from smartnav_msgs.msg import FaceEmbedding
from smartnav_msgs.srv import ExtractFaceEmbedding
from smartnav_vision.face_engine import FaceEngine


class FaceEmbeddingNode(Node):
    """人臉向量提取節點"""

    def __init__(self) -> None:
        """初始化人臉向量提取節點"""
        super().__init__("face_embedding_node")

        # 宣告參數
        self.declare_parameter(
            "image_capture_topic",
            "/camera/color/image_raw",
            ParameterDescriptor(description="相機影像話題"),
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

        # 讀取與驗證參數
        image_capture_topic = self.get_parameter("image_capture_topic").get_parameter_value().string_value
        face_embedding_topic = self.get_parameter("face_embedding_topic").get_parameter_value().string_value
        model_name = self.get_parameter("model_name").get_parameter_value().string_value
        detect_threshold = self.get_parameter("detect_threshold").get_parameter_value().double_value
        enable_gpu = self.get_parameter("enable_gpu").get_parameter_value().bool_value

        # 初始化臉部引擎
        try:
            self.face_engine = FaceEngine(
                model_name=model_name,
                det_thresh=detect_threshold,
                enable_gpu=enable_gpu,
                logger=self.get_logger(),
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

        # 訂閱相機影像
        self.image_capture_sub = self.create_subscription(
            CompressedImage,
            image_capture_topic,
            self._image_capture_callback,
            qos_profile=image_qos,
        )

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

    def _image_capture_callback(self, msg: CompressedImage) -> None:
        """處理相機幀並執行人臉向量提取"""
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
                face_msg.bbox = face.bbox.tolist()
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
            response.bbox = face.bbox.tolist()
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
