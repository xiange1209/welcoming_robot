#!/usr/bin/env python3
"""人臉實時識別 ROS 2 節點

訂閱相機影像，執行臉部檢測與識別，發佈識別結果
"""

from typing import Optional
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from rcl_interfaces.msg import ParameterDescriptor
from cv_bridge import CvBridge
from std_srvs.srv import Empty

from smartnav_vision.face_engine import FaceEngine
from smartnav_vision.database_manager import DatabaseManager


class FaceRecognitionNode(Node):
    """人臉實時識別 ROS 2 節點

    訂閱相機影像，執行臉部檢測與識別，並將識別結果發佈

    Subscriptions:
        image_topic (sensor_msgs/Image): 相機影像幀

    Publishers:
        /face_recognition/debug_image (sensor_msgs/Image): 除錯影像 (可選)
    """

    def __init__(self) -> None:
        """初始化人臉識別節點

        宣告所有必要參數，初始化臉部引擎與資料庫管理器
        """
        super().__init__("face_recognition_node")

        # 宣告參數
        self.declare_parameter(
            "image_topic",
            "/image_raw",
            ParameterDescriptor(description="相機影像話題"),
        )
        self.declare_parameter(
            "model_name",
            "buffalo_sc",
            ParameterDescriptor(description="InsightFace 模型名稱"),
        )
        self.declare_parameter(
            "confidence_threshold",
            0.5,
            ParameterDescriptor(description="檢測信心閾值 (0.0-1.0)"),
        )
        self.declare_parameter(
            "enable_gpu",
            True,
            ParameterDescriptor(description="啟用 GPU 加速"),
        )
        self.declare_parameter(
            "recognition_threshold",
            0.6,
            ParameterDescriptor(description="認人相似度閾值"),
        )
        self.declare_parameter(
            "publish_debug_image",
            False,
            ParameterDescriptor(description="發佈除錯影像"),
        )

        # 讀取與驗證參數
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        model_name = self.get_parameter("model_name").get_parameter_value().string_value
        confidence_threshold = self.get_parameter("confidence_threshold").get_parameter_value().double_value
        enable_gpu = self.get_parameter("enable_gpu").get_parameter_value().bool_value
        self.recognition_threshold = self.get_parameter("recognition_threshold").get_parameter_value().double_value
        self.publish_debug_image = self.get_parameter("publish_debug_image").get_parameter_value().bool_value

        # 初始化臉部引擎
        try:
            self.engine = FaceEngine(
                model_name=model_name,
                confidence_threshold=confidence_threshold,
                enable_gpu=enable_gpu,
                logger=self.get_logger(),
            )
        except Exception as e:
            self.get_logger().error(f"臉部引擎初始化失敗: {e}")
            raise

        # 初始化資料庫管理器
        try:
            self.db_manager = DatabaseManager(logger=self.get_logger())
        except Exception as e:
            self.get_logger().error(f"資料庫管理器初始化失敗: {e}")
            raise

        # 初始化特徵向量快取 (避免頻繁載入)
        self.embedding_cache: dict = {}
        self._update_embedding_cache()

        self.bridge = CvBridge()

        # 建立 QoS 配置檔
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # 訂閱相機影像
        self.image_subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile=sensor_qos,
        )

        # 建立快取刷新服務
        self.refresh_service = self.create_service(
            Empty, "/face_recognition/refresh_cache", self.refresh_cache_callback
        )

        # 發佈除錯影像 (可選)
        if self.publish_debug_image:
            self.debug_image_publisher = self.create_publisher(
                Image,
                "/face_recognition/debug_image",
                qos_profile=sensor_qos,
            )

        self.get_logger().info("✓ 人臉識別節點已初始化")
        self.get_logger().info(f"  訂閱主題: {image_topic}")
        self.get_logger().info(f"  模型: {model_name}")
        self.get_logger().info(f"  資料庫: {self.db_manager.database_dir}")
        self.get_logger().info(f"  認人閾值: {self.recognition_threshold}")

    def _update_embedding_cache(self) -> None:
        """更新特徵向量快取

        從資料庫載入所有已註冊人員的特徵向量，
        並計算每個人員的平均特徵向量作為代表，
        同時快取性別和人員類型資訊
        """
        try:
            all_embeddings = self.db_manager.get_all_embeddings()
            self.embedding_cache = {}

            for person_uuid, embeddings in all_embeddings.items():
                person_info = self.db_manager.get_person_info(person_uuid)
                if person_info:
                    self.embedding_cache[person_uuid] = {
                        "person_name": person_info["person_name"],
                        "gender": person_info.get("gender", "Other"),
                        "person_type": person_info.get("person_type", "VIP"),
                        "embeddings": embeddings,
                        # 計算平均特徵向量作為人員代表
                        "mean_embedding": np.mean(embeddings, axis=0).astype(np.float32),
                    }

            num_persons = len(self.embedding_cache)
            vip_count = sum(1 for info in self.embedding_cache.values() if info["person_type"] == "VIP")
            blacklist_count = sum(1 for info in self.embedding_cache.values() if info["person_type"] == "BLACKLIST")

            self.get_logger().info(
                f"✓ 特徵向量快取已更新: {num_persons} 人 (VIP: {vip_count}, 黑名單: {blacklist_count})"
            )
        except Exception as e:
            self.get_logger().warning(f"特徵向量快取更新失敗: {e}")

    def image_callback(self, msg: Image) -> None:
        """處理相機幀並執行人臉識別

        對輸入影像進行臉部檢測、特徵提取與身份識別，
        可選地發佈除錯影像

        Args:
            msg: 來自相機的 ROS 2 影像訊息
        """
        try:
            # 轉換為 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # 檢測與提取
            faces = self.engine.detect_and_extract(cv_image)

            if len(faces) > 0:
                self.get_logger().debug(f"偵測到 {len(faces)} 張臉部")

                for idx, face in enumerate(faces):
                    # 執行身份識別（優先檢查黑名單）
                    person_uuid, person_type, gender, confidence = self._recognize_face(face.embedding)

                    if person_uuid:
                        person_name = self.embedding_cache[person_uuid]["person_name"]
                        face.person_name = person_name
                        face.confidence = confidence
                        face.person_type = person_type
                        face.gender = gender

                        type_icon = "🔴 黑名單" if person_type == "BLACKLIST" else "🟢 VIP"
                        self.get_logger().info(
                            f"臉部 {idx}: [{type_icon}] {person_name} ({gender}) "
                            f"(置信度: {confidence:.3f})"
                        )
                    else:
                        face.person_name = "Unknown"
                        face.confidence = confidence
                        face.person_type = "VISITOR"
                        face.gender = ""

                        self.get_logger().debug(f"臉部 {idx}: 訪客 (最高相似度: {confidence:.3f})")
            else:
                self.get_logger().debug("未偵測到臉部")

            # 發佈除錯影像
            if self.publish_debug_image and self.debug_image_publisher:
                debug_image = self.engine.draw_detections(cv_image, faces)
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
                debug_msg.header = msg.header
                self.debug_image_publisher.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f"影像處理錯誤: {e}")

    def _recognize_face(self, embedding: np.ndarray) -> tuple[Optional[str], str, str, float]:
        """識別臉部身份

        將輸入特徵向量與快取中所有已註冊人員的平均特徵向量比較。
        優先檢查黑名單（安全優先），再檢查VIP。
        返回人員資訊、身份類型、性別和相似度。

        Args:
            embedding: 臉部特徵向量

        Returns:
            tuple[Optional[str], str, str, float]:
            (person_uuid, person_type, gender, max_similarity)
            - person_uuid: 匹配的人員UUID (若無匹配返回None)
            - person_type: VIP/BLACKLIST/VISITOR
            - gender: 性別 (M/F/Other)
            - max_similarity: 最高相似度分數
        """
        if not self.embedding_cache:
            return None, "VISITOR", "", 0.0

        max_similarity = 0.0
        best_match_uuid = None
        best_match_person_type = None
        best_match_gender = None

        # ⚠️ 優先檢查黑名單（安全優先）
        for person_uuid, cache_info in self.embedding_cache.items():
            if cache_info["person_type"] != "BLACKLIST":
                continue

            mean_embedding = cache_info["mean_embedding"]
            similarity = self.engine.compute_similarity(embedding, mean_embedding)

            if similarity > max_similarity:
                max_similarity = similarity
                best_match_uuid = person_uuid
                best_match_person_type = "BLACKLIST"
                best_match_gender = cache_info["gender"]

        # 若黑名單有高相似度匹配（>閾值），立即返回
        if max_similarity >= self.recognition_threshold and best_match_person_type == "BLACKLIST":
            return best_match_uuid, best_match_person_type, best_match_gender, max_similarity

        # 重置相似度，檢查VIP
        max_similarity = 0.0
        best_match_uuid = None
        best_match_person_type = None
        best_match_gender = None

        for person_uuid, cache_info in self.embedding_cache.items():
            if cache_info["person_type"] != "VIP":
                continue

            mean_embedding = cache_info["mean_embedding"]
            similarity = self.engine.compute_similarity(embedding, mean_embedding)

            if similarity > max_similarity:
                max_similarity = similarity
                best_match_uuid = person_uuid
                best_match_person_type = "VIP"
                best_match_gender = cache_info["gender"]

        # 判定是否超過VIP閾值
        if max_similarity >= self.recognition_threshold:
            return best_match_uuid, best_match_person_type, best_match_gender, max_similarity
        else:
            return None, "VISITOR", "", max_similarity

    def refresh_cache_callback(self, request: Empty.Request, response: Empty.Response) -> Empty.Response:
        """刷新資料庫快取

        當有新人員註冊時呼叫此方法以更新快取
        """
        try:
            self._update_embedding_cache()
            self.get_logger().info("✓ 快取刷新成功")
        except Exception as e:
            self.get_logger().error(f"快取刷新失敗: {e}")

        return response


def main(args=None):
    """人臉識別節點進入點"""
    rclpy.init(args=args)

    try:
        node = FaceRecognitionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
