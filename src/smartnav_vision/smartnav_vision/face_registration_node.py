#!/usr/bin/env python3
"""人臉註冊 ROS 2 節點

提供人臉註冊服務，允許新增已知人員到識別系統
"""

from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rcl_interfaces.msg import ParameterDescriptor
from cv_bridge import CvBridge
from std_srvs.srv import Empty

from smartnav_vision.face_engine import FaceEngine
from smartnav_vision.database_manager import DatabaseManager
from smartnav_msgs.srv import RegisterFace


class FaceRegistrationNode(Node):
    """人臉註冊 ROS 2 節點

    訂閱相機影像並提供人臉註冊服務

    Subscriptions:
        image_topic (sensor_msgs/Image): 相機幀用於採集臉部樣本

    Services:
        /face_registration/register (RegisterFace): 註冊新人員
    """

    def __init__(self) -> None:
        """初始化人臉註冊節點

        宣告所有必要參數，初始化臉部引擎與資料庫管理器，
        建立相機影像訂閱與註冊服務
        """
        super().__init__("face_registration_node")

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
            "samples_per_person",
            5,
            ParameterDescriptor(description="每個人員的採集樣本數"),
        )

        # 讀取與驗證參數
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        model_name = self.get_parameter("model_name").get_parameter_value().string_value
        confidence_threshold = self.get_parameter("confidence_threshold").get_parameter_value().double_value
        enable_gpu = self.get_parameter("enable_gpu").get_parameter_value().bool_value
        self.samples_per_person = self.get_parameter("samples_per_person").get_parameter_value().integer_value

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

        self.bridge = CvBridge()

        # 記錄當前的註冊資訊
        self.current_registration = None

        # 訂閱相機影像
        self.image_subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile=10,
        )

        # 建立註冊服務
        self.register_service = self.create_service(
            RegisterFace,
            "/face_registration/register",
            self.register_callback,
        )

        # 建立快取刷新服務客戶端
        self.refresh_cache_client = self.create_client(Empty, "/face_recognition/refresh_cache")

        self.get_logger().info("✓ 人臉註冊節點已初始化")
        self.get_logger().info(f"  訂閱主題: {image_topic}")
        self.get_logger().info(f"  模型: {model_name}")
        self.get_logger().info(f"  資料庫: {self.db_manager.database_dir}")
        self.get_logger().info(f"  每人樣本數: {self.samples_per_person}")

    def image_callback(self, msg: Image) -> None:
        """處理相機幀用於人臉採集

        自動為任何進行中的註冊流程採集臉部樣本

        Args:
            msg: 來自相機的 ROS 2 影像訊息
        """
        # 只有在有進行中的註冊流程時才處理影像
        if self.current_registration is None:
            return

        try:
            # 轉換為 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # 檢測與提取
            faces = self.engine.detect_and_extract(cv_image)

            if len(faces) != 1:
                self.get_logger().warning("未偵測到或偵測到多張人臉，停止採集")
                return

            reg = self.current_registration

            if reg["collected_samples"] < reg["target_samples"]:
                # 添加到資料庫
                success = self.db_manager.add_face_sample(reg["person_uuid"], cv_image, faces[0].embedding)

                if success:
                    reg["collected_samples"] += 1
                    self.get_logger().info(
                        f"✓ {reg['person_name']}: 已採集 "
                        f"{reg['collected_samples']}/{reg['target_samples']} "
                        "個樣本"
                    )

                    # 檢查是否完成
                    if reg["collected_samples"] >= reg["target_samples"]:
                        self.current_registration = None
                        self.send_refresh_request()
                        self.get_logger().info(f"✓ {reg['person_name']} 註冊完成！")
                else:
                    self.get_logger().warning(f"樣本添加失敗: {reg['person_uuid']}")

        except Exception as e:
            self.get_logger().error(f"影像處理錯誤: {e}")

    def register_callback(
        self, request: RegisterFace.Request, response: RegisterFace.Response
    ) -> RegisterFace.Response:
        """處理人臉註冊服務請求

        驗證請求，檢查重複註冊，並啟動人臉採集流程
        """
        if self.current_registration is not None:
            response.success = False
            response.message = f"系統忙碌中，正在採集 {self.current_registration['person_name']} 的樣本"
            return response

        try:
            person_name = request.person_name.strip()
            gender = request.gender.strip() if request.gender else "Other"
            person_type = request.person_type.strip() if request.person_type else "VIP"

            if not person_name:
                response.success = False
                response.message = "人員名稱不能為空"
                return response

            # 驗證 gender
            if gender not in ["M", "F", "Other"]:
                response.success = False
                response.message = "性別只能是 M/F/Other"
                return response

            # 驗證 person_type
            if person_type not in ["VIP", "BLACKLIST"]:
                response.success = False
                response.message = "人員類型只能是 VIP/BLACKLIST"
                return response

            # 檢查該人員是否已註冊過
            all_persons = self.db_manager.get_all_persons()
            existing_uuid = None
            for uuid, person_info in all_persons.items():
                if person_info.get("person_name") == person_name:
                    existing_uuid = uuid
                    break

            # 若已註冊過，刪除舊記錄
            if existing_uuid:
                self.get_logger().info(f"刪除舊記錄以重新註冊: {person_name}")
                self.db_manager.delete_person(existing_uuid)

            # 開始新的註冊流程
            num_samples = request.num_samples if request.num_samples > 0 else self.samples_per_person
            person_uuid = self.start_registration(person_name, gender, person_type, num_samples)

            if person_uuid:
                response.success = True
                response.person_uuid = person_uuid
                response.message = (
                    f"已開始採集 {person_name} ({gender}) [{person_type}] 的人臉樣本 (目標: {num_samples} 個)"
                )
                self.get_logger().info(f"✓ 服務請求：開始註冊 {person_name} ({gender}) [{person_type}] (目標: {num_samples} 個)")
            else:
                response.success = False
                response.message = "註冊流程啟動失敗"

        except Exception as e:
            response.success = False
            response.message = f"服務處理錯誤: {str(e)}"
            self.get_logger().error(f"register_callback 發生異常: {e}")

        return response

    def start_registration(
        self,
        person_name: str,
        gender: str = "Other",
        person_type: str = "VIP",
        num_samples: Optional[int] = None,
    ) -> Optional[str]:
        """開始新的人臉註冊流程

        在資料庫中註冊新人員，並初始化採集狀態

        Args:
            person_name: 人員名稱
            gender: 性別 (M/F/Other)
            person_type: 人員類型 (VIP/BLACKLIST)
            num_samples: 目標樣本數，若無則使用預設值

        Returns:
            str: 成功時返回 person_uuid，失敗時返回 None
        """
        try:
            # 在資料庫中註冊新人員
            person_uuid = self.db_manager.register_person(person_name, gender, person_type)

            # 若未指定樣本數，使用預設值
            target_samples = num_samples if num_samples and num_samples > 0 else self.samples_per_person

            # 初始化註冊狀態
            self.current_registration = {
                "person_uuid": person_uuid,
                "person_name": person_name,
                "gender": gender,
                "person_type": person_type,
                "collected_samples": 0,
                "target_samples": target_samples,
                "created_at": str(self.get_clock().now().to_msg().sec),
            }

            self.get_logger().info(
                f"✓ 開始採集 {person_name} ({gender}) [{person_type}] 的人臉樣本 (目標: {target_samples} 個)..."
            )
            return person_uuid

        except Exception as e:
            self.get_logger().error(f"註冊開始失敗: {e}")
            return None

    def send_refresh_request(self):
        if self.refresh_cache_client.service_is_ready():
            request = Empty.Request()
            self.refresh_cache_client.call_async(request)
            self.get_logger().info("已發送刷新快取請求至識別節點")
        else:
            self.get_logger().warning("識別節點服務不可用，無法刷新快取")

    def get_registration_info(self, person_uuid: str) -> Optional[dict]:
        """取得註冊流程的資訊

        Args:
            person_uuid: 人員 UUID

        Returns:
            dict: 資訊字典，若人員不存在返回 None
        """
        if self.current_registration and self.current_registration["person_uuid"] == person_uuid:
            return self.current_registration

        return None

    def cancel_registration(self, person_uuid: str) -> bool:
        """取消進行中的註冊流程

        從資料庫刪除已採集的樣本並移除註冊資訊

        Args:
            person_uuid: 人員 UUID

        Returns:
            bool: 成功時為 True，失敗時為 False
        """
        try:
            if self.current_registration and self.current_registration["person_uuid"] == person_uuid:
                # 從資料庫刪除已採集的樣本
                self.db_manager.delete_person(person_uuid)

                # 移除註冊資訊
                self.current_registration = None

                self.get_logger().info(f"✓ 註冊已取消: {person_uuid}")
                return True

            self.get_logger().warning(f"取消註冊失敗: UUID {person_uuid} 不匹配或無進行中的註冊流程")
            return False
        except Exception as e:
            self.get_logger().error(f"取消註冊失敗: {e}")
            return False

    def get_all_persons(self) -> list:
        """取得所有已完成註冊的人員

        Returns:
            list: 人員名稱清單
        """
        return self.db_manager.list_persons()

    def delete_person(self, person_uuid: str) -> bool:
        """刪除已註冊的人員

        Args:
            person_uuid: 人員 UUID

        Returns:
            bool: 成功時為 True，失敗時為 False
        """
        return self.db_manager.delete_person(person_uuid)

    def print_database_stats(self) -> None:
        """列印資料庫統計資訊"""
        self.db_manager.print_statistics()


def main(args=None):
    """人臉註冊節點進入點"""
    rclpy.init(args=args)

    try:
        node = FaceRegistrationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
