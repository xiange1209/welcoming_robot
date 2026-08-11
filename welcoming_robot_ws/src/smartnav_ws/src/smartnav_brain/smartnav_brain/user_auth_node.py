#!/usr/bin/env python3
"""使用者身份驗證節點"""

import cv2
import threading
import numpy as np
from typing import Optional, cast
from dataclasses import dataclass

import message_filters
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import CompressedImage

from smartnav_msgs.msg import FaceEmbedding, RegistrationProgress, UserIdentity, UserInfo
from smartnav_msgs.srv import (
    DeleteUser,
    ExtractFaceEmbedding,
    ListUsers,
    RegisterFace,
    RegisterFacePhoto,
    UpdateUser,
)
from smartnav_brain.user_manager import UserType, UserManager
from smartnav_brain.brain_utils import compute_similarity


@dataclass
class UserRegistrationInfo:
    """使用者註冊資訊資料類別"""

    user_uuid: str
    user_name: str
    target_samples: int = 5
    collected_samples: int = 0


class UserAuthNode(Node):
    """使用者身份驗證節點"""

    def __init__(self):
        """初始化使用者身份驗證節點"""
        super().__init__("user_auth_node")

        # 宣告參數
        self.declare_parameter(
            "face_embedding_topic",
            "face_embedding",
            ParameterDescriptor(description="人臉向量話題"),
        )
        self.declare_parameter(
            "image_capture_topic",
            "/camera/color/image_raw/compressed",
            ParameterDescriptor(description="相機影像話題"),
        )
        self.declare_parameter(
            "user_identity_topic",
            "user_identity",
            ParameterDescriptor(description="身份辨識結果話題"),
        )
        self.declare_parameter(
            "recognition_threshold",
            0.8,
            ParameterDescriptor(description="人臉辨識相似度閾值"),
        )
        self.declare_parameter(
            "identity_publish_interval",
            2.0,
            ParameterDescriptor(description="身份辨識結果發布的最小間隔秒數"),
        )

        # 讀取與驗證參數
        face_embedding_topic = self.get_parameter("face_embedding_topic").get_parameter_value().string_value
        image_capture_topic = self.get_parameter("image_capture_topic").get_parameter_value().string_value
        user_identity_topic = self.get_parameter("user_identity_topic").get_parameter_value().string_value
        self.recognition_threshold = self.get_parameter("recognition_threshold").get_parameter_value().double_value
        self.identity_publish_interval = (
            self.get_parameter("identity_publish_interval").get_parameter_value().double_value
        )

        # 初始化使用者管理器
        try:
            self.user_manager = UserManager()
        except Exception as e:
            self.get_logger().error(f"使用者管理器初始化失敗: {e}")
            raise

        # 建立註冊人臉服務
        self.register_face_service = self.create_service(
            RegisterFace,
            "register_face",
            self._register_face_callback,
        )

        # 使用者管理服務
        self.list_users_service = self.create_service(ListUsers, "list_users", self._list_users_callback)
        self.delete_user_service = self.create_service(DeleteUser, "delete_user", self._delete_user_callback)
        self.update_user_service = self.create_service(UpdateUser, "update_user", self._update_user_callback)

        # 照片註冊：服務回呼裡要再去呼叫 face_embedding_node 的抽特徵服務。
        # 這種巢狀呼叫在單執行緒 executor 下會死鎖，所以只把這兩個放進
        # Reentrant 群組（既有回呼維持預設互斥群組，行為完全不變）。
        photo_cb = ReentrantCallbackGroup()
        self.register_face_photo_service = self.create_service(
            RegisterFacePhoto,
            "register_face_photo",
            self._register_face_photo_callback,
            callback_group=photo_cb,
        )
        self.extract_embedding_client = self.create_client(
            ExtractFaceEmbedding,
            "extract_face_embedding",
            callback_group=photo_cb,
        )
        self.photo_lock = threading.Lock()

        # 發布註冊進度。register_face 只負責「開始」，樣本是背景累積的，
        # 沒有這個話題的話 HMI 只能輪詢 list_users 猜進度。
        self.registration_progress_pub = self.create_publisher(RegistrationProgress, "registration_progress", 10)

        # 發布身份辨識結果
        self.user_identity_pub = self.create_publisher(UserIdentity, user_identity_topic, 10)
        self._last_identity_key: Optional[str] = None
        self._last_identity_time: float = 0.0

        # 訂閱人臉向量話題
        self.face_embedding_sub = message_filters.Subscriber(
            self,
            FaceEmbedding,
            face_embedding_topic,
        )

        # 訂閱相機影像
        self.image_capture_sub = message_filters.Subscriber(
            self,
            CompressedImage,
            image_capture_topic,
        )

        self.face_image_sync = message_filters.ApproximateTimeSynchronizer(
            [self.face_embedding_sub, self.image_capture_sub],
            queue_size=10,
            slop=0.1,
        )
        self.face_image_sync.registerCallback(self._synced_face_image_callback)

        self.current_registration: Optional[UserRegistrationInfo] = None
        self.is_registering_face = False
        self.face_reg_timer = None
        self.face_lock = threading.Lock()

        self.get_logger().info("✓ 使用者身份驗證節點已初始化")
        self.get_logger().info(f"  訂閱話題: {face_embedding_topic}")
        self.get_logger().info(f"  訂閱話題: {image_capture_topic}")

    def _publish_progress(
        self,
        *,
        status: str,
        message: str,
        user_uuid: str = "",
        user_name: str = "",
        collected: int = 0,
        target: int = 0,
        active: bool = True,
    ) -> None:
        """發布註冊進度。呼叫端已持有 face_lock 也沒關係——publish 不會阻塞"""
        msg = RegistrationProgress()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.user_uuid = user_uuid
        msg.user_name = user_name
        msg.collected_samples = int(collected)
        msg.target_samples = int(target)
        msg.active = active
        msg.status = status
        msg.message = message
        self.registration_progress_pub.publish(msg)

    def _register_face_callback(self, request, response):
        """處理註冊人臉服務請求"""
        timestamp = self.get_clock().now().nanoseconds / 1e9

        with self.face_lock:
            if self.is_registering_face:
                response.success = False
                response.message = "已經在進行人臉註冊，請稍後再試"
                return response

            try:
                user_uuid = self.user_manager.register_user(
                    created_at=timestamp,
                    user_name=request.user_name,
                    user_type=UserType(request.user_type.type),
                    description=request.description,
                )

                self.current_registration = UserRegistrationInfo(
                    user_uuid=user_uuid,
                    user_name=request.user_name,
                    target_samples=request.num_samples,
                )
            except Exception as e:
                response.success = False
                response.message = f"使用者註冊失敗: {e}"
                return response

            self.is_registering_face = True

            if self.face_reg_timer:
                self.face_reg_timer.cancel()
            self.face_reg_timer = threading.Timer(20.0, self._face_registration_timeout)
            self.face_reg_timer.start()

            self._publish_progress(
                status="running",
                message="人臉註冊已開始，請看向攝影機",
                user_uuid=user_uuid,
                user_name=request.user_name,
                collected=0,
                target=request.num_samples,
            )

            response.success = True
            response.message = "人臉註冊已開始，請看向攝影機"
            return response

    def _list_users_callback(self, request, response):
        """列出所有已註冊使用者"""
        try:
            with self.face_lock:
                rows = self.user_manager.list_users()

            users = []
            for row in rows:
                info = UserInfo()
                info.user_uuid = row["user_uuid"]
                info.user_name = row["user_name"]
                info.user_type.type = UserType[row["user_type"]].value
                info.description = row["description"]
                info.created_at = row["created_at"]
                info.num_samples = int(row["num_samples"])
                users.append(info)

            response.users = users
            response.success = True
            response.message = f"查詢到 {len(users)} 位使用者"
        except Exception as e:
            response.users = []
            response.success = False
            response.message = f"查詢使用者列表失敗: {e}"
            self.get_logger().error(f"查詢使用者列表失敗: {e}")

        return response

    def _delete_user_callback(self, request, response):
        """刪除使用者及其所有人臉樣本"""
        try:
            with self.face_lock:
                if (
                    self.is_registering_face
                    and self.current_registration
                    and self.current_registration.user_uuid == request.user_uuid
                ):
                    response.success = False
                    response.message = "此使用者正在註冊中，請等註冊結束再刪除"
                    return response

                success = self.user_manager.delete_user(request.user_uuid)

            response.success = bool(success)
            response.message = "使用者已刪除" if success else "刪除失敗，UUID 可能不存在"
        except Exception as e:
            response.success = False
            response.message = f"刪除使用者失敗: {e}"
            self.get_logger().error(f"刪除使用者失敗: {e}")

        return response

    def _update_user_callback(self, request, response):
        """修改使用者資料（名稱／類型／描述），不動人臉樣本"""
        try:
            with self.face_lock:
                success = self.user_manager.update_user(
                    user_uuid=request.user_uuid,
                    user_name=request.user_name or None,
                    user_type=UserType(request.user_type.type),
                    description=request.description,
                )

            response.success = bool(success)
            response.message = "使用者資料已更新" if success else "更新失敗，UUID 不存在或名稱重複"
        except Exception as e:
            response.success = False
            response.message = f"更新使用者失敗: {e}"
            self.get_logger().error(f"更新使用者失敗: {e}")

        return response

    def _extract_embedding(self, photo: CompressedImage, timeout: float = 20.0):
        """呼叫 face_embedding_node 抽特徵

        用 call_async + Event 而不是 client.call()，後者在 executor 執行緒裡
        等自己的回應容易卡死。
        """
        request = ExtractFaceEmbedding.Request()
        request.image = photo
        future = self.extract_embedding_client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=timeout):
            future.cancel()
            raise TimeoutError("抽取特徵逾時")
        return future.result()

    def _register_face_photo_callback(self, request, response):
        """用現成照片註冊人臉

        與 register_face 不同，這支是同步的：照片當下就有，抽完特徵、寫完樣本
        才回應，使用者不必站在鏡頭前等採樣。
        """
        response.user_uuid = ""
        if not request.photos:
            response.success = False
            response.message = "沒有收到任何照片"
            return response

        if not self.photo_lock.acquire(blocking=False):
            response.success = False
            response.message = "已有照片註冊進行中，請稍後再試"
            return response

        try:
            with self.face_lock:
                if self.is_registering_face:
                    response.success = False
                    response.message = "正在進行即時採樣註冊，請等它結束"
                    return response

            if not self.extract_embedding_client.wait_for_service(timeout_sec=3.0):
                response.success = False
                response.message = "face_embedding_node 未就緒，無法分析照片"
                return response

            total = len(request.photos)
            self._publish_progress(
                status="running",
                message=f"正在分析 {total} 張照片…",
                user_name=request.user_name,
                collected=0,
                target=total,
            )

            # 先把所有照片都抽完特徵才建使用者——照片全部無效時就不會在
            # 資料庫留下一個沒有任何樣本的空使用者。
            samples = []
            rejected = 0
            for idx, photo in enumerate(request.photos, start=1):
                try:
                    res = self._extract_embedding(photo)
                except Exception as e:
                    self.get_logger().warning(f"第 {idx} 張照片分析失敗: {e}")
                    rejected += 1
                    continue

                if not res.success:
                    self.get_logger().info(f"第 {idx} 張照片略過: {res.message}")
                    rejected += 1
                else:
                    buf = np.frombuffer(photo.data, dtype=np.uint8)
                    cv_image = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                    if cv_image is None:
                        rejected += 1
                    else:
                        samples.append((np.array(res.embedding, dtype=np.float32), cv_image))

                self._publish_progress(
                    status="running",
                    message=f"分析照片　{idx}/{total} 張（採用 {len(samples)}）",
                    user_name=request.user_name,
                    collected=len(samples),
                    target=total,
                )

            if not samples:
                response.success = False
                response.message = f"{total} 張照片都沒有偵測到人臉，未建立使用者"
                self._publish_progress(
                    status="failed",
                    message=response.message,
                    user_name=request.user_name,
                    collected=0,
                    target=total,
                    active=False,
                )
                return response

            try:
                user_uuid = self.user_manager.register_user(
                    created_at=self.get_clock().now().nanoseconds / 1e9,
                    user_name=request.user_name,
                    user_type=UserType(request.user_type.type),
                    description=request.description,
                )
            except Exception as e:
                response.success = False
                response.message = f"建立使用者失敗: {e}"
                self._publish_progress(
                    status="failed",
                    message=response.message,
                    user_name=request.user_name,
                    active=False,
                )
                return response

            accepted = 0
            for embedding, image in samples:
                if self.user_manager.add_face_sample(user_uuid, embedding, image):
                    accepted += 1

            response.success = accepted > 0
            response.user_uuid = user_uuid
            response.accepted = accepted
            response.rejected = rejected
            response.message = f"「{request.user_name}」照片註冊完成，採用 {accepted} 張" + (
                f"，略過 {rejected} 張（未偵測到人臉）" if rejected else ""
            )
            self.get_logger().info(response.message)
            self._publish_progress(
                status="succeeded" if response.success else "failed",
                message=response.message,
                user_uuid=user_uuid,
                user_name=request.user_name,
                collected=accepted,
                target=total,
                active=False,
            )
            return response
        finally:
            self.photo_lock.release()

    def _face_registration_timeout(self):
        """處理人臉註冊超時"""
        need_delete_uuid = None
        info = None
        with self.face_lock:
            if self.is_registering_face:
                self.is_registering_face = False

                if self.current_registration and self.current_registration.user_uuid:
                    need_delete_uuid = self.current_registration.user_uuid

                info = self.current_registration
                self.current_registration = None

        if info is not None:
            self._publish_progress(
                status="timeout",
                message=(
                    f"採樣逾時，只收到 {info.collected_samples}/{info.target_samples} 張，" "已自動取消，請重新註冊"
                ),
                user_uuid=info.user_uuid,
                user_name=info.user_name,
                collected=info.collected_samples,
                target=info.target_samples,
                active=False,
            )

        if need_delete_uuid:
            try:
                self.user_manager.delete_user(need_delete_uuid)
            except Exception as e:
                self.get_logger().error(f"超時刪除使用者失敗: {e}")

        self.get_logger().warn("人臉註冊超時，請重新嘗試")

    def _synced_face_image_callback(self, face_msg: FaceEmbedding, image_msg: CompressedImage) -> None:
        """比對人臉向量訊息進行身份驗證"""
        face_embedding = np.array(face_msg.embedding, dtype=np.float32)
        # 註冊模式走的是另一條分支、不做比對，但下面一律會呼叫
        # _publish_identity，所以先給預設值，避免 UnboundLocalError
        similarity = 0.0

        with self.face_lock:
            if self.is_registering_face and self.current_registration:
                img_buf = np.frombuffer(image_msg.data, dtype=np.uint8)
                cv_image = cv2.imdecode(img_buf, cv2.IMREAD_COLOR)

                if cv_image is None:
                    self.get_logger().warning("影像解碼失敗（cv_image 為 None）")
                    return

                user_uuid = self.current_registration.user_uuid
                self.user_manager.add_face_sample(user_uuid, face_embedding, cv_image)
                self.current_registration.collected_samples += 1

                reg_name = self.current_registration.user_name
                collected = self.current_registration.collected_samples
                target = self.current_registration.target_samples

                if collected >= target:
                    if self.face_reg_timer:
                        self.face_reg_timer.cancel()

                    self.is_registering_face = False
                    self.get_logger().info(f"用戶 {reg_name} 人臉註冊成功")
                    self.current_registration = None
                    self._publish_progress(
                        status="succeeded",
                        message=f"「{reg_name}」註冊完成，共 {collected} 張樣本",
                        user_uuid=user_uuid,
                        user_name=reg_name,
                        collected=collected,
                        target=target,
                        active=False,
                    )
                    return

                self._publish_progress(
                    status="running",
                    message=f"請保持正對鏡頭　{collected}/{target} 張",
                    user_uuid=user_uuid,
                    user_name=reg_name,
                    collected=collected,
                    target=target,
                )

            user_uuid, similarity = self._process_face_recognition(face_embedding)
            user_info = self.user_manager.get_user_info(user_uuid) if user_uuid else None
            if user_info:
                self.get_logger().info(
                    f"身份驗證成功: {user_info['user_name']} (UUID: {user_uuid}, "
                    f"類型: {user_info['user_type']}, 相似度: {similarity:.3f}, "
                    f"描述: {user_info['description']}"
                )

        self._publish_identity(user_uuid, user_info, face_msg, similarity)

    def _publish_identity(self, user_uuid, user_info, face_msg: FaceEmbedding,
                          similarity: float = 0.0) -> None:
        """發布身份辨識結果"""
        key = user_uuid if user_info else "__unknown__"
        now = self.get_clock().now().nanoseconds / 1e9
        if key == self._last_identity_key and (now - self._last_identity_time) < self.identity_publish_interval:
            return
        self._last_identity_key = key
        self._last_identity_time = now

        msg = UserIdentity()
        msg.header = face_msg.header
        msg.bbox = face_msg.bbox
        if user_info:
            msg.user_uuid = user_uuid or ""
            msg.user_name = user_info["user_name"]
            msg.description = user_info.get("description", "")
            msg.user_type.type = UserType[user_info["user_type"]].value
            msg.recognized = True
        else:
            msg.user_uuid = ""
            msg.user_name = "Unknown"
            msg.description = ""
            msg.user_type.type = UserType.GUEST.value
            msg.recognized = False
        # ★ 2026-08-10 補上：這一行漏掉會讓 bank_reception 的 VIP／黑名單
        #   劇本永遠不觸發（它有一道 similarity < 0.5 就 return 的門檻），
        #   而且全程沒有任何錯誤訊息。E1/E2 要記的也正是這個欄位。
        msg.similarity = float(similarity)
        self.user_identity_pub.publish(msg)

    def _process_face_recognition(self, embedding: np.ndarray):
        """處理人臉識別邏輯。

        回傳 `(best_match_uuid, max_similarity)`。

        ★ 2026-08-10：原本只回 uuid，把 `max_similarity` 算完就丟掉。後果有兩層：

        1. `_publish_identity` 沒東西可填 -> `UserIdentity.similarity` 恆為 **0.0**
        2. `bank_reception_node:146` 對 VIP/BLACKLIST 有一道
           `similarity < min_confidence`(0.5) 就 return 的門檻
           -> **0.0 < 0.5 恆成立 -> VIP 迎賓與黑名單通報永遠不會觸發**

        整條路上沒有任何錯誤訊息：節點在跑、`/user_identity` 有發、
        `recognized` 也是 True，只有「劇本不動」。這是專題的主秀。

        ★ 未認出時也要回實際的最高分（不是 0），E2 誤認率分析要的
        就是「未註冊者拿到多少分」的分布。
        """
        all_embeddings = self.user_manager.get_all_embeddings()

        max_similarity = 0.0
        best_match_uuid = None

        for user_uuid, embeddings in all_embeddings.items():
            mean_embedding = cast(np.ndarray, np.mean(embeddings, axis=0, dtype=np.float32))
            similarity = compute_similarity(embedding, mean_embedding)

            if similarity > max_similarity:
                max_similarity = similarity
                best_match_uuid = user_uuid

        if max_similarity < self.recognition_threshold:
            return None, max_similarity

        return best_match_uuid, max_similarity


def main(args=None):
    """使用者身份驗證節點進入點"""
    rclpy.init(args=args)
    node = UserAuthNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
