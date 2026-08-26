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
        # ★★ 2026-08-19：0.8 -> 0.70，這是本專案第一次用數據決定這個值 ★★
        #
        # 在此之前 0.8 沒有任何量測支持。實測（同一人、正面、buffalo_sc、
        # 相似度定義是 (cos+1)/2）：
        #
        #     距離     筆數   平均相似度   範圍
        #     50 cm     4     0.7247      0.7127~0.7484
        #     1 m       4     0.7603      0.7215~0.8636     <- 最佳距離
        #     1.5 m     6     0.7450      0.6557~0.8195
        #     2 m       6     0.6272      0.4434~0.6924     <- 已不可信
        #
        # 對照基準：同一張照片自己比 = 1.000、**兩個不同的人 = 0.515**、隨機向量 = 0.489。
        #
        # ★ 0.8 實質上等於關閉辨識：20 筆只過 2 筆（10%），**註冊的人在自己的
        #   註冊距離都認不出來**。0.75 -> 0.70 之間辨識率從 21% 跳到 93%
        #   —— 同一人的相似度密集落在 0.70~0.75，0.8 剛好壓在分布最密的地方。
        #
        # ★★ 那個 93% 是 **in-sample**（用挑門檻的同一批資料算的，必然偏高）。
        #   事後用**全新資料**在 1 m 重量：**6 筆過 4 筆 = 67%**。67% 才是誠實的數字。
        #   身份每 2 秒發一次，67% 表示兩三次內會認出來，實務可用。
        #
        # 為什麼不取更靈敏的 0.65（新資料 6/6 全過）：
        #   1. 距離「不同人 0.515」的餘裕從 0.185 縮到 0.135
        #   2. ★ 0.65 會讓 2 公尺的樣本放進來 3/6，而那個距離**最低值 0.4434
        #      比「不同人」的基準還低** —— 等於在特徵已經不可信的距離開始接受比對。
        #   使用者 2026-08-19 明確選擇保守。
        #
        # ⚠ **誤認率沒有量過**：資料庫當時只有一個人，0.515 那個基準只有一組樣本。
        #   有第二個人可以測時，要重新檢視這個值。
        self.declare_parameter(
            "recognition_threshold",
            0.70,
            ParameterDescriptor(description="人臉辨識相似度閾值（2026-08-19 由實測決定，見上方註解）"),
        )
        self.declare_parameter(
            "identity_publish_interval",
            2.0,
            ParameterDescriptor(description="身份辨識結果發布的最小間隔秒數"),
        )
        # ★★ 2026-08-19：人離開之後把身份清掉 ★★
        #
        # 使用者實測：「遮住鏡頭還是有信心值，或是原本辨識的 VIP 沒消掉」。
        #
        # 機制：`face_embedding_node` **只在偵測到臉時才發布**
        # （face_embedding_node.py:135 `if face is not None:`）。
        # 沒有臉 = 沒有訊息 = 這個節點永遠不會被叫醒 =
        # `/user_identity` 上最後一則永遠留著。平板顯示的是**上一個人**，
        # 而 `bank_reception` 也會以為那個 VIP 還站在前面。
        #
        # ★ 這是「沒有訊息」與「訊息說沒人」的差別 —— ROS 裡這兩件事完全不同，
        #   而下游沒辦法區分。所以要由這裡主動發一則「現在沒人」。
        #
        # 2.0 秒的理由：相機 15+ FPS，正常情況每 66 ms 就有一幀；
        # 2 秒等於連續 30 幀都沒臉，不會被偶發的偵測漏失誤觸發。
        self.declare_parameter(
            "identity_timeout_sec",
            2.0,
            ParameterDescriptor(description="超過這麼久沒收到人臉向量就發布『無人』身份；0 = 關閉"),
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

        # ★★ 2026-08-26：人臉代表向量快取（理由見 _get_prototypes）★★
        self._proto_cache = {}      # {uuid: 正規化後的平均向量}
        self._proto_key = None      # (uuid, 樣本數) 快照，變了才重建

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
        # 身份逾時（見 identity_timeout_sec 的長註解）
        self.identity_timeout = float(
            self.get_parameter("identity_timeout_sec").get_parameter_value().double_value)
        self._last_face_time: float = 0.0     # 最後一次收到人臉向量的時刻
        self._identity_cleared: bool = True   # 目前是不是已經處於「無人」狀態
        if self.identity_timeout > 0.0:
            # ★ 用獨立計時器而不是掛在影像回呼上：影像回呼在**沒有臉**的時候
            #   一樣會跑，但相機整個掉線時它就不跑了 —— 那正是最需要清掉身份的時候。
            self._identity_timer = self.create_timer(0.5, self._identity_timeout_tick)

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

        # ★★ 2026-08-14：queue_size 從寫死的 10 改成可調，預設放大到 60 ★★
        #
        # 這個佇列是**固定格數**，所以它涵蓋的時間長度 = 格數 ÷ 影像速率，
        # 而向量要等 InsightFace 在 CPU 上推論完（實測落後約 1.1 秒）才發得出來。
        # 兩者一比就知道會不會配得到對：
        #
        #     影像 6.4 Hz（相機開深度時）-> 10 格 = 1.56 秒 > 1.1 秒  配得到
        #     影像  25 Hz（只開彩色）    -> 10 格 = 0.40 秒 < 1.1 秒  **永遠配不到**
        #
        # 8/14 為了省 CPU 把深度關掉（load 51 -> 3.2、影像 6.4 -> 25 Hz），
        # 結果**把人臉辨識弄壞了**：/user_identity 完全靜默、註冊收 0/10 張、
        # 20 秒後 `_face_registration_timeout` 把剛建好的使用者刪掉。
        # 三個症狀沒有一個指向同步佇列，全程不報錯。
        #
        # ★ 教訓：固定格數的佇列不該跟一個會變的來源速率耦合。
        #   60 格在 25 Hz 下涵蓋 2.4 秒，比推論延遲多一倍餘裕；
        #   相機日後再變速也還有空間。記憶體代價是多存 50 張壓縮影像，可忽略。
        sync_queue = self.declare_parameter(
            "sync_queue_size",
            60,
            ParameterDescriptor(description="影像與人臉向量同步佇列格數（要 > 推論延遲 × 影像速率）"),
        ).get_parameter_value().integer_value

        self.face_image_sync = message_filters.ApproximateTimeSynchronizer(
            [self.face_embedding_sub, self.image_capture_sub],
            queue_size=sync_queue,
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

    def _identity_timeout_tick(self) -> None:
        """超過 identity_timeout 沒看到人臉 -> 發一則「現在沒人」。

        ★ 只在狀態**改變**時發一次（`_identity_cleared` 旗標），不是每 0.5 秒洗一則。
        ★ 清掉的身份 similarity 一定要是 0.0、recognized 一定要是 False ——
          使用者看到的「遮住鏡頭還是有信心值」就是因為舊訊息的 similarity 留在畫面上。
        """
        if self.identity_timeout <= 0.0 or self._identity_cleared:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if self._last_face_time <= 0.0 or (now - self._last_face_time) < self.identity_timeout:
            return

        msg = UserIdentity()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.user_uuid = ""
        msg.user_name = "Unknown"
        msg.description = ""
        msg.user_type.type = UserType.GUEST.value
        msg.recognized = False
        msg.similarity = 0.0
        msg.bbox = [0.0, 0.0, 0.0, 0.0]
        self.user_identity_pub.publish(msg)

        self._identity_cleared = True
        # 一併清掉去重狀態，否則同一個人再走回來時會被
        # `key == self._last_identity_key` 擋掉，迎賓詞不會再觸發。
        self._last_identity_key = None
        self.get_logger().info(
            f"👤 已連續 {self.identity_timeout:.1f} 秒沒偵測到人臉，身份清除為『無人』")

    def _publish_identity(self, user_uuid, user_info, face_msg: FaceEmbedding,
                          similarity: float = 0.0) -> None:
        """發布身份辨識結果"""
        key = user_uuid if user_info else "__unknown__"
        now = self.get_clock().now().nanoseconds / 1e9
        # 看到臉了 -> 記時間、解除「無人」狀態（讓下一次離開能再清一次）
        self._last_face_time = now
        self._identity_cleared = False
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

    def _get_prototypes(self) -> dict:
        """取得每位使用者的代表向量（各樣本 L2 正規化後平均），只在註冊表變動時重建。

        Returns:
            dict: {user_uuid: 代表向量}

        ★★ 2026-08-26：這裡原本每一幀都重算一次 ★★

        原本 `_process_face_recognition` 每收到一則 /face_embedding 就呼叫
        `get_all_embeddings()` -> 對每個樣本 `np.load()` -> 再 `np.mean()`。
        全檔沒有任何快取。實測（3 使用者 × 10 樣本，Windows SSD、page cache 已熱）：

            每幀 16.5 ms、檔案 I/O 450 次/秒 -> 15 FPS 下佔將近 1/4 顆核心

        而 Pi 4 只有 4 顆核心、導航跑起來已經 382~398%，SD 卡又比 SSD 慢，
        實際成本只會更高。成本還與**註冊人數成正比** —— E1/E2 實驗要註冊
        多位受測者時會更明顯。

        ★★ 同時修正平均方式：先正規化再平均 ★★

        `face_engine.py` 存的是 InsightFace 的**原始** embedding
        （不是 `normed_embedding`），而它的長度會隨臉的大小與亮度變動。
        直接算算術平均等於**讓亮而大的樣本主導這個人的代表臉**。
        改成各自 L2 正規化後再平均，每張樣本權重相同。

        ★ 注意 `compute_similarity`（brain_utils.py）本來就有正規化，
          所以「比對」那一步一直是對的。問題只在**平均之前**沒有先正規化。

        ★ 快取鍵是 (uuid, 樣本數)，所以註冊新人或加樣本會自動重建。
          ⚠ 樣本數不變但檔案內容被換掉時**不會**重建 —— 目前沒有這種操作，
            若日後加了「重新註冊覆蓋樣本」的功能，要把註冊表 mtime 也放進鍵。
        """
        key = tuple(sorted(
            (uuid, len(info.get("samples", [])))
            for uuid, info in self.user_manager.user_registry.items()))
        if key == self._proto_key:
            return self._proto_cache

        protos = {}
        for user_uuid, embeddings in self.user_manager.get_all_embeddings().items():
            norms = np.linalg.norm(embeddings, axis=1, keepdims=True)
            norms[norms < 1e-6] = 1.0          # 全零向量不要製造 NaN
            protos[user_uuid] = cast(
                np.ndarray,
                np.mean(embeddings / norms, axis=0, dtype=np.float32))

        self._proto_cache, self._proto_key = protos, key
        self.get_logger().info(f"人臉代表向量已重建：{len(protos)} 位使用者")
        return protos

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
        max_similarity = 0.0
        best_match_uuid = None

        # ★ 2026-08-26：改走快取。原本每一幀都把所有樣本從磁碟重讀
        #   並重算平均，實測 16.5 ms/幀。詳見 _get_prototypes 的 docstring。
        for user_uuid, mean_embedding in self._get_prototypes().items():
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
