#!/usr/bin/env python3
"""人臉辨識文字橋接節點

訂閱 /face_recognition/result（smartnav_msgs/RecognitionResult），
將辨識結果轉換成自然語言文字，發佈到 /user_text，
讓 smartnav_llm 的 LangChain Agent 能以現有的文字輸入管道接收人臉辨識事件。

設計原則：
- 沿用現行主架構（ROS2 topic），不新增額外的服務/動作介面。
- smartnav_llm 目前維持「文字輸入」的方式，本節點只負責把辨識事件轉成文字，
  不直接呼叫 smartnav_llm 的任何工具。
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String

from smartnav_msgs.msg import RecognitionResult


class RecognitionTextBridgeNode(Node):
    """人臉辨識結果轉文字橋接節點

    Subscriptions:
        /face_recognition/result (smartnav_msgs/RecognitionResult): 人臉辨識結果

    Publishers:
        /user_text (std_msgs/String): 轉換後的自然語言文字，供 smartnav_llm 訂閱
    """

    def __init__(self) -> None:
        super().__init__("recognition_text_bridge_node")

        self.declare_parameter(
            "result_topic",
            "/face_recognition/result",
            ParameterDescriptor(description="人臉辨識結果話題"),
        )
        self.declare_parameter(
            "user_text_topic",
            "/user_text",
            ParameterDescriptor(description="輸出給 smartnav_llm 的文字話題"),
        )

        result_topic = self.get_parameter("result_topic").get_parameter_value().string_value
        user_text_topic = self.get_parameter("user_text_topic").get_parameter_value().string_value

        self.result_subscription = self.create_subscription(
            RecognitionResult,
            result_topic,
            self.result_callback,
            10,
        )
        self.user_text_publisher = self.create_publisher(String, user_text_topic, 10)

        self.get_logger().info("✓ 人臉辨識文字橋接節點已初始化")
        self.get_logger().info(f"  訂閱話題: {result_topic}")
        self.get_logger().info(f"  發布話題: {user_text_topic}")

    def result_callback(self, msg: RecognitionResult) -> None:
        """將辨識結果轉換為文字並發佈

        Args:
            msg: 人臉辨識結果
        """
        text = self._build_text(msg)
        self.get_logger().info(f"📤 轉發辨識事件: {text}")
        self.user_text_publisher.publish(String(data=text))

    @staticmethod
    def _build_text(msg: RecognitionResult) -> str:
        """依身分類型組出自然語言文字

        Args:
            msg: 人臉辨識結果

        Returns:
            str: 自然語言文字
        """
        gender_map = {"M": "男性", "F": "女性"}
        gender_text = gender_map.get(msg.gender, "性別未知")

        if msg.person_type == "BLACKLIST":
            return (
                f"人臉辨識事件：偵測到黑名單人員 {msg.person_name}（{gender_text}），"
                f"信心度 {msg.confidence:.2f}。請立即通知工作人員注意安全。"
            )
        if msg.person_type == "VIP":
            return (
                f"人臉辨識事件：偵測到 VIP 客戶 {msg.person_name}（{gender_text}），"
                f"信心度 {msg.confidence:.2f}。客戶尚未開口，請主動迎賓問候。"
            )
        return "人臉辨識事件：偵測到一般訪客，尚未開口，請主動問候並詢問需要什麼協助。"


def main(args=None):
    """人臉辨識文字橋接節點進入點"""
    rclpy.init(args=args)
    node = RecognitionTextBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
