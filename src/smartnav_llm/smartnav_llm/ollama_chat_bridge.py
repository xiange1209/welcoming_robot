#!/usr/bin/env python3
"""ollama_ros_chat 橋接節點

讓 WHEELTEC 的 ollama_ros_chat（topic 模式，JSON 包裝）能作為 smartnav_llm 的
可選替代方案（llm_stack:=wheeltec）接進既有管線：

    /user_text（純文字）→ 包成 {"content": ...} → chat_message
    chat_response（JSON，可能分段串流）→ 依 is_done 聚合 → /llm_response、/speech_text

設計原則：不修改 vendor 原始碼（保留可升級性），格式轉換由本節點吸收。
注意：ollama_ros_chat 是單輪對話、無工具呼叫；需要導航工具時請用 llm_stack:=smartnav。
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class OllamaChatBridgeNode(Node):
    """/user_text ↔ ollama_ros_chat 的雙向格式橋接

    Subscriptions:
        user_text (std_msgs/String): 純文字輸入（人臉橋接與 ASR 的共用話題）
        chat_response (std_msgs/String): ollama_ros_chat 的 JSON 回覆（含 is_done）

    Publishers:
        chat_message (std_msgs/String): 給 ollama_ros_chat 的 JSON 輸入
        llm_response (std_msgs/String): 聚合後的完整回覆（相容 smartnav_llm 介面）
        speech_text (std_msgs/String): 給 TTS 的回覆文字（相容 smartnav_audio 介面）
    """

    def __init__(self) -> None:
        super().__init__("ollama_chat_bridge_node")

        self.user_text_sub = self.create_subscription(String, "user_text", self.user_text_callback, 10)
        self.chat_response_sub = self.create_subscription(String, "chat_response", self.chat_response_callback, 10)
        self.chat_message_pub = self.create_publisher(String, "chat_message", 10)
        self.llm_response_pub = self.create_publisher(String, "llm_response", 10)
        self.speech_text_pub = self.create_publisher(String, "speech_text", 10)

        self._buffer = []

        self.get_logger().info("✓ ollama_chat_bridge 已初始化（/user_text ↔ chat_message/chat_response）")

    def user_text_callback(self, msg: String) -> None:
        """純文字包成 ollama_ros_chat 期待的 JSON 格式"""
        payload = json.dumps({"content": msg.data}, ensure_ascii=False)
        self.chat_message_pub.publish(String(data=payload))

    def chat_response_callback(self, msg: String) -> None:
        """聚合（可能分段的）JSON 回覆，完成時轉發到 smartnav 介面"""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning(f"chat_response 非 JSON，忽略: {msg.data[:80]}")
            return

        self._buffer.append(data.get("content", ""))
        if data.get("is_done"):
            full = "".join(self._buffer).strip()
            self._buffer = []
            if full:
                self.llm_response_pub.publish(String(data=full))
                self.speech_text_pub.publish(String(data=full))


def main(args=None):
    """ollama_chat_bridge 節點進入點"""
    rclpy.init(args=args)
    node = OllamaChatBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
