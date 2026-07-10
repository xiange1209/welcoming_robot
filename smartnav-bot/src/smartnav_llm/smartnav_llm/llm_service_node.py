#!/usr/bin/env python3

"""LLM 對話服務節點

使用 Ollama 調用 Gemma4:e2b 模型，根據使用者輸入智能決定調用哪個服務
支援對話記憶、流式傳輸和服務自動化
"""

import re
import threading
from collections import deque
from typing import List, Any
from langchain_ollama import ChatOllama
from langchain_core.tools import tool
from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder
from langchain_core.callbacks import BaseCallbackHandler
from langchain_core.messages import BaseMessage, AIMessage, HumanMessage, ToolMessage
from langchain_core.runnables import RunnablePassthrough

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from action_msgs.msg import GoalStatus

from smartnav_msgs.srv import ListMaps, SwitchMap, CreateWaypoint, ListWaypoints
from smartnav_msgs.action import CreateMap, GlobalLocalization, Navigate
from smartnav_llm.llm_utils import get_config_path


class RosStreamHandler(BaseCallbackHandler):
    """自訂串流處理器：將 LLM 回應的 Token 即時發布到 ROS 2 Topic"""

    def __init__(self, stream_publisher, speech_text_publisher):
        super().__init__()
        self.stream_publisher = stream_publisher
        self.speech_text_publisher = speech_text_publisher
        self.split_pattern = re.compile(r"([,.\!?;:，。！？；：\n])")
        self.clean_pattern = re.compile(r"[^\w\u4e00-\u9fa5\s]|[_-]")
        self.buffer = ""
        self.current_sentence = ""

    def on_llm_new_token(self, token: str, **kwargs: Any) -> None:
        if not token:
            return

        self.stream_publisher.publish(String(data=token))

        self.buffer += token
        parts = self.split_pattern.split(self.buffer)
        self.buffer = parts.pop() if parts else ""

        for item in parts:
            if not item:
                continue

            if self.split_pattern.match(item):
                speech_sentence = " ".join(self.clean_pattern.sub("", self.current_sentence).split())
                if speech_sentence:
                    self.speech_text_publisher.publish(String(data=speech_sentence))
                self.current_sentence = ""
            else:
                self.current_sentence += item

    def on_llm_end(self, response: Any, **kwargs: Any) -> None:
        remaining_text = " ".join(self.clean_pattern.sub("", self.current_sentence + self.buffer).split())
        if remaining_text:
            self.speech_text_publisher.publish(String(data=remaining_text))
        self.buffer = ""
        self.current_sentence = ""


class ConversationMemory:
    """對話記憶管理類別"""

    def __init__(self, max_history: int = 2):
        # 記憶對話組數
        self.history: deque = deque(maxlen=max_history * 2)

    def add_message(self, message: BaseMessage) -> None:
        self.history.append(message)

    def get_history_messages(self) -> List[BaseMessage]:
        return list(self.history)


class LLMServiceNode(Node):
    """LLM 服務節點類別"""

    def __init__(self):
        super().__init__("llm_service_node")

        # 宣告與讀取參數
        self.ollama_base_url = (
            self.declare_parameter("ollama_base_url", "http://localhost:11434").get_parameter_value().string_value
        )
        self.model_name = self.declare_parameter("model_name", "gemma4:e2b").get_parameter_value().string_value
        self.temperature = self.declare_parameter("temperature", 0.0).get_parameter_value().double_value

        # 初始化對話記憶
        self.memory = ConversationMemory()
        self.memory_lock = threading.Lock()
        self.agent_busy_lock = threading.Lock()
        self.is_agent_running = False

        # 建立 Callback Group
        self.cb_group = ReentrantCallbackGroup()

        # 建立服務/動作/客戶端
        self.list_maps_client = self.create_client(ListMaps, "list_maps", callback_group=self.cb_group)
        self.switch_map_client = self.create_client(SwitchMap, "switch_map", callback_group=self.cb_group)
        self.create_waypoint_client = self.create_client(
            CreateWaypoint, "create_waypoint", callback_group=self.cb_group
        )
        self.list_waypoints_client = self.create_client(ListWaypoints, "list_waypoints", callback_group=self.cb_group)
        self.global_localization_client = ActionClient(
            self, GlobalLocalization, "global_localization", callback_group=self.cb_group
        )
        self.create_map_client = ActionClient(self, CreateMap, "create_map", callback_group=self.cb_group)
        self.navigate_client = ActionClient(self, Navigate, "navigate", callback_group=self.cb_group)

        # 建立發佈者和訂閱者
        self.llm_response_pub = self.create_publisher(String, "llm_response", 10)
        self.llm_stream_pub = self.create_publisher(String, "llm_stream", 10)
        self.speech_text_pub = self.create_publisher(String, "speech_text", 10)
        self.user_text_sub = self.create_subscription(
            String, "user_text", self.user_text_callback, 10, callback_group=self.cb_group
        )

        # 等待所有服務就緒
        self._init_timer = self.create_timer(0.1, self._init_callback, callback_group=self.cb_group)
        # 初始化工具鏈
        self._init_modern_llm_tools()

        self.get_logger().info(f"✓ LLM 對話服務節點已初始化")

    def _init_callback(self) -> None:
        """初始化回調函式"""
        self._init_timer.cancel()
        self._wait_for_services()

    def _wait_for_services(self) -> None:
        """等待所有基礎服務或動作就緒"""
        services = [
            ("list_maps", self.list_maps_client),
            ("create_waypoint", self.create_waypoint_client),
            ("list_waypoints", self.list_waypoints_client),
            ("switch_map", self.switch_map_client),
        ]
        actions = [
            ("create_map", self.create_map_client),
            ("global_localization", self.global_localization_client),
            ("navigate", self.navigate_client),
        ]
        for service_name, client in services:
            while not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(f"⌛ 等待服務 {service_name}...")
        for action_name, client in actions:
            while not client.wait_for_server(timeout_sec=2.0):
                self.get_logger().warn(f"⌛ 等待動作 {action_name}...")
        self.get_logger().info("✓ 所有基礎服務與動作已就緒")

    def _wait_for_future(self, future, timeout_sec: float) -> Any:
        """等待服務回應的輔助函式"""
        event = threading.Event()
        future.add_done_callback(lambda f: event.set())

        # 阻塞當前獨立執行緒，直到被喚醒或超時
        if not event.wait(timeout=timeout_sec):
            raise TimeoutError(f"服務請求超時")

        return future.result()

    def _init_modern_llm_tools(self) -> None:
        """定義工具對照表"""

        @tool
        def create_map_tool(map_name: str) -> str:
            """建立一張新地圖，並自動載入該地圖與座標，除非使用者要求建立地圖，否則不應該呼叫此工具"""
            goal = CreateMap.Goal()
            goal.map_name = map_name
            future = self.create_map_client.send_goal_async(goal)
            try:
                goal_handle = self._wait_for_future(future, timeout_sec=5.0)

                if not goal_handle.accepted:
                    return f"執行結果: 失敗, 詳細信息: 建立地圖請求被系統拒絕"

                result_future = goal_handle.get_result_async()
                action_result = self._wait_for_future(result_future, timeout_sec=500.0)

                id = action_result.result.map_info.map_id
                name = action_result.result.map_info.map_name

                if action_result.status == GoalStatus.STATUS_SUCCEEDED:
                    return f"執行結果: 成功, 地圖信息(包含地圖ID與地圖名稱): ({id}, {name}), 詳細信息: {action_result.result.message}"
                elif action_result.status == GoalStatus.STATUS_CANCELED:
                    return "執行結果: 取消, 詳細信息: 建立地圖請求被系統或使用者取消"
                elif action_result.status == GoalStatus.STATUS_ABORTED:
                    return f"執行結果: 失敗, 詳細信息: 建立地圖過程中發生錯誤"
                else:
                    return f"執行結果: 失敗, 詳細信息: {action_result.result.message}"
            except Exception as e:
                return f"執行結果: 失敗, 詳細信息: {str(e)}"

        @tool
        def list_maps_tool() -> str:
            """查詢現有的地圖列表信息"""
            req = ListMaps.Request()
            future = self.list_maps_client.call_async(req)
            try:
                res = self._wait_for_future(future, timeout_sec=5.0)
                if res.success:
                    ids = [wp.map_id for wp in res.maps_info]
                    names = [wp.map_name for wp in res.maps_info]
                    return f"執行結果: 成功, 地圖列表信息(包含地圖ID與地圖名稱): {list(zip(ids, names))}, 詳細信息: {res.message}"
                else:
                    return f"執行結果: 失敗, 詳細信息: {res.message}"
            except Exception as e:
                return f"執行結果: 失敗, 詳細信息: {str(e)}"

        @tool
        def switch_map_tool(map_id: str) -> str:
            """切換地圖，呼叫此工具時不應該使用地圖名稱"""
            req = SwitchMap.Request()
            req.map_id = map_id
            future = self.switch_map_client.call_async(req)
            try:
                res = self._wait_for_future(future, timeout_sec=5.0)
                if res.success:
                    return f"執行結果: 成功, 詳細信息: {res.message}"
                else:
                    return f"執行結果: 失敗, 詳細信息: {res.message}"
            except Exception as e:
                return f"執行結果: 失敗, 詳細信息: {str(e)}"

        @tool
        def create_waypoint_tool(waypoint_name: str) -> str:
            """在當前載入的地圖中建立一個地點，除非使用者要求建立地點，否則不應該呼叫此工具"""
            req = CreateWaypoint.Request()
            req.waypoint_name = waypoint_name
            future = self.create_waypoint_client.call_async(req)
            try:
                res = self._wait_for_future(future, timeout_sec=5.0)
                if res.success:
                    id = res.waypoint_info.waypoint_id
                    name = res.waypoint_info.waypoint_name
                    map_id = res.waypoint_info.map_id
                    x = res.waypoint_info.pose.position.x
                    y = res.waypoint_info.pose.position.y
                    return f"執行結果: 成功, 地點信息(包含地點ID、地點名稱、地圖ID、地點座標): ({id}, {name}, {map_id}, ({x}, {y})), 詳細信息: {res.message}"
                else:
                    return f"執行結果: 失敗, 詳細信息: {res.message}"
            except Exception as e:
                return f"執行結果: 失敗, 詳細信息: {str(e)}"

        @tool
        def list_waypoints_tool() -> str:
            """查詢當前載入的地圖中的地點列表信息，如果不知道導航地點，應該優先呼叫此工具"""
            req = ListWaypoints.Request()
            future = self.list_waypoints_client.call_async(req)
            try:
                res = self._wait_for_future(future, timeout_sec=5.0)
                if res.success:
                    ids = [wp.waypoint_id for wp in res.waypoints_info]
                    names = [wp.waypoint_name for wp in res.waypoints_info]
                    map_ids = [wp.map_id for wp in res.waypoints_info]
                    x = [wp.pose.position.x for wp in res.waypoints_info]
                    y = [wp.pose.position.y for wp in res.waypoints_info]
                    return f"執行結果: 成功, 地點列表信息(包含地點ID、地點名稱、地圖ID、地點座標): {list(zip(ids, names, map_ids, list(zip(x, y))))}, 詳細信息: {res.message}"
                else:
                    return f"執行結果: 失敗, 詳細信息: {res.message}"
            except Exception as e:
                return f"執行結果: 失敗, 詳細信息: {str(e)}"

        @tool
        def global_localization_tool() -> str:
            """進行全域定位，獲取當前座標"""
            goal = GlobalLocalization.Goal()
            future = self.global_localization_client.send_goal_async(goal)
            try:
                goal_handle = self._wait_for_future(future, timeout_sec=5.0)

                if goal_handle is None or not goal_handle.accepted:
                    return f"執行結果: 失敗, 詳細信息: 全域定位請求未被接受"

                result_future = goal_handle.get_result_async()
                action_result = self._wait_for_future(result_future, timeout_sec=300.0)
                self.get_logger().info(f"[Debug] 收到原始 Result 物件: {action_result.result}")
                self.get_logger().info(f"[Debug] 原始 message 內容: '{action_result.result.message}'")

                if action_result.status == GoalStatus.STATUS_SUCCEEDED:
                    return f"執行結果: 成功, 詳細信息: {action_result.result.message}"
                elif action_result.status == GoalStatus.STATUS_CANCELED:
                    return "執行結果: 取消, 詳細信息: 全域定位請求被系統或使用者取消"
                elif action_result.status == GoalStatus.STATUS_ABORTED:
                    return f"執行結果: 失敗, 詳細信息: 全域定位過程中發生錯誤"
                else:
                    return f"執行結果: 失敗, 詳細信息: {action_result.result.message}"
            except Exception as e:
                return f"執行結果: 失敗, 詳細信息: {str(e)}"

        @tool
        def navigate_tool(waypoint_id: str) -> str:
            """控制機器人導航到特定地點，請提供地點ID而非座標，如果不知道導航地點，應該優先呼叫 list_waypoints_tool 工具來查詢地點列表信息"""
            goal = Navigate.Goal()
            goal.waypoint_id = waypoint_id
            future = self.navigate_client.send_goal_async(goal)
            try:
                goal_handle = self._wait_for_future(future, timeout_sec=5.0)

                if goal_handle is None or not goal_handle.accepted:
                    return f"執行結果: 失敗, 詳細信息: 導航請求未被接受"

                result_future = goal_handle.get_result_async()
                action_result = self._wait_for_future(result_future, timeout_sec=200.0)
                self.get_logger().info(f"[Debug] 收到原始 Result 物件: {action_result.result}")
                self.get_logger().info(f"[Debug] 原始 message 內容: '{action_result.result.message}'")

                if action_result.status == GoalStatus.STATUS_SUCCEEDED:

                    return f"執行結果: 成功, 詳細信息: {action_result.result.message}"
                elif action_result.status == GoalStatus.STATUS_CANCELED:
                    return "執行結果: 取消, 詳細信息: 導航請求被系統或使用者取消"
                elif action_result.status == GoalStatus.STATUS_ABORTED:
                    return f"執行結果: 失敗, 詳細信息: 導航過程中發生錯誤"
                else:
                    return f"執行結果: 失敗, 詳細信息: {action_result.result.message}"
            except Exception as e:
                return f"執行結果: 失敗, 詳細信息: {str(e)}"

        # 對照表與工具集
        self.tools_map = {
            "create_map_tool": create_map_tool,
            "list_maps_tool": list_maps_tool,
            "switch_map_tool": switch_map_tool,
            "create_waypoint_tool": create_waypoint_tool,
            "list_waypoints_tool": list_waypoints_tool,
            "navigate_tool": navigate_tool,
            "global_localization_tool": global_localization_tool,
        }

        # 初始化模型並綁定工具
        stream_handler = RosStreamHandler(self.llm_stream_pub, self.speech_text_pub)
        raw_llm = ChatOllama(
            base_url=self.ollama_base_url,
            model=self.model_name,
            temperature=self.temperature,
            callbacks=[stream_handler],
        )
        self.llm_with_tools = raw_llm.bind_tools(list(self.tools_map.values()))

        # 定義對話提示模板，包含系統提示、歷史消息和工具調用結果
        self.prompt_template = ChatPromptTemplate.from_messages(
            [
                ("system", "{system_prompt}"),
                MessagesPlaceholder(variable_name="history"),
                ("human", "{input}"),
                MessagesPlaceholder(variable_name="agent_scratchpad"),
            ]
        )

        # 定義整合工具的 Agent Chain，將提示模板與工具調用結合起來
        self.agent_chain = RunnablePassthrough() | self.prompt_template | self.llm_with_tools

    def user_text_callback(self, msg: String) -> None:
        """處理使用者輸入"""
        user_input = msg.data

        with self.agent_busy_lock:
            if self.is_agent_running:
                warning_msg = "系統目前正在處理上一個指令，請稍後再試..."
                self.get_logger().warn(warning_msg)
                self.llm_response_pub.publish(String(data=warning_msg))
                return
            self.is_agent_running = True

        self.get_logger().info(f"📥 收到使用者輸入: {user_input}")
        # 啟動獨立執行緒進行 LLM 多步驟循環推理
        threading.Thread(target=self._run_agent_loop, args=(user_input,)).start()

    def _run_agent_loop(self, user_input: str) -> None:
        """ReAct 自主思考迴圈"""
        system_content_path = get_config_path("system_prompt.txt")
        if system_content_path:
            with open(system_content_path, "r", encoding="utf-8") as f:
                system_content = f.read()
        else:
            system_content = (
                "你是一個專業的智慧導航機器人助手，職責是幫助使用者管理地圖並完成多步驟導航\n"
                "你可以連續、分步驟地呼叫工具來完成任務，如果使用者給予複合指令，請一步一步調用工具\n"
            )
            self.get_logger().warning("✗ 無法找到 system_prompt.txt 使用內建預設提示語")

        with self.memory_lock:
            chat_history = self.memory.get_history_messages()

        agent_scratchpad: List[BaseMessage] = []
        max_iterations = 10
        self.get_logger().info("🧠 LLM 開始進入多步驟決策鏈...")

        try:
            for _ in range(max_iterations):
                response = self.agent_chain.invoke(
                    {
                        "system_prompt": system_content,
                        "history": chat_history,
                        "input": user_input,
                        "agent_scratchpad": agent_scratchpad,
                    }
                )

                # 檢查 LLM 是否需要叫工具
                if not response.tool_calls:
                    final_reply = str(response.content)
                    self.get_logger().info(f"🤖 Agent 最終決策回應: {final_reply}")

                    with self.memory_lock:
                        self.memory.add_message(HumanMessage(content=user_input))
                        self.memory.add_message(AIMessage(content=final_reply))

                    self.llm_response_pub.publish(String(data=final_reply))
                    return

                # 記錄 LLM 的工具調用請求
                agent_scratchpad.append(response)

                # 處理工具調用
                for tool_call in response.tool_calls:
                    tool_name = tool_call["name"]
                    tool_args = tool_call["args"]
                    tool_id = tool_call["id"]

                    self.get_logger().info(f"🛠️ LLM 決定呼叫服務: {tool_name}, 參數: {tool_args}")

                    if tool_name in self.tools_map:
                        tool_result = self.tools_map[tool_name].invoke(tool_args)
                    else:
                        tool_result = f"錯誤：找不到工具 {tool_name}"

                    self.get_logger().info(f"📥 服務回傳數據: {tool_result}")

                    # 紀錄 LLM 的工具調用結果
                    agent_scratchpad.append(
                        ToolMessage(
                            content=str(tool_result),
                            tool_call_id=tool_id,
                            name=tool_name,
                        )
                    )

            # 超過最大步數
            timeout_msg = "任務執行步驟過多，已強制中止防止機器人發生異常"
            self.llm_response_pub.publish(String(data=timeout_msg))

        except Exception as e:
            error_msg = f"✗ Agent 執行時發生異常: {str(e)}"
            self.get_logger().error(error_msg)
            self.llm_response_pub.publish(String(data=error_msg))
        finally:
            with self.agent_busy_lock:
                self.is_agent_running = False


def main(args=None):
    rclpy.init(args=args)
    node = LLMServiceNode()
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
