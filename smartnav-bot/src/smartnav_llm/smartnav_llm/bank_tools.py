#!/usr/bin/env python3

"""銀行場景 LLM 工具集

供 llm_service_node 掛載的三個銀行工具：
  guide_to_vip_room_tool  帶 VIP 到貴賓室（查 waypoint → 導航）
  notify_staff_tool       通報行員（發布 /staff_notify_request，由 bank_reception_node 送 Telegram）
  query_bank_faq_tool     查詢銀行常見問題（讀 config/bank_faq.txt 供 LLM 摘取）

設計原則：不直接依賴外部網路服務——通報實際發送集中在 smartnav_brain
（通報中樞），本模組只發 ROS topic，無 brain 節點時工具仍可執行（會提示）。
"""

from langchain_core.tools import tool

from smartnav_msgs.srv import ListWaypoints
from smartnav_msgs.action import Navigate
from action_msgs.msg import GoalStatus

from smartnav_llm.llm_utils import get_config_path


def make_bank_tools(node) -> dict:
    """建立銀行工具對照表

    Args:
        node: LLMServiceNode 實例（複用其 service/action clients 與 _wait_for_future）

    Returns:
        dict: {工具名: tool 物件}，格式與 tools_map 相同
    """

    @tool
    def guide_to_vip_room_tool() -> str:
        """帶領 VIP 貴賓前往貴賓室。當 VIP 貴賓要求帶位、或同意前往貴賓室時呼叫此工具"""
        room_name = node.vip_room_waypoint_name
        try:
            if not node.list_waypoints_client.service_is_ready():
                return f"執行結果: 失敗, 詳細信息: 導航系統未啟動，無法帶位，請以口頭指引貴賓前往{room_name}"

            res = node._wait_for_future(node.list_waypoints_client.call_async(ListWaypoints.Request()), 5.0)
            if not res.success:
                return f"執行結果: 失敗, 詳細信息: 無法取得地點列表（{res.message}）"

            waypoint_id = None
            known_names = []
            for wp in res.waypoints_info:
                known_names.append(wp.waypoint_name)
                if wp.waypoint_name == room_name:
                    waypoint_id = wp.waypoint_id
            if waypoint_id is None:
                return (
                    f"執行結果: 失敗, 詳細信息: 地圖中找不到名為「{room_name}」的地點，"
                    f"現有地點: {known_names}，請先在貴賓室位置建立同名地點"
                )

            goal = Navigate.Goal()
            goal.waypoint_id = waypoint_id
            goal_handle = node._wait_for_future(node.navigate_client.send_goal_async(goal), 5.0)
            if goal_handle is None or not goal_handle.accepted:
                return "執行結果: 失敗, 詳細信息: 導航請求未被接受"

            action_result = node._wait_for_future(goal_handle.get_result_async(), 200.0)
            if action_result.status == GoalStatus.STATUS_SUCCEEDED:
                return f"執行結果: 成功, 詳細信息: 已將貴賓帶到{room_name}"
            return f"執行結果: 失敗, 詳細信息: {action_result.result.message}"
        except Exception as e:
            return f"執行結果: 失敗, 詳細信息: {str(e)}"

    @tool
    def notify_staff_tool(reason: str) -> str:
        """通報行員前來處理。遇到黑名單人員、客戶要求真人服務、或發生機器人無法處理的狀況時呼叫此工具，reason 填通報原因"""
        try:
            from std_msgs.msg import String

            node.staff_notify_pub.publish(String(data=f"🔔 機器人通報：{reason}"))
            return "執行結果: 成功, 詳細信息: 已送出行員通報，請告知客戶行員即將前來"
        except Exception as e:
            return f"執行結果: 失敗, 詳細信息: {str(e)}"

    @tool
    def query_bank_faq_tool(question: str) -> str:
        """查詢銀行常見問題資料（營業時間、開戶、匯兌等）。客戶詢問銀行業務相關問題時，先呼叫此工具取得正確資訊再回答"""
        faq_path = get_config_path("bank_faq.txt")
        if faq_path is None:
            return "執行結果: 失敗, 詳細信息: 找不到銀行FAQ資料檔，請告知客戶洽詢櫃台"
        try:
            with open(faq_path, "r", encoding="utf-8") as f:
                faq_content = f.read()
            return f"執行結果: 成功, 銀行FAQ資料如下（請從中摘取與問題相關的內容回答）:\n{faq_content}"
        except Exception as e:
            return f"執行結果: 失敗, 詳細信息: {str(e)}"

    return {
        "guide_to_vip_room_tool": guide_to_vip_room_tool,
        "notify_staff_tool": notify_staff_tool,
        "query_bank_faq_tool": query_bank_faq_tool,
    }
