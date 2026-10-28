#!/usr/bin/env python3

"""迎賓對話可以呼叫的工具

LLM 只負責「理解客人要什麼」並提出請求；能不能做、怎麼做，由別的節點決定：

  list_waypoints_tool    唯讀：查地圖上的地點（waypoint_service）
  navigate_tool          帶位。送給 smartnav_brain 的 nav_arbiter，由它依客人身分放行或拒絕
  notify_staff_tool      通報行員。發 /staff_notify_request，由 bank_reception_node 送出（通報只有一個出口）
  query_exchange_rate_tool / query_datetime_tool   即時資訊（realtime_info.py），可用 enable_realtime_info 關掉

本節點不改地圖、不碰 /navigate：建圖、切圖、建地點是 HMI 的管理功能。
"""

from langchain_core.tools import tool
from std_msgs.msg import String

from smartnav_msgs.srv import ListWaypoints

from smartnav_llm import realtime_info


def make_tools(node, enable_realtime_info: bool) -> dict:
    """建立工具對照表 {工具名: tool}

    Args:
        node: LLMServiceNode（使用其 list_waypoints_client、staff_notify_pub、_navigate_to、_wait_for_future）
        enable_realtime_info: 是否掛上匯率與時間查詢
    """

    @tool
    def list_waypoints_tool() -> str:
        """查詢當前載入的地圖中的地點列表信息，如果不知道導航地點，應該優先呼叫此工具"""
        try:
            res = node._wait_for_future(node.list_waypoints_client.call_async(ListWaypoints.Request()), 5.0)
            if not res.success:
                return f"執行結果: 失敗, 詳細信息: {res.message}"
            rows = [
                (wp.waypoint_id, wp.waypoint_name, wp.map_id, (wp.pose.position.x, wp.pose.position.y))
                for wp in res.waypoints_info
            ]
            return (
                f"執行結果: 成功, 地點列表信息(包含地點ID、地點名稱、地圖ID、地點座標): {rows}, 詳細信息: {res.message}"
            )
        except Exception as e:
            return f"執行結果: 失敗, 詳細信息: {str(e)}"

    @tool
    def navigate_tool(waypoint_id: str) -> str:
        """帶客戶走到特定地點（機器人會實際開過去帶路），請提供地點ID而非座標。
        不知道地點ID時，先呼叫 list_waypoints_tool 查詢；貴賓室也是一般地點，用這個工具帶過去"""
        # 地點是否存在、這位客人能不能去，都由 nav_arbiter 判斷；被拒絕時結果裡會帶要對客人說的話
        try:
            return node._navigate_to(waypoint_id)
        except Exception as e:
            return f"執行結果: 失敗, 詳細信息: {str(e)}"

    @tool
    def notify_staff_tool() -> str:
        """客戶要找真人、找行員、找專員、要求真人服務或轉接櫃檯時，一定要呼叫此工具通報行員前來。
        遇到黑名單人員、或發生機器人無法處理的狀況時也呼叫此工具。
        機器人本身沒有辦法「轉接」或「安排連線」，唯一能做的就是呼叫此工具請行員過來"""
        try:
            # 通報內容用客人的原句，不用模型自己寫的「原因」——小模型會在這裡亂碼或混入簡體字，
            # 而這段文字會直接送到行員的手機（bank_reception_node → Telegram）
            said = (node.current_user_text or "").strip() or "（未取得客戶原話）"
            node.staff_notify_pub.publish(String(data=f"🔔 機器人通報：客戶說「{said}」"))
            return "執行結果: 成功, 詳細信息: 已送出行員通報，請告知客戶行員即將前來"
        except Exception as e:
            return f"執行結果: 失敗, 詳細信息: {str(e)}"

    @tool
    def query_exchange_rate_tool(currency: str, amount: float = 1.0) -> str:
        """查詢外幣對新台幣的即時參考匯率。currency 填幣別，例如「美金」「日幣」；
        amount 填客戶要換算的外幣金額，客戶沒說金額就不用填"""
        try:
            return realtime_info.query_exchange_rate(currency, amount)
        except Exception as e:
            return f"執行結果: 失敗, 詳細信息: 匯率查詢發生錯誤 {str(e)}"

    @tool
    def query_datetime_tool() -> str:
        """查詢**此刻的鐘面時間**，例如「現在幾點」「今天幾號」「今天星期幾」。
        ★ 只在客戶問當下時刻時使用。客戶問「你們幾點關門」「營業到幾點」問的是
        本行的營業時段，那要用查銀行業務資料的工具，不要用這個"""
        try:
            return realtime_info.query_datetime()
        except Exception as e:
            return f"執行結果: 失敗, 詳細信息: 時間查詢發生錯誤 {str(e)}"

    tools = {
        "list_waypoints_tool": list_waypoints_tool,
        "navigate_tool": navigate_tool,
        "notify_staff_tool": notify_staff_tool,
    }
    if enable_realtime_info:
        tools["query_exchange_rate_tool"] = query_exchange_rate_tool
        tools["query_datetime_tool"] = query_datetime_tool
    return tools
