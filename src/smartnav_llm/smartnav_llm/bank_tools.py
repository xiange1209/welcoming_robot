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
        """帶客戶走到貴賓室。客戶說「帶我去貴賓室」「請帶路」「我要去 VIP 室」或同意前往貴賓室時，
        一定要呼叫此工具，機器人會實際開過去帶路。不要只用嘴巴指路，也不要反問客戶要不要帶位"""
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

            # ★ 2026-08-26：逾時要主動取消 goal，否則嘴上說失敗、車還在走。
            #   見 llm_service_node._wait_for_action_result 的長註解。
            action_result = node._wait_for_action_result(goal_handle, 200.0, "帶位")
            if action_result.status == GoalStatus.STATUS_SUCCEEDED:
                return f"執行結果: 成功, 詳細信息: 已將貴賓帶到{room_name}"
            return f"執行結果: 失敗, 詳細信息: {action_result.result.message}"
        except Exception as e:
            return f"執行結果: 失敗, 詳細信息: {str(e)}"

    @tool
    def notify_staff_tool(reason: str) -> str:
        """客戶要找真人、找行員、找專員、要求真人服務或轉接櫃檯時，一定要呼叫此工具通報行員前來。
        遇到黑名單人員、或發生機器人無法處理的狀況時也呼叫此工具。reason 填通報原因。
        機器人本身沒有辦法「轉接」或「安排連線」，唯一能做的就是呼叫此工具請行員過來"""
        try:
            from std_msgs.msg import String

            node.staff_notify_pub.publish(String(data=f"🔔 機器人通報：{reason}"))
            return "執行結果: 成功, 詳細信息: 已送出行員通報，請告知客戶行員即將前來"
        except Exception as e:
            return f"執行結果: 失敗, 詳細信息: {str(e)}"

    @tool
    def query_bank_faq_tool(question: str) -> str:
        """查詢本行的營業與業務資料。客戶問「幾點開門」「幾點關門」「營業到幾點」「假日有沒有開」
        這類**本行營業時間**的問題，以及開戶、要帶什麼證件、換匯、外幣、貸款、信用卡、掛失、
        櫃台位置、手續費、貴賓室等本行業務問題時，都必須先呼叫此工具取得正確資訊再回答。
        注意：問「本行幾點關門」要用這個工具，不是查現在時刻的工具。請把客戶問題原句傳入"""
        # ★ 2026-08-14：資料來源刻意**不走** knowledge_store（RAG）。
        #
        # 一度改成優先用 node.knowledge_store.build_context()，因為知識庫工具
        # 已經在銀行模式下取消掛載（見 llm_service_node），想把檢索能力搬進來。
        # 實測後撤回：knowledge/bank_faq.md 是一份**還沒填的範本**，
        # 整份有 62 個「（請填寫）」。模型會把括號裡的「例如 …」當成答案唸出來，
        # 還會自己補值——實測回答「開戶約需 15 分鐘」，但 config/bank_faq.txt
        # 寫的是 30 分鐘，那個 15 是憑空生出來的。
        #
        # config/bank_faq.txt 是目前唯一填好的一份，所以就讀它。
        # 等 knowledge/bank_faq.md 填完，再考慮切回 RAG（那時才有檢索的價值）。
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
