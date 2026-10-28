"""依 brain 回報的客人等級，產生附在系統提示最後的「目前的客人」說明。

只決定「怎麼說話」：能不能帶位、能去哪裡由 smartnav_brain 的 nav_arbiter 判斷，這裡不判斷也不執行。
"""


def prompt_context(tier: str, user_name: str, description: str) -> str:
    """只有 VIP 會拿到姓名；黑名單只給中性的服務限制、不說原因；一般訪客不附加。"""
    if tier == "vip":
        name = (user_name or "").strip()
        desc = (description or "").strip()
        lines = ["", "---", "", "# 目前在機器人前面的客人（人臉辨識，會隨時更新）", f"- 這位是本行貴賓：{name or '（未提供姓名）'}"]
        if desc:
            lines.append(f"- 備註：{desc}")
        lines.append("- **你回覆的第一句話必須以「" + (name or "貴賓") + "貴賓您好」開頭**（後面再接回答），語氣更親切，並主動提供貴賓服務（例如帶位到貴賓室、請專員協助）。")
        lines.append("- 不要提到相似度、辨識、資料庫這些系統細節。")
        return "\n".join(lines)
    if tier == "blocked":
        return ("\n\n---\n\n# 目前的服務限制\n- 這位客人目前不提供帶位服務。客人要求帶路時，說「需要協助的話，行員馬上會過來為您服務」。\n"
                "- 不可以提到原因、身分、名單或辨識；其他一般問題照常回答。")
    return ""
