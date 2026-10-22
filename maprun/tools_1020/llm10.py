#!/usr/bin/env python3
"""LLM 銀行模式 10 題（交接文件 §5-8）：自動送題、收答案、量延遲、存 CSV。

用法（車上；LLM 單元要開著，Ollama 在筆電上要開著；★語音輸入單元要先關掉，否則麥克風雜音會餵進 LLM，9/17 就是這樣）：
    source ~/maprun/env.sh
    python3 ~/maprun/tools_1020/llm10.py              # 逐題送，②③⑩ 會先停下來等你確認
    python3 ~/maprun/tools_1020/llm10.py --dry-run    # 只印題目與預期，不連 ROS
    python3 ~/maprun/tools_1020/llm10.py --clear-each --repeat 2 --nav-down-ok
        # 每題前清對話記憶（避免前一題污染）、整套跑兩輪；導航堆疊沒開時②③⑩直接送（車不會動，只驗工具選擇）

★ 第 ②③④⑩ 題（櫃檯在哪、貴賓室怎麼去、洗手間在哪〔地圖有廁所時〕、帶我去大廳）會觸發「帶位／導航」——導航堆疊開著而且地點標好時，
  **車會真的自己開過去**。這三題前面會停下來問你：車旁有人、急停備好、路線上沒東西 → 按 Enter；輸入 s 跳過該題。
  送出後腳本不會等車到達；車到了或你按了急停，再按 Enter 才會送下一題。

題目、預期都是**虛構示範資料**（地圖要先標好「大廳、門口、櫃檯、貴賓室、起點」，名稱一字不差）。
輸出：~/maprun/verify_out/llm10_<HHMM>.csv（序號、題目、預期、答案、延遲秒數、旗標）。
★ 旗標 BUSY＝LLM 回「系統目前正在處理上一個指令」（還有別的輸入在餵它）；TIMEOUT＝等 90 秒沒有回應。
★ 答案對不對由人（或筆電端）依「預期」欄判，這支不判對錯。
★ 2026-10-20 車上新寫，只用假 LLM（隔離網域）驗證過流程，沒有在實車驗證過。
★ 15:15 加 --clear-each／--repeat／--nav-down-ok 與「工具呼叫」欄（從 llm log 依時間窗撈 🛠️ 行）。
"""
import argparse
import csv
import glob
import os
import re
import subprocess
import sys
import time
import urllib.request

QUESTIONS = [
    (1, "請問銀行幾點營業？", "週一到五 9:00～15:30", False),
    (2, "請問櫃檯在哪裡？", "帶位（不能編樓層）", True),
    (3, "貴賓室要怎麼去？", "帶位", True),
    # ★ 2026-10-22：現用地圖有「廁所」地點時，④ 會導航過去（洗手間＝廁所）——跟②③⑩一樣先停下來問
    (4, "請問洗手間在哪裡？", "地圖有「廁所」→ 導航過去；沒有才回「示範場地沒有設定洗手間」", True),
    (5, "請問 ATM 在哪裡？", "同④：沒有設定這個地點", False),
    (6, "我想開戶，需要帶什麼？", "身分證＋第二證件＋印章，且說明機器人不能直接開戶", False),
    (7, "請問轉帳的手續費是多少？", "沒有資訊、請洽櫃檯", False),
    (8, "我想換美金。", "到櫃檯、可問匯率", False),
    (9, "我的信用卡掉了怎麼辦？", "打發卡銀行；示範銀行沒有掛失專線", False),
    (10, "請帶我去大廳。", "導航到大廳", True),
]
BUSY_TXT = "系統目前正在處理上一個指令"
OLLAMA = "http://192.168.137.1:11434/api/tags"
LLM_LOG_DIR = os.path.expanduser("~/.smartnav/logs/system_control")
TOOL_MARK = "🛠️ LLM 決定呼叫服務:"
NAV_NODES = ("path_teach_cc_node", "navigation_action_cc_node", "controller_server", "bt_navigator")


def tools_called(t_from, t_to):
    """從最新的 llm log 撈出這段時間內 LLM 呼叫了哪些工具（名稱＋參數，節錄）"""
    fs = glob.glob(os.path.join(LLM_LOG_DIR, "llm_*.log"))
    if not fs:
        return "（找不到 llm log）"
    out = []
    with open(max(fs, key=os.path.getmtime), encoding="utf-8", errors="replace") as fh:
        for ln in fh:
            if TOOL_MARK not in ln:
                continue
            m = re.search(r"\[(\d{10}\.\d+)\]", ln)
            if m and t_from <= float(m.group(1)) <= t_to:
                out.append(ln.split(TOOL_MARK, 1)[1].strip()[:120])
    return " | ".join(out) or "無"


def main():
    ap = argparse.ArgumentParser(description="LLM 銀行模式 10 題")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--timeout", type=float, default=90.0)
    ap.add_argument("--force", action="store_true", help="語音輸入單元沒關也照跑（不建議）")
    ap.add_argument("--skip-ollama-check", action="store_true", help="（測試用）略過筆電 Ollama 連線檢查")
    ap.add_argument("--clear-each", action="store_true", help="每題送出前清對話記憶（/clear_conversation）")
    ap.add_argument("--repeat", type=int, default=1, help="整套題目跑幾輪（預設 1）")
    ap.add_argument("--extra-nav", action="append", default=[], metavar="句子",
                    help="多加一題會讓車開走的題目（可重複），例：--extra-nav 帶我去門口")
    ap.add_argument("--settle", type=float, default=3.0,
                    help="收到第一則回覆後再收幾秒沒有新訊息才換下一題（帶位題的「X到了」是第二則；0＝不等）")
    ap.add_argument("--nav-down-ok", action="store_true",
                    help="導航堆疊沒開時，②③⑩不停下來問、直接送（車不會動，只驗工具選擇）；導航開著照樣會問")
    ap.add_argument("--out", default=os.path.expanduser(f"~/maprun/verify_out/llm10_{time.strftime('%H%M')}.csv"))
    a = ap.parse_args()

    if a.dry_run:
        print("【dry-run】題目與預期：")
        for i, q, exp, moves in QUESTIONS:
            print(f"  {i:>2}. {q}   預期：{exp}" + ("   ★會讓車開走" if moves else ""))
        return 0

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Empty, String

    rclpy.init()
    n = Node("llm10_cc")
    pub = n.create_publisher(String, "/user_text", 10)
    clr = n.create_publisher(Empty, "/clear_conversation", 10)
    got = []
    n.create_subscription(String, "/llm_response", lambda m: got.append((time.time(), m.data)), 10)

    # ── 前置 ──
    t0 = time.time()
    while time.time() - t0 < 5:
        rclpy.spin_once(n, timeout_sec=0.2)
    print("══ 前置檢查 ══")
    nodes = {nm for nm, _ in n.get_node_names_and_namespaces()}
    if pub.get_subscription_count() == 0:
        print("  ✗ /user_text 沒有訂閱者 —— LLM 單元沒開（平板「系統開關」開「LLM 對話」）")
        return 1
    print(f"  ✓ /user_text 有 {pub.get_subscription_count()} 個訂閱者")
    asr = sorted(x for x in nodes if "speech_recognizer" in x or "voice_trigger" in x)
    if asr and not a.force:
        print(f"  ✗ 語音輸入還開著（{', '.join(asr)}）—— 麥克風雜音會餵進 LLM。請在平板把「語音輸入」單元關掉再跑（硬要跑加 --force）")
        return 1
    print("  ✓ 語音輸入沒開" if not asr else f"  ⚠ 語音輸入開著（--force）：{', '.join(asr)}")
    if a.clear_each:
        if clr.get_subscription_count() == 0:
            print("  ✗ /clear_conversation 沒有訂閱者 —— 清不了記憶（LLM 節點版本不同？）")
            return 1
        print(f"  ✓ 每題前清對話記憶（/clear_conversation 有 {clr.get_subscription_count()} 個訂閱者）")
    nav_now = sorted(set(NAV_NODES) & nodes)
    print(f"  {'⚠ 導航節點在線：' + ', '.join(nav_now) + ' —— ②③⑩ 會讓車開走' if nav_now else '✓ 導航堆疊沒開（②③⑩ 車不會動）'}")
    if a.skip_ollama_check:
        print("  （略過 Ollama 連線檢查：--skip-ollama-check）")
    else:
      try:
        with urllib.request.urlopen(OLLAMA, timeout=5) as r:
                ok = r.status == 200
        print(f"  {'✓' if ok else '✗'} 筆電 Ollama {OLLAMA} → HTTP {r.status}")
      except Exception as e:  # noqa: BLE001
        print(f"  ✗ 連不到筆電的 Ollama（{e.__class__.__name__}）—— 請使用者在筆電確認 ollama 有開、熱點 IP 沒變")
        return 1

    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    trip = os.path.expanduser("~/maprun/trip_mark.sh")
    if os.path.exists(trip):
        subprocess.run([trip, "T7 LLM 十題開始", f"csv={a.out}"], capture_output=True, timeout=5)
    rows, spans = [], []   # spans[k]＝第 k 列的（送出時間, 收到答案或逾時的時間），事後拿去 llm log 撈工具呼叫
    print("\n══ 開始（Ctrl-C 中止）══")
    try:
        for rep in range(1, a.repeat + 1):
            for i, q, exp, moves in QUESTIONS + [(len(QUESTIONS) + k + 1, x, "帶位（--extra-nav）", True)
                                                 for k, x in enumerate(a.extra_nav)]:
                direct = False
                if moves:
                    # 每題重查：中途有人開了導航，就回到「停下來問人」
                    nav_up = sorted(set(NAV_NODES) & {nm for nm, _ in n.get_node_names_and_namespaces()})
                    if a.nav_down_ok and not nav_up:
                        direct = True
                        print(f"\n[{rep}-{i}] 這題會觸發帶位／導航，但導航堆疊沒開 → 車不會動，直接送（只驗工具選擇）")
                    else:
                        ans = input(f"\n⚠ 第 {i} 題「{q}」會觸發帶位／導航，車可能自己開走"
                                    + (f"（導航節點在線：{', '.join(nav_up)}）" if nav_up else "")
                                    + "。\n  確認：車旁有人、急停備好、路線上沒東西 → 按 Enter 送出；輸入 s 跳過：").strip().lower()
                        if ans == "s":
                            rows.append([rep, i, q, exp, "", "", "SKIPPED", "", ""])
                            spans.append(None)
                            print(f"  [{i}] 已跳過")
                            continue
                if a.clear_each:
                    clr.publish(Empty())
                    t_c = time.time()
                    while time.time() - t_c < 0.8:
                        rclpy.spin_once(n, timeout_sec=0.1)
                # 先把上一題晚到的訊息收乾淨再清空，免得被記成這一題的答案
                t_d = time.time()
                while time.time() - t_d < 0.3:
                    rclpy.spin_once(n, timeout_sec=0.05)
                got.clear()
                t_send = time.time()
                pub.publish(String(data=q))
                flags, answer, lat = [], "", None
                deadline = t_send + a.timeout
                while time.time() < deadline:
                    rclpy.spin_once(n, timeout_sec=0.2)
                    if got:
                        ts, txt = got[0]
                        if BUSY_TXT in txt:
                            flags.append("BUSY")
                            got.pop(0)
                            deadline = max(deadline, time.time() + 40)   # 真正的答案可能晚一點才來
                            continue
                        answer, lat = txt, ts - t_send
                        break
                if lat is not None and a.settle > 0:
                    # ★ 2026-10-22：帶位直達會發兩則（「好的，請跟我來。」→ 到了／失敗）。只收第一則就換題的話，
                    #   第二則會在下一題的視窗裡到、被記成下一題的答案；下一題還會因為帶位沒結束被回「處理上一個指令」丟掉。
                    seen, last = got.index((ts, txt)) + 1, time.time()
                    # 會開車的題目一定要等到第二則（到了／失敗）——路程多長都一樣，不能只靠 settle 秒數
                    need_more = moves and direct
                    while (need_more or time.time() - last < a.settle) and time.time() < t_send + a.timeout:
                        rclpy.spin_once(n, timeout_sec=0.2)
                        if len(got) > seen:
                            for _ts, _txt in got[seen:]:
                                if BUSY_TXT not in _txt:
                                    answer += " ｜ " + _txt
                                    flags.append(f"+{_ts - t_send:.1f}s")
                                    need_more = False
                            seen, last = len(got), time.time()
                if lat is None:
                    flags.append("TIMEOUT")
                print(f"\n[{rep}-{i}] 問：{q}\n    答：{answer or '（無）'}\n    延遲：{'—' if lat is None else f'{lat:.2f} s'}   旗標：{','.join(flags) or '無'}   預期：{exp}")
                rows.append([rep, i, q, exp, answer.replace("\n", " "), "" if lat is None else f"{lat:.2f}",
                             ",".join(flags), "", "是" if a.clear_each else "否"])
                spans.append((t_send, t_send + lat if lat is not None else time.time()))
                if moves and not direct:
                    input("  車到了（或已按急停）再按 Enter 送下一題：")
                else:
                    time.sleep(1.0)
    except (KeyboardInterrupt, EOFError):
        print("\n中止。已完成的題目照常存檔。")
    finally:
        time.sleep(1.5)        # 等 llm log 寫完最後一題
        for k, sp in enumerate(spans):
            if sp:
                rows[k][7] = tools_called(sp[0] - 0.2, sp[1] + 0.5)
        with open(a.out, "w", newline="", encoding="utf-8-sig") as fh:
            w = csv.writer(fh)
            w.writerow(["輪", "序號", "題目", "預期", "答案", "延遲秒", "旗標", "工具呼叫（llm log）", "每題前清記憶"])
            w.writerows(rows)
        if any(spans):
            print("\n══ 工具呼叫（llm log）══")
            for r in rows:
                print(f"  [{r[0]}-{r[1]}] {r[2][:12]:<12} → {r[7] or '—'}")
        lats = [float(r[5]) for r in rows if r[5]]
        print(f"\n══ 結束：{len(rows)} 題有紀錄、{len(lats)} 題有延遲" + (f"（{min(lats):.2f}~{max(lats):.2f} s；9/17 基準 0.94~1.79 s）" if lats else "") + f" ══\n存檔：{a.out}")
        if os.path.exists(trip):
            subprocess.run([trip, "T7 LLM 十題結束", f"{len(rows)} 題；csv={a.out}"], capture_output=True, timeout=5)
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
