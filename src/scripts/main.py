#!/usr/bin/env python3
"""
智慧銀行 VIP 迎賓系統 - 主選單
統一入口：啟動所有已實作的功能
"""

import os
import sys
import subprocess
import time
from pathlib import Path

# Windows 中文輸出支援
if sys.platform == "win32":
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")

# Linux 自動設定 VNC 顯示器
if sys.platform != "win32" and not os.environ.get("DISPLAY"):
    os.environ["DISPLAY"] = ":1"

try:
    import psutil
    PSUTIL_AVAILABLE = True
except ImportError:
    PSUTIL_AVAILABLE = False

SCRIPT_DIR = Path(__file__).parent
PROJECT_ROOT = SCRIPT_DIR.parent.parent

# ANSI 顏色碼
RESET   = "\033[0m"
BOLD    = "\033[1m"
DIM     = "\033[2m"
GREEN   = "\033[92m"
YELLOW  = "\033[93m"
RED     = "\033[91m"
CYAN    = "\033[96m"
WHITE   = "\033[97m"
GRAY    = "\033[90m"


def clear():
    os.system("cls" if os.name == "nt" else "clear")


def get_system_status() -> str:
    """讀取 CPU / RAM / 溫度"""
    if not PSUTIL_AVAILABLE:
        return f"{DIM}psutil 未安裝，無法讀取系統狀態{RESET}"

    cpu = psutil.cpu_percent(interval=0.3)
    ram_mb = psutil.virtual_memory().used / (1024 ** 2)

    temp_str = ""
    try:
        result = subprocess.run(
            ["vcgencmd", "measure_temp"],
            capture_output=True, text=True, timeout=2
        )
        if result.returncode == 0:
            temp_str = f" | 溫度 {result.stdout.strip().replace('temp=', '')}"
    except Exception:
        pass

    cpu_color = RED if cpu > 85 else YELLOW if cpu > 60 else GREEN
    return (f"CPU {cpu_color}{cpu:.0f}%{RESET} | "
            f"RAM {ram_mb:.0f} MB{temp_str}")


def print_header():
    status = get_system_status()
    print(f"{CYAN}{BOLD}")
    print("╔══════════════════════════════════════════════╗")
    print("║      智慧銀行 VIP 迎賓與安全通報系統          ║")
    print("║      Phase 2b  |  RPi4 Edge Computing       ║")
    print("╚══════════════════════════════════════════════╝")
    print(f"{RESET}  系統狀態：{status}")
    print()


def print_menu():
    print(f"{BOLD}{WHITE}── 人臉識別 ─────────────────────────────────{RESET}")
    print(f"  {WHITE}[1]{RESET} 啟動實時人臉檢測 (GUI)            {GREEN}✅ 已實作{RESET}")
    print(f"  {WHITE}[2]{RESET} 單幀照片識別測試                   {GREEN}✅ 已實作{RESET}")
    print()
    print(f"{BOLD}{WHITE}── VIP 管理 ─────────────────────────────────{RESET}")
    print(f"  {WHITE}[3]{RESET} 新增 VIP（照片路徑）               {GREEN}✅ 已實作{RESET}")
    print(f"  {WHITE}[4]{RESET} 新增 VIP（攝影機即時拍攝）         {GREEN}✅ 已實作{RESET}")
    print(f"  {WHITE}[5]{RESET} 查看所有 VIP 名單                  {GREEN}✅ 已實作{RESET}")
    print()
    print(f"{BOLD}{WHITE}── 黑名單管理 ───────────────────────────────{RESET}")
    print(f"  {WHITE}[6]{RESET} 新增黑名單（照片路徑）             {GREEN}✅ 已實作{RESET}")
    print(f"  {WHITE}[7]{RESET} 新增黑名單（攝影機即時拍攝）       {GREEN}✅ 已實作{RESET}")
    print(f"  {WHITE}[8]{RESET} 查看黑名單                         {GREEN}✅ 已實作{RESET}")
    print()
    print(f"{BOLD}{WHITE}── 系統工具 ─────────────────────────────────{RESET}")
    print(f"  {WHITE}[9]{RESET}  性能基準測試                      {GREEN}✅ 已實作{RESET}")
    print(f"  {WHITE}[10]{RESET} 初始化資料庫                      {GREEN}✅ 已實作{RESET}")
    print(f"  {WHITE}[11]{RESET} 刪除 VIP                          {GREEN}✅ 已實作{RESET}")
    print(f"  {WHITE}[12]{RESET} 刪除黑名單                        {GREEN}✅ 已實作{RESET}")
    print()
    print(f"{BOLD}{GRAY}── 規劃中 (Phase 3+，尚未實作) ──────────────{RESET}")
    print(f"  {GRAY}[13]{RESET} {GRAY}語音銀行助手 (Gemma / 本機 LLM)   🔄 規劃中{RESET}")
    print(f"  {GRAY}[14]{RESET} {GRAY}人臉數據增強 (多樣本生成)          🔄 規劃中{RESET}")
    print(f"  {GRAY}[15]{RESET} {GRAY}ROS 2 節點管理                     🔄 規劃中{RESET}")
    print()
    print(f"  {WHITE}[0]{RESET} 退出")
    print()


def run_script(args: list, cwd=None):
    """執行子腳本，結束後暫停讓使用者閱讀輸出"""
    print(f"\n{DIM}{'─' * 50}{RESET}")
    try:
        subprocess.run(
            [sys.executable] + args,
            cwd=cwd or PROJECT_ROOT
        )
    except KeyboardInterrupt:
        print(f"\n{YELLOW}已中斷，返回主選單{RESET}")
    print(f"\n{DIM}{'─' * 50}{RESET}")
    input(f"{DIM}按 Enter 返回主選單...{RESET}")


def action_realtime():
    print(f"\n{YELLOW}啟動實時人臉檢測...{RESET}")
    print(f"{DIM}按 Q 退出，S 保存當前幀，Space 暫停{RESET}\n")
    run_script([str(SCRIPT_DIR / "realtime_detection_insightface.py")])


def action_test_recognition():
    print(f"\n{WHITE}單幀照片識別測試{RESET}")
    img_path = input("  照片路徑: ").strip()
    if not img_path:
        print(f"{RED}未輸入路徑，取消{RESET}")
        time.sleep(1)
        return
    if not Path(img_path).exists():
        print(f"{RED}找不到檔案：{img_path}{RESET}")
        time.sleep(1.5)
        return
    run_script([str(SCRIPT_DIR / "test_vip_recognition.py"), img_path])


def action_add_vip():
    print(f"\n{WHITE}新增 VIP（照片路徑，交互式）{RESET}\n")
    run_script([str(SCRIPT_DIR / "manage_vip_database.py"), "add-vip"])


def action_add_vip_camera():
    print(f"\n{WHITE}新增 VIP（攝影機即時拍攝）{RESET}")
    print(f"{DIM}請面對攝影機，按空白鍵逐張拍攝 5 張樣本{RESET}\n")
    run_script([str(SCRIPT_DIR / "manage_vip_database.py"), "add-vip-camera"])


def action_list_vips():
    print(f"\n{WHITE}所有 VIP 名單{RESET}\n")
    run_script([str(SCRIPT_DIR / "manage_vip_database.py"), "list-vips"])


def action_add_blacklist():
    print(f"\n{WHITE}新增黑名單（照片路徑，交互式）{RESET}\n")
    run_script([str(SCRIPT_DIR / "manage_vip_database.py"), "add-blacklist"])


def action_add_blacklist_camera():
    print(f"\n{WHITE}新增黑名單（攝影機即時拍攝）{RESET}")
    print(f"{DIM}請面對攝影機，按空白鍵逐張拍攝 5 張樣本{RESET}\n")
    run_script([str(SCRIPT_DIR / "manage_vip_database.py"), "add-blacklist-camera"])


def action_list_blacklist():
    print(f"\n{WHITE}黑名單{RESET}\n")
    run_script([str(SCRIPT_DIR / "manage_vip_database.py"), "list-blacklist"])


def action_benchmark():
    print(f"\n{WHITE}性能基準測試{RESET}")
    print(f"{DIM}測試推理延遲、CPU、記憶體使用率（需要數分鐘）{RESET}\n")
    run_script([str(SCRIPT_DIR / "benchmark.py")])


def action_init_db():
    print(f"\n{WHITE}初始化資料庫{RESET}\n")
    run_script([str(SCRIPT_DIR / "manage_vip_database.py"), "init"])


def action_planned(name: str, description: str):
    clear()
    print_header()
    print(f"{YELLOW}{BOLD}  {name}  —  規劃中{RESET}\n")
    print(f"  {description}\n")
    input(f"{DIM}按 Enter 返回主選單...{RESET}")


def action_delete_vip():
    print(f"\n{WHITE}刪除 VIP{RESET}\n")
    run_script([str(SCRIPT_DIR / "manage_vip_database.py"), "delete-vip"])


def action_delete_blacklist():
    print(f"\n{WHITE}刪除黑名單{RESET}\n")
    run_script([str(SCRIPT_DIR / "manage_vip_database.py"), "delete-blacklist"])


ACTIONS = {
    "1":  action_realtime,
    "2":  action_test_recognition,
    "3":  action_add_vip,
    "4":  action_add_vip_camera,
    "5":  action_list_vips,
    "6":  action_add_blacklist,
    "7":  action_add_blacklist_camera,
    "8":  action_list_blacklist,
    "9":  action_benchmark,
    "10": action_init_db,
    "11": action_delete_vip,
    "12": action_delete_blacklist,
    "13": lambda: action_planned(
        "語音銀行助手",
        "計畫：Whisper STT (語音輸入) → 本機 LLM / Gemma API (對話) → pyttsx3 TTS (語音輸出)\n"
        "  全邊緣運算目標：Qwen2.5-1.5B INT4 在 RPi4 本機跑\n"
        "  若速度不足：LLM 在電腦端，透過 ROS 2 Topic 傳遞"
    ),
    "14": lambda: action_planned(
        "人臉數據增強",
        "計畫：電腦端從單張照片生成 30 個不同角度/光線的版本\n"
        "  工具：InsightFace augmentation (本機) 或 Stable Diffusion ControlNet (高品質)\n"
        "  目標：單樣本識別率從 40% 提升到 70-90%"
    ),
    "15": lambda: action_planned(
        "ROS 2 節點管理",
        "計畫：face_recognition_node + voice_banking_node + brain_node\n"
        "  ROS 2 消息已定義：RecognitionResult.msg, RegisterFace.srv\n"
        "  節點已實作：face_recognition_node.py, face_registration_node.py\n"
        "  待完成：smartnav_brain (決策), smartnav_audio (語音)"
    ),
}


def main():
    while True:
        clear()
        print_header()
        print_menu()

        choice = input(f"  {BOLD}請選擇 [0-15]：{RESET} ").strip()

        if choice == "0":
            clear()
            print(f"\n{CYAN}感謝使用智慧銀行 VIP 迎賓系統。{RESET}\n")
            break

        action = ACTIONS.get(choice)
        if action:
            clear()
            print_header()
            action()
        else:
            print(f"\n{RED}無效選項 '{choice}'，請輸入 0 到 15{RESET}")
            time.sleep(1)


if __name__ == "__main__":
    main()
