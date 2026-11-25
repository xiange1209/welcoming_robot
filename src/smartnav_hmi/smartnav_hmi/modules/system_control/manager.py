"""子系統開關：單元表、測試情境、啟停與狀態查詢。

從 hmi_server_node.py 搬出來。只吃 node（要讀 logger、要問 /get_nav_mode
服務）與 jobs（call_service 的阻塞封裝）。唯一的 ROS 訂閱在 nav_camera.py
（★ 2026-10-28：/navigation_active → 導航中暫停人臉相機）。
"""

import importlib.util
import os
import time
from typing import Dict, Optional

from std_srvs.srv import Trigger

from ..system.procinfo import proc_cmdlines
from .nav_camera import NavCameraPause
from .cpu_share import CpuTimeSharing
from .supervisor import LaunchSupervisor, remote_units

# ★ 2026-10-22 雙板：B 板單元在 A 的面板上要看「ROS 節點圖」，不能看本機行程（A 看不到 B 的行程）。
#   名稱是 ROS 節點名（不是執行檔名）：system_control.launch.py 的 Node 以 f"{unit}_node" 命名，
#   asr_chain.launch.py 命名 voice_trigger_node／speech_recognizer_node，Astra 的節點名是 camera。
REMOTE_UNIT_NODES = {
    "camera": ("camera",),
    "face": ("face_node",),
    "user_auth": ("user_auth_node",),
    "asr_chain": ("voice_trigger_node", "speech_recognizer_node"),
}


class SystemControlManager:
    # ------------------------------------------------------------------
    # 系統節點開關
    # ------------------------------------------------------------------
    # 只允許這張表裡的項目；實際啟動固定交給 system_control.launch.py。
    # 這是刻意的：HMI 是有網頁介面的服務，如果讓它執行前端傳來的字串，
    # 等於把 shell 開放給任何拿到管理者權杖的人。
    #
    # detect  用來判斷是否在跑（比對行程指令列，不是 pgrep -f 以免誤殺）
    # 不保存前端可控制的命令列，避免管理 API 變成任意命令執行入口。
    SYSTEM_UNITS = {
        "sensors": {
            "label": "底盤 + 雷達 + IMU",
            "detect": ["lslidar_driver_node", "wheeltec_robot_node"],
            # 啟動後要靜止校準 IMU，時間較長
            "start_hint": "啟動後約 20 秒完成 IMU 零偏校準，期間車子必須靜止",
        },
        # ★★ 從「深度相機」一個按鈕改成「相機 + 兩個模式」★★
        #
        # 純彩色人臉模式與深度避障模式都由 package 化 launch 提供入口。
        #
        # 為什麼是「一個單元 + 兩個 variant」而不是兩個單元：
        # **相機只有一台**，兩種模式都起 astra_camera_node，物理上互斥。
        # 拆成兩個單元的話，起了深度模式會讓彩色模式也顯示執行中——
        # 這正是導航堆疊踩過的坑（見下方 nav 的註解），不要再踩一次。
        #
        # detect 只留 astra_camera_node（兩種模式的共同必要條件，也是唯一的相機行程）。
        # 深度模式多起的 depth_obstacle_cc 不列進 detect，否則人臉模式會被誤判成 partial。
        # 目前是哪個模式由 variant_detect 比對指令列得出，另外回報成 mode。
        "camera": {
            "label": "相機",
            "detect": ["astra_camera_node"],
            # ⚠ 換模式一定要先停：相機只有一台，而 system_control 在單元執行中時
            #   會拒絕 start（避免重複啟動）。前端在執行中也只畫「停止」鈕，
            #   所以實際操作就是「停止 → 按另一個模式」。
            "start_hint": "兩種模式互斥（只有一台相機）。要換模式先按停止，再按另一個",
            "variant_detect": {
                "face": "camera_face_cc.launch",
                "obstacle": "camera_obstacle_cc.launch",
            },
            "variants": {
                "face": {
                    "label": "人臉模式（彩色）",
                },
                "obstacle": {
                    "label": "避障模式（深度點雲）",
                },
            },
        },
        # 導航堆疊只有一個單元，但有三種啟動方式。
        #
        # 一開始寫成 nav_mapping / nav_explore / nav_localization 三個獨立單元，
        # 結果是災難：三者共用同一批行程（map_service_cc、planner_server、amcl…），
        # 只靠比對行程名稱根本分不出來，於是啟動建圖模式之後
        # 「定位＋導航」也顯示執行中、「建圖＋探索」顯示部分 —— UI 在騙人。
        #
        # 真正的模式是 map_service_cc 的內部狀態，要問 /get_nav_mode 才知道，
        # 所以改成「一個單元 + 三個啟動選項」，模式由 /api/maps/status 另外顯示。
        "nav": {
            "label": "導航堆疊",
            "detect": ["map_service_cc"],
            "start_hint": "啟動約需 60 秒。有地圖就進定位模式（可帶位），沒有就進建圖模式；"
            "要建新圖直接到「遙控建圖」分頁按開始建圖，存檔後自動回定位。★ 導航仲裁（帶位的唯一入口）跟著一起起、一起停",
            # ★ 2026-11-03：不再分「建圖」「定位 + 導航」—— 一顆啟動（start_mode=auto：有地圖就定位、沒有就建圖），
            #   建圖／定位之間由 map_service 自己切。舊腳本送的 start:mapping／explore／localization 仍然接受
            #  （system_control.launch.py 照舊對應），面板上不列。
            "legacy_variants": ("mapping", "explore", "localization"),
        },
        # ── 迎賓流程的功能模組 ──────────────────────────
        # 流程：待機（人臉辨識）-> 認出貴賓 -> LLM 對話 -> 觸發導航 -> 到達 -> 恢復辨識
        #
        # 這些單一節點由 system_control.launch.py 的 Node action 啟動。
        #
        # requires 欄位是**前置條件檢查**：與其讓操作者按了沒反應、
        # 還要自己去翻 log，不如在按鈕旁邊直接說明缺什麼。
        "face": {
            "label": "人臉辨識",
            "detect": ["face_embedding"],
            # ★ 補上 enable_gpu:=false。
            #   節點預設是 True（face_embedding_node.py 的 declare_parameter），
            #   RPi4 沒有 CUDA，會先噴一串誤導性的 CUDA 警告才退回 CPUExecutionProvider。
            #   system_control.launch.py 與 demo.launch.py 都明確覆寫為 false。
            "start_hint": "迎賓流程的觸發點。需要深度相機的彩色影像",
            "requires": {"module": "insightface"},
            # ★ 2026-10-23：省電模式（沒人時每 4 秒推論一次）。全功能同開時 CPU 閒置是 0%，
            #   人臉沒人時也要 43%（以單核計）；省電模式量到 22%，迎賓延遲中位 1.57 → 2.16 s。
            "variant_detect": {"eco": "empty_interval_sec:=4.0"},
            "variants": {
                "normal": {"label": "一般（每 2 秒）"},
                "eco": {"label": "省電（沒人時每 4 秒）"},
            },
        },
        "user_auth": {
            "label": "使用者認證 / 決策",
            "detect": ["user_auth"],
            "start_hint": "辨識到人之後決定要不要迎賓、走哪個流程",
        },
        "bank_reception": {
            "label": "迎賓劇本（銀行）",
            "detect": ["bank_reception"],
            "start_hint": "訂 /user_identity 出迎賓詞到 /speech_text；需先開「使用者認證 / 決策」",
        },
        "llm": {
            "label": "LLM 對話",
            "detect": ["llm_service"],
            "start_hint": "連遠端 Ollama（筆電，不是這台 Pi）：自動找 MSI.mshome.net／MSI.local／預設閘道／已知 IP；筆電的 Ollama 要聽區網（OLLAMA_HOST=0.0.0.0）",
        },
        # ★★ 語音喚醒與辨識必須一起運作，且由 asr_chain.launch.py 統一管理。
        #
        #    缺的三件事（實機才查出來，全部不報錯）：
        #      1. Astra S 的 ALSA 卡號**每次開機都會變**（那天一天內 1 -> 2 -> 3），
        #         裝置索引必須當場問 sounddevice，不能沿用預設
        #      2. 增益有 'Mic',0 與 'Mic',1 **兩個**控制項，只設一個沒有用；
        #         而預設的 48（24 dB）底噪 -35 dBFS、人聲埋在裡面 -> VAD 一次都沒觸發
        #      3. voice_trigger 要收到 `device:=<索引>`，否則抓到別張音效卡
        #
        #    這三件事都在 package 化的 ASR preflight 內處理。
        #
        # ★ 為什麼合併成一顆而不是修兩顆：這兩個節點**只有一起跑才有意義** ——
        #   voice_trigger 只在 VAD 判定有人講話時才送 /audio_in，
        #   speech_recognizer 收到才吐 /user_text。分開按只會製造「開了一半」的狀態。
        #   而且發表當天沒有鍵盤，**面板上一顆會動的按鈕勝過兩顆不會動的**。
        #
        # detect 兩個都列：只起來一個時前端會顯示 partial（半亮），一眼看得出不對。
        "asr_chain": {
            "label": "語音輸入（麥克風 → 文字）",
            "detect": ["voice_trigger", "speech_recognizer"],
            "requires": {"module": "sounddevice"},
            "start_hint": "麥克風長在 Astra S 相機裡；相機沒插好就沒有它。載入模型約 10~25 秒",
        },
        # ★ 這兩個是「車上出聲」用的，而**車上沒有喇叭**（決定不採購）。
        #   實際的語音輸出走平板瀏覽器的 speechSynthesis：
        #       /speech_text -> HMI -> 平板唸出來
        #   保留按鈕是因為之後若真的加了喇叭就能直接用，但要在面板上寫清楚，
        #   否則操作者會在「沒聲音」時來按這兩顆，然後以為是它們壞了。
        "speech_synthesizer": {
            "label": "語音合成（需車上喇叭）",
            "detect": ["speech_synthesizer"],
            "requires": {"audio_output": True},
            "hide_when_missing": True,
            "start_hint": "車上沒有喇叭，展示時的語音輸出是平板瀏覽器唸的，不需要開這個",
        },
        "voice_playback": {
            "label": "語音播放（需車上喇叭）",
            "detect": ["voice_playback"],
            "requires": {"module": "sounddevice", "audio_output": True},
            "hide_when_missing": True,
            "start_hint": "同上：車上沒有喇叭。要平板出聲請開右上角的朗讀開關",
        },
    }

    # ------------------------------------------------------------------
    # 測試情境
    # ------------------------------------------------------------------
    # ★ 為什麼要這張表：使用者的原話是「我無法只用那些按鈕就來執行我這個專題的測試
    #   —— 我想要測試人臉辨識與註冊，我不知道要開哪些」。
    #
    # 十個獨立開關對應的是**節點**，但人在現場想的是**任務**。
    # 兩者之間的對應關係散在程式碼與四份文件裡，而且有幾條是反直覺的：
    #   - 測人臉要關導航（skip_while_navigating 預設 true，導航中完全不推論）
    #   - 測語音要關人臉（實測 703 次 input overflow，真因是人臉吃光 CPU）
    #   - 建圖要關人臉（建圖+探索時 idle 只剩 0.4%，TF 出現空窗、目標全 abort）
    # 這些「要關什麼」比「要開什麼」更難想到，而漏關的代價是當場查不出來。
    #
    # stop 先做、start 後做，且 start 照順序（感測器要 20 秒 IMU 校準、導航要 60 秒）。
    SCENARIOS = {
        # ★ 2026-11-24 專題收尾：使用者要「一個直接把所有功能打開然後就好的按鈕」「其他不用的舊刪掉」。
        #   刪掉：測人臉辨識／註冊、教導路徑、測語音／LLM、完整迎賓故事線（人臉省電）——全開時人臉註冊
        #   （不在帶位中）、教導路徑、語音都照常能用。只留「全部開啟／建圖／全部關閉」，順序＝按鈕順序。
        #   key 沿用 "demo"：car_menu.sh、day_test.sh、hmi_unit.py 照樣能叫。
        #   已經開著（模式也對）的單元直接跳過；導航已在定位模式就不重開 —— 見 _plan_steps。
        "demo": {
            "label": "全部開啟",
            "why": "展示用：所有功能依序開好（底盤雷達 → 導航 → 相機人臉 → 認證迎賓 → 語音 → LLM），已經開著的會跳過",
            # 導航在建圖模式（或問不到模式）才先停再開：重啟後 start_mode=auto 會用最近的地圖進定位模式
            "stop": ["nav"],
            "keep_if_localization": ["nav"],
            "start": [
                "sensors",
                "nav",
                "camera:face",
                "face",
                "user_auth",
                "bank_reception",
                "asr_chain",
                "llm",
            ],
            "note": "全部起完約 90 秒，請等狀態全綠再開始。導航已經在定位模式就不重開；"
            "在建圖模式的話會先停掉、用最新的地圖重開（還沒存的圖會丟掉——建圖請先按「完成存檔」）。"
            "全開時照樣可以註冊人臉、測語音；帶位時處理器分時會自動讓導航優先。",
        },
        # ★ 2026-11-03：又合回一個「建圖」—— explorer 現在一律待命，遙控／自動探索在建圖頁選、途中可切換，
        #   下面 9/24 拆開的理由（遙控模式根本沒有 explorer）已經不存在。
        # ★ 2026-09-24：原本只有一個叫「建圖」的情境，但它起的是 nav:mapping ——
        #   run_nav_cc.sh mapping **false**，frontier explorer 根本不會啟動，
        #   按了「開始建圖」車子也不會自己動（create_map 會退回遙控建圖）。
        #   名稱看不出這件事，所以拆成兩個、名字寫清楚。
        #
        # ★ 2026-09-25：凡是指定了 nav 模式（nav:xxx）的情境，stop 清單都要有 "nav"。
        #   system_control 看到導航堆疊已經在跑就拒絕 start（防止疊兩套 Nav2），
        #   而 scenario_apply 失敗照樣往下做 —— 於是從「教導路徑」換到「建圖（遙控）」時，
        #   導航其實還停在舊模式，步驟列表只多一行「已經在執行中」，很容易看漏。
        #   先停再用正確的模式啟動，代價是每次多等約 60 秒。
        "mapping": {
            "label": "建圖",
            "why": "底盤雷達 + 導航堆疊（自動探索待命）。其餘全關，把 CPU 讓出來",
            "stop": ["face", "user_auth", "bank_reception", "llm", "asr_chain", "camera", "nav"],
            "start": ["sensors", "nav"],
            "note": "導航堆疊會先停掉再重啟（有舊地圖時會先進定位模式，按開始建圖才切到建圖，新圖從空白開始）。"
            "感測器啟動後約 20 秒 IMU 零偏校準，期間車子必須靜止；導航堆疊再約 60 秒。"
            "人臉一定要關：建圖時 idle 掉到 0.4% 會讓 TF 出現空窗。"
            "全綠後到「遙控建圖」分頁：按「開始建圖（遙控）」自己開，或「開始建圖（自動探索）」讓車子自己找路；"
            "途中可以隨時切換（例如窄走廊先遙控、進到大廳再交給自動探索，卡住了再停下來用遙控補）。"
            "⚠ 自動探索時車子會自己跑動，旁邊要有人顧著。"
            "結束按「完成存檔」（會存圖；「取消」會丟掉這張圖）。"
            "自動探索中有四種情況會自動存圖：探索完成、4 分鐘地圖沒有長大、同一處卡住 3 次、30 分鐘上限。"
            "★ 緊急狀況照樣直接按緊急停止 —— 安全優先，圖可以重建。",
        },
        # ★ 2026-10-22：展示結束／換情境前一鍵全關（以前要 SSH 進去跑停止腳本）。順序是「上層先關」：
        #   LLM／語音 → 迎賓／認證／人臉 → 相機 → 導航 → 底盤。本來就沒開的會被 scenario_request 跳過。
        "all_off": {
            "label": "全部關閉",
            "why": "展示結束、或換情境之前：把所有功能單元依序關掉。HMI 本身照常（平板不會斷線）",
            "stop": ["llm", "asr_chain", "speech_synthesizer", "voice_playback", "bank_reception", "user_auth",
                     "face", "camera", "nav", "sensors"],
            "start": [],
            "note": "本來就沒開的單元會直接跳過。要關機請用下方「電源」卡（關機前也會先做這一步）",
        },
    }


    def __init__(self, node, jobs):
        self._node = node
        self._jobs = jobs
        # Python 模組是否存在的結果會被快取：狀態頁每幾秒就打一次，
        # 而模組裝沒裝在一次執行期間不會變。
        self._requires_cache: Dict[str, str] = {}
        self._supervisor = LaunchSupervisor(lambda msg: self._node.get_logger().error(msg))
        # ★ 2026-10-28：導航中暫停人臉相機（單板全開時帶位 CPU 不夠；說明在 nav_camera.py）
        self.nav_camera = NavCameraPause(node, self._supervisor,
                                         self.SYSTEM_UNITS["camera"]["variant_detect"]["face"])
        # ★ 2026-11-24：處理器分時——帶位時導航優先、平常迎賓時語音優先（說明在 cpu_share.py）
        self.cpu_share = CpuTimeSharing(node, self._supervisor, list(self.SYSTEM_UNITS.keys()))

    def _forget_camera_if(self, units) -> None:
        """操作者自己動到相機（單元開關或情境）→ 導航中暫停相機的「自動開回來」作廢，以人為準。"""
        if any(str(u).partition(":")[0] == "camera" for u in units):
            self.nav_camera.forget()

    def _check_requires(self, requires: Optional[dict]) -> str:
        """回傳「缺什麼」的說明，都齊全就回空字串"""
        if not requires:
            return ""
        # ★ 硬體檢查。使用者要求「HMI 要自己把沒在用的設備剔除」。
        #   做成偵測而不是刪掉單元 —— 日後真的裝了喇叭，按鈕會自己回來。
        if requires.get("audio_output"):
            cached = self._requires_cache.get("__aplay__")
            if cached is None:
                try:
                    # 問核心而不是看設定檔。/proc/asound/pcm 每行結尾是
                    # "playback 1" / "capture 1"，只有播放裝置才有 playback。
                    txt = open("/proc/asound/pcm", encoding="utf-8", errors="ignore").read()
                    cached = "" if "playback" in txt else "車上沒有喇叭（找不到播放裝置）"
                except OSError:
                    cached = "車上沒有喇叭（找不到播放裝置）"
                self._requires_cache["__aplay__"] = cached
            if cached:
                return cached

        module = requires.get("module")
        if not module:
            return ""
        cached = self._requires_cache.get(module)
        if cached is None:
            try:
                cached = "" if importlib.util.find_spec(module) else f"缺少 Python 模組 {module}"
            except (ImportError, ValueError):
                cached = f"缺少 Python 模組 {module}"
            self._requires_cache[module] = cached
        return cached

    def scenario_apply(self, key: str) -> tuple:
        """套用一個測試情境：先停該停的，再依序啟動該開的。

        回傳 (成功與否, 逐步結果的文字列表)。
        ★ 任何一步失敗都**繼續往下做**並記錄 —— 半套用的狀態仍然比原地不動有用，
          而且面板上每個單元的燈號會誠實反映實際情形，操作者看得出是哪一步沒成。
        """
        spec = self.SCENARIOS.get(key)
        if spec is None:
            return False, [f"未知的情境：{key}"]

        self._forget_camera_if(list(spec["stop"]) + list(spec["start"]))
        steps = []
        ok_all = True
        for unit, action, variant in self._plan_steps(spec):
            if action == "stop":
                ok, msg = self._supervisor.run_unit_sync(unit, "stop")
                steps.append(f"{'✓' if ok else '✗'} 停止 {unit}：{msg}")
            else:
                ok, msg = self._supervisor.run_unit_sync(unit, "start", variant)
                steps.append(f"{'✓' if ok else '✗'} 啟動 {unit}{':' + variant if variant else ''}：{msg}")
            ok_all = ok_all and ok
        return ok_all, steps

    def scenario_request(self, key: str) -> tuple[bool, dict]:
        """排入情境套用，避免 HTTP request 等待每個單元的完整關閉流程。

        ★ 可能要問導航模式（最多約 6 秒）：router 用 asyncio.to_thread 呼叫，不要卡住網頁伺服器。
        """
        spec = self.SCENARIOS.get(key)
        if spec is None:
            return False, {"message": f"未知的情境：{key}"}
        self._forget_camera_if(list(spec["stop"]) + list(spec["start"]))
        return self._supervisor.request_scenario(key, self._plan_steps(spec))

    def _plan_steps(self, spec: dict) -> list:
        """情境 → 實際要做的步驟 [(unit, "stop"|"start", variant)]，停止全部排在啟動前面。

        ★ 2026-10-22：本來就沒開的單元不下「停止」——對沒在跑的單元下停止會被記成「操作失敗」
          （_stop 找不到受管 launch），「全部關閉」會滿畫面紅字。有任何行程在（partial）照樣去停。
        ★ 2026-11-24「全部開啟」：
          - 已經在跑、模式也對 → 不再下「啟動」（以前會多一行紅字「單元已由受管 launch 執行中」）；
          - 在跑但模式不對（相機在避障模式、人臉在省電模式）或只起來一半 → 先停，再用對的模式啟動；
          - keep_if_localization 的單元（導航）已在定位模式 → 不停也不重開；問不到模式就照舊先停再開。
        """
        status = {u["key"]: u for u in self.system_status()}
        live = {k for k, u in status.items() if u.get("running") or u.get("partial")}
        keep = {
            unit for unit in spec.get("keep_if_localization", ())
            if status.get(unit, {}).get("running") and self._nav_mode_safe() == "localization"
        }
        stops = [unit for unit in spec["stop"] if unit in live and unit not in keep]
        starts = []
        for item in spec["start"]:
            unit, _, variant = item.partition(":")
            st = status.get(unit, {})
            if unit not in stops and st.get("running") and self._mode_matches(unit, variant, st):
                continue                         # 已經在跑、模式也對
            if unit in live and unit not in stops:
                stops.append(unit)               # 模式不對或只起來一半 → 先停
            starts.append((unit, variant or None))
        return [(unit, "stop", None) for unit in stops] + [(unit, "start", v) for unit, v in starts]

    def _mode_matches(self, unit: str, variant: str, st: dict) -> bool:
        """system_status 回報的模式（variant_detect 認得的模式名稱，認不出＝空字串）是不是要的那個"""
        spec = self.SYSTEM_UNITS.get(unit, {})
        detect = spec.get("variant_detect") or {}
        want = spec.get("variants", {}).get(variant, {}).get("label", variant) if variant in detect else ""
        return (st.get("mode") or "") == want

    def _nav_mode_safe(self) -> str:
        try:
            return self.query_nav_mode()
        except Exception:  # noqa: BLE001 — 問不到就當 unknown：照舊先停再開（安全的那一邊）
            return "unknown"

    def operation(self, operation_id: str) -> Optional[dict]:
        return self._supervisor.operation(operation_id)

    def scenario_list(self) -> list:
        """給前端畫按鈕用。把單元 key 換成看得懂的名稱。"""
        out = []
        for key, spec in self.SCENARIOS.items():

            def _label(item):
                unit, _, variant = item.partition(":")
                u = self.SYSTEM_UNITS.get(unit, {})
                base = u.get("label", unit)
                if variant:
                    base += f"（{u.get('variants', {}).get(variant, {}).get('label', variant)}）"
                return base

            out.append(
                {
                    "key": key,
                    "label": spec["label"],
                    "why": spec["why"],
                    "note": spec.get("note", ""),
                    "start": [_label(i) for i in spec["start"]],
                    "stop": [_label(i) for i in spec["stop"]],
                }
            )
        return out

    def _graph_node_names(self) -> set:
        """ROS 節點圖上看得到的節點名（跨板；DDS 發現對方約 10 秒、對方消失約 12 秒才反映）。取不到回空集合。"""
        try:
            return {name for name, _ns in self._node.get_node_names_and_namespaces()}
        except Exception:  # noqa: BLE001
            return set()

    def system_status(self) -> list:
        """回報每個單元是否在跑"""
        procs = proc_cmdlines()
        mypid = os.getpid()
        units = []
        remote = remote_units()                     # ★ 2026-10-22 雙板：在 B 板的單元（單板＝空集合）
        graph = self._graph_node_names() if remote else set()
        for key, spec in self.SYSTEM_UNITS.items():
            # ★ 缺硬體而且標了 hide_when_missing 的單元直接不列出來。
            #   面板上一顆永遠按不動的按鈕，比沒有這顆按鈕更糟 ——
            #   操作者會在「沒聲音」時去按它，然後以為是它壞了。
            if spec.get("hide_when_missing") and self._check_requires(spec.get("requires")):
                continue
            found = []
            is_remote = key in remote
            needles = list(REMOTE_UNIT_NODES.get(key, spec["detect"])) if is_remote else spec["detect"]
            if is_remote:
                # B 板單元：看 ROS 節點圖（本機行程看不到 B）
                found = [n for n in needles if n in graph]
            else:
                for needle in spec["detect"]:
                    for pid, cmd in procs:
                        # 排除自己，也排除 shell 包裝（bash -c "..." 會含有關鍵字）
                        if pid == mypid or cmd.lstrip().startswith("/bin/bash"):
                            continue
                        if needle in cmd:
                            found.append(needle)
                            break
            # ★ 有些單元的「模式」看行程名分不出來（相機的彩色/深度
            #   都是同一個 astra_camera_node），只能比對指令列裡的 launch 檔名。
            #   不做的話面板只會說「相機在跑」，而操作者最需要知道的是**哪一種**在跑。
            mode = ""
            if not is_remote:
                for vk, needle in (spec.get("variant_detect") or {}).items():
                    # ★ 2026-10-29：HMI 自己開的單元指令列是 `system_control.launch.py unit:=X variant:=Y`，
                    #   看不到 launch 檔內部 include 的檔名／參數 → 以前相機、人臉的模式在面板上永遠是空的。兩種都認。
                    tokens = {f"unit:={key}", f"variant:={vk}"}
                    if any(needle in cmd or tokens <= set(cmd.split()) for pid, cmd in procs if pid != mypid):
                        mode = spec.get("variants", {}).get(vk, {}).get("label", vk)
                        break
            blocked = self._check_requires(spec.get("requires"))
            hint = spec.get("start_hint", "")
            if key == "camera" and not found and not is_remote and self.nav_camera.paused():
                mode = "導航中自動暫停（到點後自動恢復）"
                hint = "帶位途中把 CPU 讓給導航；導航結束約 3 秒後自動用人臉模式開回來。現在按啟動＝以你為準、不再自動開關"
            if is_remote:
                # ★ 缺什麼就先講（面板會把 blocked 顯示成原因）：B 離線時一眼看得出來，不用去翻 log
                blocked = (
                    "在 B 板（感知板）開機自動運作，A 不啟動也不停止它"
                    if found
                    else "B 板（感知板）離線或還沒就緒：看網路線、B 的 smartnav-perception 服務（DDS 發現約 10 秒）"
                )
                hint = "雙板：這個單元在 B 板；燈號＝B 的節點在 ROS 網路上看得到"
            # ★ 2026-11-03：「啟動中」＝受管 launch 還活著、節點還沒全部上線、也不在 start 操作本身（transition）裡。
            #   以前這段期間面板會跳回「啟動」鈕（transition 在 launch 行程出現 0.5 秒後就結束了），
            #   等節點上線才變「停止」。超過 start_timeout_sec 還沒全上線就不再當「啟動中」，回到
            #   原本的 partial／未執行顯示（卡住了要讓人看得出來）。
            running = len(found) == len(needles)
            transition = self._supervisor.transition(key)
            started_at = None if (is_remote or running or transition) else self._supervisor.session_started_at(key)
            starting = started_at is not None and time.time() - started_at < spec.get("start_timeout_sec", 150)
            units.append(
                {
                    "key": key,
                    "label": spec["label"],
                    "mode": mode,
                    "remote": "B" if is_remote else "",
                    "running": running,
                    "partial": 0 < len(found) < len(needles),
                    "starting": starting,
                    "starting_since": started_at if starting else None,
                    # 受管 launch 還活著但超過時限仍沒全上線：面板要標成卡住（停止鈕照樣給）
                    "start_stuck": started_at is not None and not starting,
                    "hint": hint,
                    # 缺什麼就先講，不要讓操作者按了沒反應才去翻 log
                    "blocked": blocked,
                    # 有 variants 的單元由前端畫成多顆啟動按鈕
                    "variants": [
                        {"key": vk, "label": v["label"], "warn": v.get("warn", "")}
                        for vk, v in spec.get("variants", {}).items()
                    ],
                    "transition": transition,
                }
            )
        return units

    def system_control(self, unit: str, action: str) -> tuple[bool, dict]:
        """排入啟停工作；不在 API 執行緒中等待 20 秒 graceful shutdown。"""
        spec = self.SYSTEM_UNITS.get(unit)
        if spec is None:
            return False, {"message": f"未知的單元：{unit}"}
        # ★ 2026-10-22 雙板：B 板的單元直接說明，不去檢查 A 自己有沒有 Astra／模組（那些在 B）
        if unit in remote_units():
            return False, {"message": f"（雙板）{spec['label']} 在 B 板（感知板）開機自動運作，A 的 HMI 不啟動也不停止它"}

        variant_key = None
        if action.startswith("start:"):
            action, variant_key = "start", action.split(":", 1)[1]
        if action not in ("start", "stop"):
            return False, {"message": f"未知的動作：{action}"}

        if action == "start" and spec.get("variants"):
            if variant_key is None:
                return False, {"message": f"{spec['label']} 需要指定啟動方式"}
            variant = spec["variants"].get(variant_key)
            if variant is None:
                return False, {"message": f"未知的啟動方式：{variant_key}"}
        elif variant_key is not None and variant_key not in spec.get("legacy_variants", ()):
            return False, {"message": f"{spec['label']} 不支援啟動方式 {variant_key}"}

        # 重複啟動的防呆。
        #
        # 導航堆疊要 60 秒才會有可見的變化，操作者按了沒反應很自然會再按一次，
        # 於是同時跑起兩套 Nav2 —— 兩個 controller_server、兩個 planner_server、
        # 兩個 slam_toolbox 搶同一個 /map 與 map->odom TF，比沒啟動還糟，
        # 而且外顯症狀（地圖亂跳、目標一直 abort）看起來完全不像是「按了兩次」。
        if action == "start":
            if self._check_requires(spec.get("requires")):
                return False, {"message": self._check_requires(spec.get("requires"))}
            for u in self.system_status():
                if u["key"] != unit:
                    continue
                if u["running"]:
                    return False, {"message": f"{spec['label']} 已經在執行中，不需要再啟動一次"}
                if u.get("starting"):
                    return False, {"message": f"{spec['label']} 正在啟動中（節點載入中），請稍候，不要重複按啟動"}
                if u["partial"]:
                    return False, {
                        "message": f"{spec['label']} 正在啟動或只起來一半，請先按停止再重新啟動，不要重複按啟動"
                    }
                break
        ok, operation = self._supervisor.request_unit(unit, action, variant_key)
        if ok:
            self._forget_camera_if([unit])
            what = spec["label"]
            if variant_key:
                what += f" — {spec.get('variants', {}).get(variant_key, {}).get('label', variant_key)}"
            operation["message"] = f"已排入{'啟動' if action == 'start' else '停止'} {what}"
        return ok, operation

    def query_nav_mode(self) -> str:
        """問 map_service_cc 目前是建圖還是定位模式"""
        client = self._node.service_clients.get("get_nav_mode")
        if client is None or not client.wait_for_service(timeout_sec=1.0):
            return "unknown"
        try:
            res = self._jobs.await_future(client.call_async(Trigger.Request()), timeout=5.0)
        except Exception:  # noqa: BLE001
            return "unknown"
        if not res:
            return "unknown"
        # /get_nav_mode 把模式字串放在 message 裡
        msg = (res.message or "").strip()
        for token in ("mapping", "localization", "unknown"):
            if token in msg:
                return token
        return msg or "unknown"
