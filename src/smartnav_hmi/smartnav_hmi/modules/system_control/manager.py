"""子系統開關：單元表、測試情境、啟停與狀態查詢。

從 hmi_server_node.py 搬出來。只吃 node（要讀 logger、要問 /get_nav_mode
服務）與 jobs（call_service 的阻塞封裝），不持有任何 ROS 訂閱/發布。
"""

import importlib.util
import os
import re
import subprocess
from typing import Dict, Optional

from std_srvs.srv import Trigger

from ..system.procinfo import proc_cmdlines


class SystemControlManager:
    # ------------------------------------------------------------------
    # 系統節點開關
    # ------------------------------------------------------------------
    # 只允許這張表裡的項目，而且對應的是**既有的啟動腳本**而不是任意指令。
    # 這是刻意的：HMI 是有網頁介面的服務，如果讓它執行前端傳來的字串，
    # 等於把 shell 開放給任何拿到管理者權杖的人。
    #
    # detect  用來判斷是否在跑（比對行程指令列，不是 pgrep -f 以免誤殺）
    # start   啟動腳本；stop 停止腳本（沒有停止腳本的用 detect 找 PID 送 TERM）
    SYSTEM_UNITS = {
        "sensors": {
            "label": "底盤 + 雷達 + IMU",
            "detect": ["lslidar_driver_node", "wheeltec_robot_node"],
            "start": ["/home/user/maprun/run_sensors_cc.sh"],
            "stop": ["/home/user/maprun/kill_sensors_cc.sh"],
            # 啟動後要靜止校準 IMU，時間較長
            "start_hint": "啟動後約 20 秒完成 IMU 零偏校準，期間車子必須靜止",
        },
        # ★★ 從「深度相機」一個按鈕改成「相機 + 兩個模式」★★
        #
        # 舊版只有深度避障那一種，而 `camera_face_only.launch.py`（人臉用的純彩色）
        # **沒有任何啟動入口** —— kill_camera_cc.sh:46 一直知道怎麼殺它，
        # 但 maprun 裡沒有腳本會起它，這張表裡也沒有對應單元。
        # 後果是「想在平板上測人臉辨識」這件事從按鈕上做不到：
        # 按得到「人臉辨識」節點，卻沒有東西餵它影像。
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
            "stop": ["/home/user/maprun/kill_camera_cc.sh"],
            # ⚠ 換模式一定要先停：相機只有一台，而 system_control 在單元執行中時
            #   會拒絕 start（避免重複啟動）。前端在執行中也只畫「停止」鈕，
            #   所以實際操作就是「停止 → 按另一個模式」。
            "start_hint": "兩種模式互斥（只有一台相機）。要換模式先按停止，再按另一個",
            "variant_detect": {
                "face": "camera_face_only.launch",
                "obstacle": "camera_obstacle_cc.launch",
            },
            "variants": {
                "face": {
                    "label": "人臉模式（彩色）",
                    "cmd": ["/home/user/maprun/run_camera_face_cc.sh", "15", "85"],
                },
                "obstacle": {
                    "label": "避障模式（深度點雲）",
                    "cmd": ["/home/user/maprun/run_camera_obstacle_cc.sh", "false", "160", "120", "5.0", "2"],
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
            "stop": ["/home/user/maprun/stop_nav_cc.sh"],
            "start_hint": "啟動約需 60 秒。模式請看建圖頁的狀態列",
            "variants": {
                "mapping": {
                    "label": "建圖（遙控）",
                    "cmd": ["/home/user/maprun/run_nav_cc.sh", "mapping", "false"],
                },
                "explore": {
                    "label": "建圖 + 自動探索",
                    "cmd": ["/home/user/maprun/run_nav_cc.sh", "mapping", "true"],
                    "warn": "車子會自己跑動",
                },
                "localization": {
                    "label": "定位 + 導航",
                    "cmd": ["/home/user/maprun/run_nav_cc.sh", "localization"],
                },
            },
        },
        # ── 迎賓流程的功能模組 ──────────────────────────
        # 流程：待機（人臉辨識）-> 認出貴賓 -> LLM 對話 -> 觸發導航 -> 到達 -> 恢復辨識
        #
        # 這些都是 `ros2 run` 起的單一節點，沒有 launch 檔，
        # 所以統一透過 run_node_cc.sh（負責補上 ROS 與 DDS 環境）。
        #
        # requires 欄位是**前置條件檢查**：與其讓操作者按了沒反應、
        # 還要自己去翻 log，不如在按鈕旁邊直接說明缺什麼。
        "face": {
            "label": "人臉辨識",
            "detect": ["face_embedding"],
            # ★ 補上 enable_gpu:=false。
            #   節點預設是 True（face_embedding_node.py 的 declare_parameter），
            #   RPi4 沒有 CUDA，會先噴一串誤導性的 CUDA 警告才退回 CPUExecutionProvider。
            #   demo.launch.py 一直有覆寫，只有這條面板路徑沒有 —— 從平板起的人臉節點
            #   跟從 demo.launch.py 起的行為不一樣，而 log 看起來像壞了。
            #   ⚠ 第 3 個位置 "face" 是 **log 檔名，不可省略**：
            #     run_node_cc.sh 的 LOGNAME 吃 $3、參數從 "${@:4}" 才開始傳，
            #     漏掉的話 --ros-args 會被當成 log 檔名吃掉，參數靜默失效且不報錯。
            "start": [
                "/home/user/maprun/run_node_cc.sh",
                "smartnav_vision",
                "face_embedding",
                "face",
                "--ros-args",
                "-p",
                "enable_gpu:=false",
            ],
            "stop": ["/home/user/maprun/kill_node_cc.sh", "smartnav_vision", "face_embedding"],
            "start_hint": "迎賓流程的觸發點。需要深度相機的彩色影像",
            "requires": {"module": "insightface"},
        },
        "user_auth": {
            "label": "使用者認證 / 決策",
            "detect": ["user_auth"],
            "start": ["/home/user/maprun/run_node_cc.sh", "smartnav_brain", "user_auth"],
            "stop": ["/home/user/maprun/kill_node_cc.sh", "smartnav_brain", "user_auth"],
            "start_hint": "辨識到人之後決定要不要迎賓、走哪個流程",
        },
        "bank_reception": {
            "label": "迎賓劇本（銀行）",
            "detect": ["bank_reception"],
            "start": ["/home/user/maprun/run_node_cc.sh", "smartnav_brain", "bank_reception"],
            "stop": ["/home/user/maprun/kill_node_cc.sh", "smartnav_brain", "bank_reception"],
            "start_hint": "訂 /user_identity 出迎賓詞到 /speech_text；需先開「使用者認證 / 決策」",
        },
        "llm": {
            "label": "LLM 對話",
            "detect": ["llm_service"],
            "start": ["/home/user/maprun/run_node_cc.sh", "smartnav_llm", "llm_service"],
            "stop": ["/home/user/maprun/kill_node_cc.sh", "smartnav_llm", "llm_service"],
            "start_hint": "連遠端 Ollama（預設 192.168.137.1:11434 = 筆電），不是跑在這台 Pi 上",
        },
        # ★★ 原本這裡是「語音喚醒」與「語音辨識」兩顆分開的按鈕，
        #    兩顆都走 run_node_cc.sh —— 而那條路**啟動出來的 ASR 收不到人聲**。
        #
        #    缺的三件事（實機才查出來，全部不報錯）：
        #      1. Astra S 的 ALSA 卡號**每次開機都會變**（那天一天內 1 -> 2 -> 3），
        #         裝置索引必須當場問 sounddevice，不能沿用預設
        #      2. 增益有 'Mic',0 與 'Mic',1 **兩個**控制項，只設一個沒有用；
        #         而預設的 48（24 dB）底噪 -35 dBFS、人聲埋在裡面 -> VAD 一次都沒觸發
        #      3. voice_trigger 要收到 `device:=<索引>`，否則抓到別張音效卡
        #
        #    這三件事都在 `~/maprun/run_asr_cc.sh` 裡（實測校準過，增益 66 = 33 dB）。
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
            "start": ["/home/user/maprun/run_asr_cc.sh"],
            "stop": ["/home/user/maprun/run_asr_cc.sh", "stop"],
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
            "start": ["/home/user/maprun/run_node_cc.sh", "smartnav_audio", "speech_synthesizer"],
            "stop": ["/home/user/maprun/kill_node_cc.sh", "smartnav_audio", "speech_synthesizer"],
            "requires": {"audio_output": True},
            "hide_when_missing": True,
            "start_hint": "車上沒有喇叭，展示時的語音輸出是平板瀏覽器唸的，不需要開這個",
        },
        "voice_playback": {
            "label": "語音播放（需車上喇叭）",
            "detect": ["voice_playback"],
            "start": ["/home/user/maprun/run_node_cc.sh", "smartnav_audio", "voice_playback"],
            "stop": ["/home/user/maprun/kill_node_cc.sh", "smartnav_audio", "voice_playback"],
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
        "face": {
            "label": "測人臉辨識 / 註冊",
            "why": "只開彩色相機 + 人臉 + 認證。註冊與辨識都靠同一條鏈",
            "stop": ["nav", "asr_chain"],
            "start": ["camera:face", "face", "user_auth"],
            "note": "不需要底盤與雷達。導航一定要關 —— 導航中人臉節點完全不推論，"
            "註冊採樣會逾時然後把剛註冊的人回滾刪掉",
        },
        "mapping": {
            "label": "建圖",
            "why": "底盤雷達 + 導航堆疊（建圖模式）。其餘全關，把 CPU 讓出來",
            "stop": ["face", "user_auth", "bank_reception", "llm", "asr_chain", "camera"],
            "start": ["sensors", "nav:mapping"],
            "note": "感測器啟動後約 20 秒 IMU 零偏校準，期間車子必須靜止；"
            "導航堆疊再約 60 秒。人臉一定要關：建圖時 idle 掉到 0.4% 會讓 TF 出現空窗",
        },
        "teach": {
            "label": "教導路徑 錄製 / 重播",
            "why": "底盤雷達 + 導航堆疊（定位模式）。path_teach 隨導航堆疊自動起",
            "stop": ["face", "llm", "asr_chain", "camera"],
            "start": ["sensors", "nav:localization"],
            "note": "需要 map→base_footprint 的 TF，所以導航堆疊一定要起。"
            "換過地圖的話舊路徑會被擋下來，那是正確行為不是壞掉",
        },
        "voice": {
            "label": "測語音 / LLM 對話",
            "why": "語音輸入 + LLM。出聲走平板瀏覽器，車上沒喇叭",
            "stop": ["face", "nav", "camera"],
            "start": ["asr_chain", "llm"],
            "note": "★ 人臉一定要關：實測音訊破碎 703 次，直接死因就是人臉推論"
            "吃光 CPU、音訊執行緒排不進去。另外平板要先在畫面上點一下解鎖朗讀",
        },
        "demo": {
            "label": "完整迎賓故事線",
            "why": "全鏈路。順序照 demo.launch.py 的分階段：感測 → 導航 → 視覺 → 語音 → LLM",
            "stop": [],
            "start": [
                "sensors",
                "nav:localization",
                "camera:face",
                "face",
                "user_auth",
                "bank_reception",
                "asr_chain",
                "llm",
            ],
            "note": "⚠ 這是唯一近乎全開的情境，CPU 帳已逼近 400%。全部起完約需 90 秒，請等狀態全綠再開始演",
        },
    }

    # ------------------------------------------------------------------
    # 一鍵驗證（2026-09-21 新增，2026-09-24 搬進模組化結構）
    # ------------------------------------------------------------------
    # 跟上面的「測試情境」分開是刻意的：情境是「把東西開起來」，
    # 驗證是「證明它真的在做事」。9/17 的教訓就是節點全亮綠燈、話題也都在，
    # 但 ASR 從沒建立過辨識器、相機降幀從未生效。
    #
    # runnable=False 的兩支要讀鍵盤輸入（捲尺／量角器量到的值），
    # 網頁沒有 stdin，所以只回傳指令讓人去終端機貼，不假裝能跑。
    VERIFY_SCRIPTS = {
        "v0": {
            "label": "V0 開工前檢查",
            "why": "電池、裝置、時鐘、目錄結構，以及各項修正有沒有部署到車上",
            "script": "/home/user/maprun/verify/verify_0_preflight.sh",
            "runnable": True,
            "note": "這關沒過就別往下做，後面量到的數字都不可信",
        },
        "v1": {
            "label": "V1 修正生效驗證",
            "why": "量相機實際幀率、人臉 CPU、模型載入、麥克風增益",
            "script": "/home/user/maprun/verify/verify_1_fixes.sh",
            "runnable": True,
            "note": "★ 先套用「測人臉辨識 / 註冊」情境把相機與人臉開起來，再按這個。"
            "ros2 param get 不算驗證，這支量的是實際輸出",
        },
        "v2": {
            "label": "V2 建圖後記錄",
            "why": "確認地圖存了，並記下門口／走廊／最窄處三個淨寬",
            "script": "/home/user/maprun/verify/verify_2_map.sh",
            "runnable": False,
            "note": "要輸入捲尺量到的寬度，請在終端機跑",
        },
        "v3": {
            "label": "V3 舵機量測",
            "why": "前輪離地打滿舵，量實際角度（是量測，不是校準）",
            "script": "/home/user/maprun/verify/verify_3_steer.sh",
            "runnable": False,
            "note": "要輸入量角器讀數，請在終端機跑",
        },
        "v4": {
            "label": "V4 循跡誤差統計",
            "why": "當場算出今天重現幾趟的 95 百分位，不必等回電腦",
            "script": "/home/user/maprun/verify/verify_4_replay.sh",
            "runnable": True,
            "note": "教導錄一條、重現 4 趟之後再按",
        },
        "collect": {
            "label": "打包回傳",
            "why": "把驗證結果與原始數據打包，並印出 scp 指令",
            "script": "/home/user/maprun/verify/collect_data.sh",
            "runnable": True,
            "note": "★ 已排除 secrets 與 face_database，打包前會自我複查",
        },
    }

    def __init__(self, node, jobs):
        self._node = node
        self._jobs = jobs
        # Python 模組是否存在的結果會被快取：狀態頁每幾秒就打一次，
        # 而模組裝沒裝在一次執行期間不會變。
        self._requires_cache: Dict[str, str] = {}

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

        steps = []
        ok_all = True
        for unit in spec["stop"]:
            ok, msg = self.system_control(unit, "stop")
            steps.append(f"{'✓' if ok else '✗'} 停止 {unit}：{msg}")
            ok_all = ok_all and ok
        for item in spec["start"]:
            unit, _, variant = item.partition(":")
            action = f"start:{variant}" if variant else "start"
            ok, msg = self.system_control(unit, action)
            steps.append(f"{'✓' if ok else '✗'} 啟動 {item}：{msg}")
            ok_all = ok_all and ok
        return ok_all, steps

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

    def verify_run(self, key: str) -> tuple:
        """跑一支驗證腳本，把畫面輸出原封不動回傳給前端。

        ★ 會阻塞到腳本結束（V1 光量幀率就要 12 秒），router 一定要丟執行緒。
        """
        spec = self.VERIFY_SCRIPTS.get(key)
        if spec is None:
            return False, f"沒有這個驗證項目：{key}"
        if not spec.get("runnable", True):
            return False, f"這支要讀鍵盤輸入，網頁跑不了。請在終端機執行：\n  {spec['script']}"
        try:
            # 逾時放寬到 300 秒：V1 量幀率 12 秒，V4 要掃當天全部 CSV
            r = subprocess.run(
                [spec["script"]],
                capture_output=True,
                text=True,
                timeout=300,
                env={**os.environ, "TERM": "dumb"},
            )
            out = (r.stdout or "") + (("\n" + r.stderr) if r.stderr else "")
            # 去掉 ANSI 色碼，平板上看才不會一堆亂碼
            out = re.sub(r"\x1b\[[0-9;]*m", "", out)
            return r.returncode == 0, out.strip() or "(沒有輸出)"
        except subprocess.TimeoutExpired:
            return False, "逾時（300 秒）—— 腳本可能卡在等輸入，改到終端機跑"
        except FileNotFoundError:
            return False, (
                f"找不到腳本 {spec['script']} —— 車上還沒有這批檔案，需要先把 maprun 解壓上去"
            )
        except PermissionError:
            return False, f"腳本沒有執行權限：chmod +x {spec['script']}"
        except Exception as exc:  # noqa: BLE001
            return False, f"執行失敗：{exc}"

    def verify_list(self) -> list:
        """給前端畫按鈕用"""
        return [
            {
                "key": k,
                "label": v["label"],
                "why": v["why"],
                "runnable": v.get("runnable", True),
                "script": v["script"],
                "note": v.get("note", ""),
            }
            for k, v in self.VERIFY_SCRIPTS.items()
        ]

    def system_status(self) -> list:
        """回報每個單元是否在跑"""
        procs = proc_cmdlines()
        mypid = os.getpid()
        units = []
        for key, spec in self.SYSTEM_UNITS.items():
            # ★ 缺硬體而且標了 hide_when_missing 的單元直接不列出來。
            #   面板上一顆永遠按不動的按鈕，比沒有這顆按鈕更糟 ——
            #   操作者會在「沒聲音」時去按它，然後以為是它壞了。
            if spec.get("hide_when_missing") and self._check_requires(spec.get("requires")):
                continue
            found = []
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
            for vk, needle in (spec.get("variant_detect") or {}).items():
                if any(needle in cmd for pid, cmd in procs if pid != mypid):
                    mode = spec.get("variants", {}).get(vk, {}).get("label", vk)
                    break
            units.append(
                {
                    "key": key,
                    "label": spec["label"],
                    "mode": mode,
                    "running": len(found) == len(spec["detect"]),
                    "partial": 0 < len(found) < len(spec["detect"]),
                    "hint": spec.get("start_hint", ""),
                    # 缺什麼就先講，不要讓操作者按了沒反應才去翻 log
                    "blocked": self._check_requires(spec.get("requires")),
                    # 有 variants 的單元由前端畫成多顆啟動按鈕
                    "variants": [
                        {"key": vk, "label": v["label"], "warn": v.get("warn", "")}
                        for vk, v in spec.get("variants", {}).items()
                    ],
                }
            )
        return units

    def system_control(self, unit: str, action: str) -> tuple:
        """啟動或停止一個單元

        action 是 "stop"，或 "start"（單一啟動方式）／"start:<variant>"（多選一）。
        """
        logger = self._node.get_logger()
        spec = self.SYSTEM_UNITS.get(unit)
        if spec is None:
            return False, f"未知的單元：{unit}"

        variant_key = None
        if action.startswith("start:"):
            action, variant_key = "start", action.split(":", 1)[1]
        if action not in ("start", "stop"):
            return False, f"未知的動作：{action}"

        if action == "start" and spec.get("variants"):
            if variant_key is None:
                return False, f"{spec['label']} 需要指定啟動方式"
            variant = spec["variants"].get(variant_key)
            if variant is None:
                return False, f"未知的啟動方式：{variant_key}"
            cmd = variant["cmd"]
        else:
            cmd = spec.get(action)
        if not cmd:
            return False, f"{spec['label']} 不支援 {action}"

        # 重複啟動的防呆。
        #
        # 導航堆疊要 60 秒才會有可見的變化，操作者按了沒反應很自然會再按一次，
        # 於是同時跑起兩套 Nav2 —— 兩個 controller_server、兩個 planner_server、
        # 兩個 slam_toolbox 搶同一個 /map 與 map->odom TF，比沒啟動還糟，
        # 而且外顯症狀（地圖亂跳、目標一直 abort）看起來完全不像是「按了兩次」。
        if action == "start":
            for u in self.system_status():
                if u["key"] != unit:
                    continue
                if u["running"]:
                    return False, f"{spec['label']} 已經在執行中，不需要再啟動一次"
                if u["partial"]:
                    return False, (f"{spec['label']} 正在啟動或只起來一半，" "請先按停止再重新啟動，不要重複按啟動")
                break

        try:
            if action == "start":
                # setsid + 完全脫離：HMI 服務重啟時不能把這些節點一起帶走
                subprocess.Popen(
                    cmd,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    stdin=subprocess.DEVNULL,
                    start_new_session=True,
                )
                what = spec["label"]
                if variant_key:
                    what += f" — {spec['variants'][variant_key]['label']}"
                hint = spec.get("start_hint", "")
                return True, f"已啟動 {what}" + (f"（{hint}）" if hint else "")
            # stop 要等它跑完才知道結果
            res = subprocess.run(cmd, capture_output=True, text=True, timeout=60, stdin=subprocess.DEVNULL)
            tail = (res.stdout or res.stderr or "").strip().splitlines()
            return True, f"已停止 {spec['label']}" + (f"：{tail[-1]}" if tail else "")
        except subprocess.TimeoutExpired:
            return False, f"{action} {spec['label']} 逾時"
        except Exception as exc:  # noqa: BLE001
            logger.error(f"system_control({unit},{action}) 失敗: {exc}")
            return False, f"執行失敗：{exc}"

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
