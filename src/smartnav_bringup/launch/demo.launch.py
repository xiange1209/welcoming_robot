#!/usr/bin/env python3
"""迎賓機器人統一啟動（★ 發表當天用這一支）

## 為什麼需要這支

在此之前要開四個終端機分別跑 `~/maprun/` 底下的腳本，順序錯了就出事。
發表當天沒有第二次機會，也沒有時間慢慢排錯。

## ★★ 啟動順序是有理由的，不要改 ★★

    感測器 ──(8s)──> 導航 ──(6s)──> 視覺/大腦 ──(4s)──> 語音 ──(3s)──> LLM/HMI

1. **感測器一定要在導航之前**。反過來會進入救不回來的 mapping 死結
   （專案硬性規則第 5 條）。8 秒是等底盤串口、光達轉速穩定、TF 樹建起來。
2. **導航要在視覺之前**：`user_auth` 啟動時會去連資料庫，而 nav2 的
   lifecycle 轉換很吃 CPU，兩個搶會讓 nav2 的 activate 逾時。
3. **LLM 最後**：它啟動時會等 `create_map` / `navigate` 兩個 action server
   （見 `llm_service_node._wait_for_services`），那兩個要 nav2 起來才有。

## 用法

    ros2 launch smartnav_bringup demo.launch.py                 # 全部
    ros2 launch smartnav_bringup demo.launch.py audio:=false    # 不要語音
    ros2 launch smartnav_bringup demo.launch.py start_mode:=mapping   # 建圖模式

★ 想單獨重啟某個節點，仍然用 `~/maprun/kill_node_cc.sh` + `run_node_cc.sh`，
  **不要**把整支 launch 殺掉重來（會連感測器一起重啟，等 8 秒）。
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _inc(pkg, rel, args=None, cond=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(pkg), rel])),
        launch_arguments=list((args or {}).items()),
        condition=cond,
    )


def _guard_duplicate_stack(context, *args, **kwargs):
    """★★ 2026-08-20：起飛前先確認沒有另一份堆疊在跑 ★★

    `~/maprun/run_nav_cc.sh` 有這個防護（`pgrep -x` + `FORCE_NAV=1` 覆寫），
    這支統一 launch 一直沒有。而它同時起十幾個節點，跟手動啟動的併存時
    症狀是**定位數據被汙染但不會報錯** —— 實測曾同時有四份 stuck_detector_cc、
    三份 scan_filter_cc，多份 scan_filter 同時發布過濾雷達讓吻合度掉到 88%。

    ★ 用 `ros2 node list` 而不是 pgrep：
      (a) 專案硬性規則禁用 `pgrep -f` / `pkill -f`（字串比對誤殺過七次）
      (b) `pgrep -x` 比對的是 comm 欄位，**Linux 上限 15 字元** ——
          `stuck_detector_cc`(17) / `controller_server`(17) 這類永遠比不中，
          等於檢查了個寂寞
      ROS 圖是這件事唯一可靠的事實來源。

    帶 `force:=true` 可以跳過（知道自己在做什麼的時候）。
    """
    if LaunchConfiguration("force").perform(context).lower() in ("true", "1"):
        return [LogInfo(msg="[bringup] ⚠ force:=true，跳過重複堆疊檢查")]

    import subprocess
    try:
        out = subprocess.run(["ros2", "node", "list"], capture_output=True,
                             text=True, timeout=8).stdout
    except Exception as exc:
        # 查不到就放行 —— 這是保護不是門禁，不該因為自己壞掉就擋住啟動
        return [LogInfo(msg=f"[bringup] 重複堆疊檢查跳過（{exc}）")]

    names = [n.strip() for n in out.splitlines() if n.strip()]

    # ★★ 2026-08-31 修正：守衛要問的是「我等一下會不會**再起一份**」，
    #    不是「這個節點在不在」。★★
    #
    #    舊版把 hmi_server 無條件列進監視名單，但 `hmi` 的預設值就是 false，
    #    理由正是「Pi 上 smartnav-hmi.service 已經跑了一份」（見下面的宣告）。
    #    於是那個設計預期的組態會自我否決：
    #      服務有跑 -> ros2 node list 看得到 hmi_server -> 守衛 raise
    #      -> **整支 demo.launch.py 拒絕啟動**，而 systemd 的
    #         Restart=on-failure 會每 10 秒重試、每次都失敗。
    #    llm_service 同理（llm:=false 時）。
    #
    #    正確判準：某個節點已經在跑，只有當**本次啟動也會起它**時才算衝突。
    watch = {
        "scan_filter_cc": "sensors",
        "stuck_detector_cc": "sensors",
        "path_teach_cc": "nav",
        "controller_server": "nav",
        "amcl": "nav",
        "hmi_server": "hmi",
        "llm_service": "llm",
    }

    def _on(arg):
        return LaunchConfiguration(arg).perform(context).lower() in ("true", "1")

    dup = sorted({w for w, arg in watch.items()
                  if _on(arg) and any(w in n for n in names)})
    if dup:
        raise RuntimeError(
            "偵測到已經有節點在跑，而本次啟動會再起一份："
            + "、".join(dup)
            + "。兩份併存會汙染定位數據而且不會報錯。"
            + " 先停乾淨：~/maprun/stop_nav_cc.sh、~/maprun/kill_sensors_cc.sh；"
            + " 或確定要併存：ros2 launch smartnav_bringup demo.launch.py force:=true")

    # ★★ 2026-08-31：反向檢查 —— 沒有 HMI 就等於**整台車啞掉** ★★
    #    車上沒有喇叭，語音輸出走的是平板瀏覽器的 speechSynthesis
    #    （/speech_text -> HMI -> 瀏覽器唸出來）。所以「hmi:=false 而且
    #    外面也沒有一份在跑」不是少一個附屬功能，是**發表當天沒有聲音、
    #    平板也沒有畫面**，而且完全不會有任何錯誤訊息。
    #    註：smartnav-hmi.service 這個檔**不在版本庫裡**，全樹只出現在
    #    下面那句註解中 —— 重灌 Pi 之後很可能就沒有了。
    msgs = [LogInfo(msg="[bringup] ✓ 重複堆疊檢查通過")]
    if not _on("hmi") and not any("hmi_server" in n for n in names):
        msgs.append(LogInfo(
            msg="[bringup] ⛔ 沒有 HMI 在跑，而 hmi:=false —— 車上沒有喇叭，"
                "語音輸出靠平板瀏覽器，這樣等於**沒有聲音也沒有畫面**。"
                "請改帶 hmi:=true，或先確認 smartnav-hmi.service 有起來。"))
    return msgs


def generate_launch_description():
    args = [
        DeclareLaunchArgument("sensors", default_value="true"),
        DeclareLaunchArgument("nav", default_value="true"),
        DeclareLaunchArgument("vision", default_value="true"),
        DeclareLaunchArgument("brain", default_value="true"),
        DeclareLaunchArgument("audio", default_value="true"),
        DeclareLaunchArgument("llm", default_value="true"),
        # ★ 2026-08-20：預設改 false。Pi 上 `smartnav-hmi.service` 已經
        #   開機自動跑 hmi_server_node 並佔住 :8080，這裡再起一份會是
        #   兩個節點搶同一個埠（第二個綁不上，平板連到的是哪一個看運氣）。
        #   確定沒裝那個 service 時才 hmi:=true。
        DeclareLaunchArgument("hmi", default_value="false"),
        # ★★ 2026-08-20：預設從 true 改成 false ★★
        #   sensors_cc.launch.py 自己的檔頭寫著：「相機預設不啟動。相機相關
        #   行程實測吃掉約 110% CPU，**是導航失敗的主因**。」
        #   統一 launch 原本設 true，等於在 0 秒就把那 110% 疊在 nav2 起機的
        #   CPU 尖峰上，是用失敗換來的教訓被一個預設值推翻。
        #   要人臉辨識時由 camera_manager_cc 按階段開，或明確帶 with_camera:=true。
        DeclareLaunchArgument("with_camera", default_value="false"),
        # ★★ 2026-08-24：把 camera_manager_cc 真的接進來 ★★
        #   在此之前它只出現在上面那行註解裡 —— 註解承諾了一個沒有被啟動的機制。
        #   實測(8/24)：車子靜止、沒開人臉沒開 ASR，CPU 已經是 382~398% / 400%，
        #   而人臉單項還要 ~190%。相機與導航**必須互斥**，這個節點就是做那件事的。
        DeclareLaunchArgument(
            "camera_manager", default_value="true",
            description="依 /navigation_active 自動開關相機（導航中關、待機開）"),
        DeclareLaunchArgument(
            "start_mode", default_value="auto",
            description="auto / mapping / localization"),
        DeclareLaunchArgument(
            "force", default_value="false",
            description="跳過重複堆疊檢查（★ 只在確定要併存時用）"),
        DeclareLaunchArgument(
            "use_exploration", default_value="false",
            description="★ 兩個探索參數在下游要一起設，這裡只暴露一個避免設出矛盾組合"),
    ]

    sensors = LaunchConfiguration("sensors")
    nav = LaunchConfiguration("nav")
    start_mode = LaunchConfiguration("start_mode")
    use_expl = LaunchConfiguration("use_exploration")

    # ── 0 s：感測器（底盤、光達、相機、TF）──
    stage_sensors = GroupAction([
        LogInfo(msg="[bringup] 1/6 感測器啟動中…（導航要等它 8 秒）"),
        _inc("smartnav_navigation_cc", "launch/sensors_cc.launch.py",
             {"with_camera": LaunchConfiguration("with_camera")},
             IfCondition(sensors)),
    ])

    # ── 8 s：導航堆疊 ──
    stage_nav = TimerAction(period=8.0, actions=[
        LogInfo(msg="[bringup] 2/6 導航堆疊啟動中…"),
        _inc("smartnav_navigation_cc", "launch/nav_bringup_cc.launch.py",
             {"use_sim_time": "false", "use_rviz": "false",
              "start_mode": start_mode,
              # ★ 這兩個一定要一起給。只給 use_exploration 的話 explorer 不啟動，
              #   但 map_service_cc 仍以為要自動探索，去呼叫不存在的服務，
              #   15 秒後 create_map 失敗而且畫面上什麼都看不到。
              "use_exploration": use_expl,
              "auto_start_exploration": use_expl},
             IfCondition(nav)),
    ])

    # ── 14 s：視覺 + 大腦 ──
    stage_brain = TimerAction(period=14.0, actions=[
        LogInfo(msg="[bringup] 3/6 人臉辨識與迎賓劇本…"),
        # ★ enable_gpu 節點端預設是 True，但 RPi4 沒有 CUDA（專案硬性規則第 5 條）。
        #   不覆寫的話 InsightFace 會先試 CUDAExecutionProvider、噴一串誤導性的
        #   CUDA 警告才退回 CPU —— 現場看到那串會以為是相機壞了。
        Node(package="smartnav_vision", executable="face_embedding",
             name="face_embedding_node", output="screen",
             parameters=[{"enable_gpu": False}],
             condition=IfCondition(LaunchConfiguration("vision"))),
        Node(package="smartnav_brain", executable="user_auth",
             name="user_auth_node", output="screen",
             condition=IfCondition(LaunchConfiguration("brain"))),
        Node(package="smartnav_brain", executable="bank_reception",
             name="bank_reception_node", output="screen",
             condition=IfCondition(LaunchConfiguration("brain"))),
    ])

    # ── 18 s：語音 ──
    # ★ voice_trigger 要先於 speech_recognizer：前者開麥克風，
    #   後者只是訂閱它發出來的音訊。反過來不會壞，只是 log 會先噴一堆等待。
    stage_audio = TimerAction(period=18.0, actions=[
        LogInfo(msg="[bringup] 4/6 語音（雙麥克風 cancel 模式）…"),
        # ★★ 2026-08-20：改成呼叫 run_asr_cc.sh，不要直接起 Node ★★
        #
        # 直接起兩個 Node 看起來乾淨，但**啟動起來是啞的**，因為
        # 語音鏈有兩件事 launch 檔天生做不到，而那兩件事都是必要的：
        #
        #   1. **裝置索引要當場問**。Astra S 的音效卡編號每次開機都會變
        #      （2026-08-14 一天內實測 1 -> 2 -> 3）。voice_trigger 的
        #      `device` 預設 -1 會抓系統預設裝置，那不是 Astra S。
        #   2. **麥克風增益要 amixer 設**，而且 'Mic',0 與 'Mic',1 兩個
        #      控制項都要設。實測 48（預設）時「VAD 一次都沒觸發過」，
        #      66 才可用 —— 這是量測校準出來的值，不是猜的。
        #
        # 這兩件 run_asr_cc.sh 已經做對了，而且它還帶了 num_threads:=2、
        # vad_num_threads:=1、mic_mode:=cancel 與係數檔。與其在 launch 裡
        # 重寫一份會走樣的版本，不如直接叫那支唯一驗證過的腳本。
        #
        # ★ 它用 nohup 背景起節點後自己結束，這在 launch 裡是正常的；
        #   nohup 不會離開 cgroup，所以 systemd 的 KillMode=control-group
        #   停機時照樣整棵殺乾淨。
        ExecuteProcess(
            cmd=["/home/user/maprun/run_asr_cc.sh"],
            output="screen",
            condition=IfCondition(LaunchConfiguration("audio"))),
    ])

    # ── 21 s：LLM + HMI ──
    stage_top = TimerAction(period=21.0, actions=[
        LogInfo(msg="[bringup] 5/6 LLM 與平板介面…（HMI 在 :8080）"),
        Node(package="smartnav_llm", executable="llm_service",
             name="llm_service_node", output="screen",
             condition=IfCondition(LaunchConfiguration("llm"))),
        _inc("smartnav_hmi", "launch/hmi.launch.py", {},
             IfCondition(LaunchConfiguration("hmi"))),
        # ★ 2026-08-20：這行原本寫「✓ 全部啟動完成」，但它只是一個
        #   到 21 秒就無條件印出的定時訊息 —— 底盤沒接、光達沒轉、
        #   ASR 啞掉，它一樣照印。8/20 的驗收若拿它當「啟動成功」，
        #   會產生假 PASS，而假 PASS 比失敗更貴。
    ])

    # ── 26 s：相機電源管理（★ 一定要排在最後）──
    #
    # 為什麼是 26 秒而不是跟著視覺階段(14 s)一起起：
    # 相機相關行程實測吃掉約 110% CPU，而 14 秒正是 nav2 lifecycle 轉換的
    # CPU 尖峰。把 `start_with_camera` 疊在那個尖峰上，等於用另一種方式
    # 重演 `with_camera` 預設 true 那個已經被推翻的錯誤。
    #
    # ★★ camera_launch_cmd 一定要指向 camera_face_only.launch.py ★★
    # 節點自己的預設是原廠的 `wheeltec_camera.launch.py`，那支會開
    # 彩色+深度+IR+點雲+d2c viewer 外加兩個 republish，實測 astra_camera_node
    # 吃到 135.9%、load average 衝到 51，人臉向量掉到 0.3 Hz
    # （見 camera_face_only.launch.py 檔頭）。人臉辨識**完全不用深度**。
    #
    # 與 with_camera 互斥：兩邊都開會有兩份 astra_camera_node 搶同一支 USB。
    stage_camera = TimerAction(period=26.0, actions=[
        LogInfo(msg="[bringup] 6/6 相機電源管理（camera_manager:=false 時這階段不起任何節點）…"),
        Node(package="smartnav_navigation_cc", executable="camera_manager_cc",
             name="camera_manager_cc_node", output="screen",
             parameters=[{
                 "camera_launch_cmd":
                     "ros2 launch /home/user/maprun/camera_face_only.launch.py",
                 "start_with_camera": True,
                 # ★★ 2026-08-27：True -> False ★★
                 #
                 # 使用者確認的需求：「導航中可能要開相機顯示在 HMI，
                 # 但是可以不用辨識。」相機關掉的話平板就沒畫面了。
                 #
                 # 而省 CPU 的目的仍然達成，只是換個地方省：
                 #     關相機（舊做法）      省 ~52%，但平板沒畫面
                 #     關人臉推論（新做法）  省 ~190%，平板照樣有畫面
                 # 貴的一直是推論不是相機。由 face_embedding_node 的
                 # `skip_while_navigating`（預設 true）負責，它同樣訂
                 # /navigation_active，政策集中在一個地方。
                 #
                 # ★ 要退回舊行為（例如 CPU 又不夠用）：把這個設回 True，
                 #   兩邊會疊加（相機關 + 推論關），但平板會失去畫面。
                 "auto_follow_navigation": False,
                 "resume_delay_sec": 3.0,
             }],
             condition=IfCondition(PythonExpression([
                 "'", LaunchConfiguration("camera_manager"), "'.lower() == 'true' and '",
                 LaunchConfiguration("with_camera"), "'.lower() != 'true'"]))),
        # ★ 2026-08-20 的教訓保留：這只是一個到時間就印的定時訊息，
        #   底盤沒接、光達沒轉、ASR 啞掉，它一樣照印。拿它當「啟動成功」
        #   會產生假 PASS，而假 PASS 比失敗更貴。
        LogInfo(msg="[bringup] 六個階段都已送出啟動指令（★ 這不代表成功）。"),
        LogInfo(msg="[bringup] 請跑 ~/maprun/nav_cc_check.py 確認實際狀態，"
                    "並用 ros2 node list 檢查有沒有重複節點。"),
    ])

    return LaunchDescription(args + [OpaqueFunction(function=_guard_duplicate_stack),
                                     stage_sensors, stage_nav,
                                     stage_brain, stage_audio, stage_top,
                                     stage_camera])
