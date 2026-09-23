"""整個 HMI 共用的常數"""

import re

from smartnav_msgs.msg import UserType

# 使用者類別清單
USER_TYPE_NAMES = {
    UserType.GUEST: "GUEST",
    UserType.VIP: "VIP",
    UserType.ADMIN: "ADMIN",
    UserType.BLACKLIST: "BLACKLIST",
}

# 健康頁的「預期節點」清單。列在這裡的節點沒啟動時會顯示成「未啟動」，
# 沒列到的有在跑會出現在「其他」。
NODE_GROUPS = [
    (
        "SmartNav",
        [
            "hmi_server_node",
            "user_auth_node",
            "face_embedding_node",
            "llm_service_node",
            "voice_trigger_node",
            "speech_recognizer_node",
            "speech_synthesizer_node",
            "audio_playback_node",
            # ★ 2026-08-10：這三個原本寫的是 map_service_node /
            #   waypoint_service_node / navigation_service_node —— 那是**舊套件**
            #   smartnav_navigation 的節點名，實機從不啟動，於是健康頁上
            #   永遠有三行紅色的「尚未啟動」，久了就被當背景雜訊忽略，
            #   真的有東西沒起來時反而看不出來。
            #   實機 nav_bringup_cc.launch.py 起的是 _cc 版：
            "map_service_cc_node",
            "waypoint_service_cc_node",
            "navigation_action_cc_node",
            "path_teach_cc_node",
            "scan_filter_cc_node",
            "steering_trim_cc_node",
            # 迎賓劇本沒起來時要看得出來——它是完整故事線的核心
            "bank_reception_node",
        ],
    ),
    ("nav2 定位", ["map_server", "amcl"]),
    (
        "nav2 導航",
        [
            "controller_server",
            "smoother_server",
            "planner_server",
            "route_server",
            "behavior_server",
            "velocity_smoother",
            "collision_monitor",
            "bt_navigator",
            "waypoint_follower",
            "docking_server",
        ],
    ),
]
OTHER_GROUP = "其他執行中的節點"

# 由其他節點內部自動建立、無法對應到單一行程的節點。
# 它們跟著父行程走，光看名字判不出在哪台機器，所以不硬猜。
INTERNAL_NODE_RE = re.compile(r"^(transform_listener_impl_|_ros2cli_daemon_|.*_rclcpp_node$)")

# 比對 speech_text 與 LLM 回覆時用來抹平標點差異。
# llm_service_node 送給 TTS 的句子已經去過標點，原文則保留標點。
NON_WORD_RE = re.compile(r"[^\w一-龥]")

# 照片註冊的上限。base64 會讓內容膨脹約 1/3，這裡限制的是解碼後的位元組數。
MAX_PHOTO_BYTES = 8 * 1024 * 1024
MAX_PHOTOS = 20

# 各動作的等待上限，比動作節點自己的逾時再多留一點餘裕
# 這裡的值只在動作伺服器整個掛掉時才會用到，避免留下永不結束的執行緒
# 這些值必須**大於**對應節點自己的逾時。小於的話 HMI 會先放棄等待
ACTION_TIMEOUTS = {
    "create_map": 480.0,  # 節點內建 400 秒
    "global_localization": 260.0,  # 節點內建 200 秒
    "navigate": 340.0,  # 節點內建 300 秒（navigation_timeout_sec）
}
