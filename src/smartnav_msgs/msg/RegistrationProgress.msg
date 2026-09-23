# 人臉註冊進度
#
# 由 user_auth_node 在採樣過程中每收到一張樣本就發布一次。
# register_face 服務只負責「開始」，樣本是背景累積的，沒有這個話題的話
# HMI 只能輪詢 list_users 猜進度（會跳號、也看不到終止原因）。

std_msgs/Header header

string user_uuid                # 這輪註冊的使用者 UUID
string user_name                # 使用者名稱
int32 collected_samples         # 已收集樣本數
int32 target_samples            # 目標樣本數

bool active                     # 這輪是否仍在進行；false 時 status 為終態
string status                   # running / succeeded / timeout / cancelled / failed
string message                  # 給人看的說明
