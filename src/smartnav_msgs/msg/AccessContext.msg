# 目前站在機器人前面的客人，在業務規則下屬於哪一級
# 由 smartnav_brain 的 nav_arbiter 以固定頻率發布；身分過期、沒人、認不出來都是 guest。
# 只供「調整說話語氣」與「畫面顯示」使用——能不能帶位永遠由 nav_arbiter 在 RequestNavigation 時判斷。

std_msgs/Header header

string TIER_GUEST = "guest"     # 一般訪客／認不出來／身分過期
string TIER_VIP = "vip"         # VIP 或 ADMIN
string TIER_BLOCKED = "blocked" # 黑名單：不提供帶位
string tier

# 只有 vip 會帶姓名與備註；其他等級一律留空（黑名單的身分不外流給對話與畫面）
string user_name
string description
