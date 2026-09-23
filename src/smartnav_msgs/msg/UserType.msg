# 定義人員類型的常量
#
# BLACKLIST 是 2026-08-01 補上的。專題名稱是「VIP 迎賓與**安全通報**系統」，
# 但先前的資料模型裡根本沒有黑名單這個概念 —— 沒有它，安全通報就沒有
# 觸發條件。這不是「工具還沒寫」，是資料模型的缺口。
#
# 數值刻意接在 ADMIN 之後而不是插在中間：這是 uint8 常量，
# 改動既有值會讓已註冊的使用者型別全部錯位。
uint8 GUEST     = 0
uint8 VIP       = 1
uint8 ADMIN     = 2
uint8 BLACKLIST = 3

# 人員類型 (GUEST / VIP / ADMIN / BLACKLIST)
uint8 type
