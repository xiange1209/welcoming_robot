# 人臉識別結果
# 包含識別出的人員資訊、身份類型、性別等

# 人員UUID (若未識別則為空)
string person_uuid

# 人員名稱 (若未識別則為 "Unknown")
string person_name

# 性別 (M/F/Other，若未識別則為空)
string gender

# 人員類型 ("VIP" / "BLACKLIST" / "VISITOR")
string person_type

# 相似度信心度 (0.0-1.0)
float32 confidence

# 時間戳記
int64 timestamp

# 邊界框座標 [x1, y1, x2, y2]
float32[] bbox
