# 使用者身份辨識結果
# 由 user_auth_node 比對人臉向量後發布，供 HMI 顯示與下游劇本觸發使用

std_msgs/Header header

# 使用者 UUID（未認出時為空字串）
string user_uuid

# 使用者名稱（未認出時為 "Unknown"）
string user_name

# 使用者類型（GUEST / VIP / ADMIN）
UserType user_type

# 備註描述（未認出時為空字串）
string description

# 與資料庫樣本的餘弦相似度 (0.0-1.0)
float32 similarity

# 是否通過相似度門檻而認定為已知使用者
bool recognized

# 人臉邊界框 [x1, y1, x2, y2]，供 HMI 疊框顯示
float32[4] bbox
