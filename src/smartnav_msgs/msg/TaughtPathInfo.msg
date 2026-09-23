# 教導路徑的摘要資訊（不含逐點座標，那個太大不適合放在列表回應裡）

string path_id                  # 內部識別碼
string name                     # 使用者取的名字
string map_id                   # 綁定的地圖。地圖不同時這條路徑無意義
uint32 num_points               # 路徑點數
float32 length_m                # 路徑總長度（公尺）
uint32 num_cusps                # 折返點數量（阿克曼車多段掉頭會有）
string source                   # "teach"（開車錄的）或 "plan"（地圖點選規劃的）
string created_at               # ISO 8601
