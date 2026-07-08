# 音訊數據訊息

std_msgs/Header header          # 時間戳
uint8[] data                    # 原始音訊數據
string format                   # 編碼格式："pcm_s16le", "pcm_s32le", "pcm_f32le"
uint32 sample_rate              # 採樣率 (Hz)，例如 16000, 44100, 48000
uint8 channels                  # 聲道數：1 (Mono) 或 2 (Stereo)
bool is_final                   # 最終塊標誌：true 表示語音已結束，false 表示語音繼續
