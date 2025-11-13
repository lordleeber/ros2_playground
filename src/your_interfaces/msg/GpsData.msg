# Header
std_msgs/Header header

# 狀態常數
int8 STATUS_NO_FIX = -1
int8 STATUS_FIX = 0

# 狀態值
int8 status          # -1: No Fix, 0: Fix

# 位置資訊
float64 latitude     # 緯度 (degrees)
float64 longitude    # 經度 (degrees)
float64 altitude     # 海拔 (meters)

# 精度與衛星數
float32 hdop         # 水平精度因子 (Horizontal Dilution of Precision)
int32 used_sats      # 用於定位的衛星數量
int32 visible_sats   # 可見衛星總數
