# 位置設定（mm）
# STM32 韌體限制：
#   Height1(Z) : -350 ~ 0 mm   (0=最高，-350=最低)
#   length1(X) : -400 ~ 0 mm   (0=最短，-400=最長)

int32 x_pos     # X 軸長度 (mm) -> STM32 length1
int32 z_pos     # Z 軸高度 (mm) -> STM32 Height1

# 鋸片角度（deg）
# STM32 限制：-90 ~ 90 度
float32 angle   # 鋸片角度 (deg) -> STM32 Servo_Angle_SE

# 速度設定
# STM32 限制：
#   Move_speed_L(X) : 500 ~ 5000
#   Move_speed_H(Z) : 500 ~ 5000
#   Saw_speed       : 0 ~ 6000
int32 x_speed   # X 軸移動速度
int32 z_speed   # Z 軸移動速度
int32 saw_speed # 鋸片轉速 (RPM)

# 停止旗標
# stop = true:
#   - 上位機會送一筆「停機 frame」：Saw_speed=0, EN_Moto1~3=0
bool stop
