# CircularSawState.msg
# 下位機回傳狀態（feedback）
# 位置回報（mm）
#   R_H1(Z) : -350 ~ 0 mm   (0=最高，-350=最低)
#   R_L1(X) : -400 ~ 0 mm   (0=最短，-400=最長)

int32 x_pos      # 現在 X 軸位置 (mm) <- STM32 R_L1
int32 z_pos      # 現在 Z 軸位置 (mm) <- STM32 R_H1

# 鋸片角度（deg）
# 由 STM32 回傳 Servo_Angle_RE（已經是 deg * 0.01）
float32 angle    # 現在鋸片角度 (deg)
int32 saw_speed # 鋸片轉速 (RPM)

# 馬達使能狀態
bool en1         # 馬達1 使能狀態 (R_En1 != 0)
bool en2         # 馬達2 使能狀態 (R_En2 != 0)
bool en3         # 馬達3 使能狀態 (R_En3 != 0)

# 電壓（V）
float32 voltage  # 電池電壓 (V) = STM32 Voltage / 100.0

# 簡單診斷資訊
uint32 frame_count  # 已接收到的有效 frame 計數（對應你原本的 b）
