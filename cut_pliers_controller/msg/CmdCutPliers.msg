int32 height1   # Z axis
int32 length1   # X axis
int32 height2
int32 length2
bool claw1  # 0 = close, 1 = open
bool claw2
bool enable_motor1  # arm1 Z axis
bool enable_motor2  # arm1 X axis
bool enable_motor3
bool enable_motor4
int32 target_motor  # 目标马达编号
int32 motor_value   # 马达目标值
bool enable_motor   # 启用/禁用马达
int8 mode  # 0 = 前伸, 1 = 後退
int8 speed_h1         # 新增，對應馬達1速度
int8 speed_l1         # 新增，對應馬達2速度
int8 speed_h2         # 新增，對應馬達3速度
int8 speed_l2         # 新增，對應馬達4速度