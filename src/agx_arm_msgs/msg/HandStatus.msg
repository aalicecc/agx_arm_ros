# Revo2灵巧手状态反馈消息

std_msgs/Header header

# 左右手标志：1=左手，2=右手
uint8 left_or_right

# 各指位置 [0, 100]
uint8 thumb_tip_pos         # 拇指尖位置
uint8 thumb_base_pos        # 拇指根位置
uint8 index_finger_pos      # 食指位置
uint8 middle_finger_pos     # 中指位置
uint8 ring_finger_pos       # 无名指位置
uint8 pinky_finger_pos      # 小指位置

# 各指电机状态：0=空闲，1=运行，2=堵转
uint8 thumb_tip_status      # 拇指尖电机状态
uint8 thumb_base_status     # 拇指根电机状态
uint8 index_finger_status   # 食指电机状态
uint8 middle_finger_status  # 中指电机状态
uint8 ring_finger_status    # 无名指电机状态
uint8 pinky_finger_status   # 小指电机状态

