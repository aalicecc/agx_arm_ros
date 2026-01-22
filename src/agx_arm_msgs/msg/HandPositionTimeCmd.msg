# Revo2灵巧手 位置/时间混合控制消息
# 用于position_time_ctrl()，一次性传递位置和到位时间

# 各指目标位置 [0, 100]
int8 thumb_tip_pos          # 拇指尖位置
int8 thumb_base_pos         # 拇指根位置
int8 index_finger_pos       # 食指位置
int8 middle_finger_pos      # 中指位置
int8 ring_finger_pos        # 无名指位置
int8 pinky_finger_pos       # 小指位置

# 各指到位时间 (单位10ms, 范围[0, 255], 例如200表示2秒)
uint8 thumb_tip_time        # 拇指尖到位时间
uint8 thumb_base_time       # 拇指根到位时间
uint8 index_finger_time     # 食指到位时间
uint8 middle_finger_time    # 中指到位时间
uint8 ring_finger_time      # 无名指到位时间
uint8 pinky_finger_time     # 小指到位时间

