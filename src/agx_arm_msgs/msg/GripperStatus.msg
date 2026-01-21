# AgxGripper夹爪状态反馈消息

std_msgs/Header header

# 夹爪状态
float64 width           # 当前夹爪开口宽度，单位：m
float64 force           # 当前夹持力，单位：N

# 驱动状态
bool voltage_too_low        # 电压过低
bool motor_overheating      # 电机过温
bool driver_overcurrent     # 驱动过流
bool driver_overheating     # 驱动过温
bool sensor_status          # 传感器状态
bool driver_error_status    # 驱动错误状态
bool driver_enable_status   # 驱动使能状态
bool homing_status          # 回零/零位状态

