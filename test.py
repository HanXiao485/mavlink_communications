# 在Python终端中测试
from pymavlink.dialects.v20 import common
print(hasattr(common, 'MAV_CMD_DO_MOTOR_TEST'))  # 应输出True