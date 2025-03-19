#!/usr/bin/env python
"""
示例程序：通过本地电脑使用WiFi（UDP）向雷迅X7飞控发送MAVLink指令，
实现完整的起飞前解锁步骤，并控制四旋翼无人机电机转动。

程序说明：
1. 利用 pymavlink.mavutil 建立UDP连接，请根据实际情况修改连接字符串中的IP和端口。
2. 程序等待飞控心跳，确保通信建立后，预先发送定点消息以激活OFFBOARD模式。
3. 切换飞控模式至 OFFBOARD，并发送Arm命令（利用魔数绕过安全检查）。
4. 等待飞控确认解锁后，进入主循环持续发送SET_ACTUATOR_CONTROL_TARGET消息，
   保证电机控制指令持续更新。
   
参考MAVLink协议文档及pymavlink使用示例。
"""

from pymavlink import mavutil
import time
import sys

# 修改以下连接字符串，确保与雷迅X7飞控所在网络的IP地址和端口匹配
connection_string = "udp:0.0.0.0:14550"

# 建立MAVLink连接，source_system可设置为255表示地面站
master = mavutil.mavlink_connection(connection_string, source_system=255)
print("正在连接飞控...")
master.wait_heartbeat()  # 等待心跳包，建立通信
print("已接收到心跳：系统ID %u, 组件ID %u" % (master.target_system, master.target_component))

# # Choose a mode
# mode = 'OFFBOARD'

# # Check if mode is available
# if mode not in master.mode_mapping():
#     print('Unknown mode : {}'.format(mode))
#     print('Try:', list(master.mode_mapping().keys()))
#     sys.exit(1)

# # Get mode ID
# mode_id = master.mode_mapping()[mode]
# # Set new mode
# master.mav.command_long_send(
#    master.target_system, master.target_component,
#    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
#    0, mode_id, 0, 0, 0, 0, 0) 
# or:
# master.set_mode(mode_id) 
# or:
# master.mav.set_mode_send(
#     master.target_system,
#     mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
#     mode_id)

# Arm
# master.arducopter_arm() or:
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)

# wait until arming confirmed (can manually check with master.motors_armed())
print("Waiting for the vehicle to arm")
master.motors_armed_wait()
print('Armed!')

# --------------------------------------------
# 设置目标组件ID（飞控通常使用MAV_COMP_ID_AUTOPILOT1）
master.target_component = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1

# 构造控制指令
# 对于四旋翼，假设控制组0对应电机输出；
# controls数组共有8个元素，前4个电机设置为50%油门（0.1示例中可调），其余置0。
controls = [0.7, 0.7, 0.7, 0.7] + [0.0] * 4

def send_motor_control(controls_array):
    """
    发送SET_ACTUATOR_CONTROL_TARGET消息，
    参数 controls_array 为长度为8的浮点数列表，每个数值表示对应通道的输出（归一化0~1）。
    """
    time_usec = int(time.time() * 1e6)
    master.mav.set_actuator_control_target_send(
        time_usec,                # 时间戳（微秒）
        0,                        # 控制组编号
        master.target_system,     # 目标系统ID
        master.target_component,  # 目标组件ID
        controls_array            # 控制数组（8个浮点数）
    )
    print("已发送电机控制指令：", controls_array)

# 主循环：持续发送控制指令（间隔100毫秒）
try:
    while True:
        send_motor_control(controls)
        time.sleep(0.1)
except KeyboardInterrupt:
    print("程序终止。")
