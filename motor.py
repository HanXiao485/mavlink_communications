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

# 修改以下连接字符串，确保与雷迅X7飞控所在网络的IP地址和端口匹配
connection_string = "udp:0.0.0.0:14550"

# 建立MAVLink连接，source_system可设置为255表示地面站
master = mavutil.mavlink_connection(connection_string, source_system=255)
print("正在连接飞控...")
master.wait_heartbeat()  # 等待心跳包，建立通信
print("已接收到心跳：系统ID %u, 组件ID %u" % (master.target_system, master.target_component))

# --------------------------------------------
# 预先发送定点消息，激活OFFBOARD模式要求
print("预先发送定点消息以激活OFFBOARD模式...")
for i in range(30):  # 约3秒，每100毫秒发送一次定点消息
    master.mav.set_actuator_control_target_send(
        int(time.time() * 1e6),  # 当前时间戳（微秒）
        0,                       # 控制组编号
        master.target_system,    # 目标系统ID
        master.target_component, # 目标组件ID
        [0.0]*8                  # 全部控制通道置0
    )
    time.sleep(0.1)

# --------------------------------------------
# 切换飞控模式至 OFFBOARD
print("切换飞控模式为 OFFBOARD...")
master.set_mode(4)  # 示例中使用数字4表示OFFBOARD模式，请参考飞控文档确认
time.sleep(1)
print("当前飞行模式：", master.flightmode)

# --------------------------------------------
# 发送Arm命令（强制解锁，绕过安全检查）
print("发送Arm命令，解锁飞机...")
master.mav.command_long_send(
    master.target_system,     # 目标系统ID
    master.target_component,   # 目标组件ID
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # 命令
    0,      # 确认标志
    1,      # param1: 1 表示Arm
    float(21196),  # param2: 魔数参数，强制Arm（绕过安全检查）
    0,      # param3
    0,      # param4
    0,      # param5
    0,      # param6
    0,      # param7
)

# 等待ARM成功（不断接收心跳更新状态）
print("等待飞机解锁（ARM）...")
while True:
    # 阻塞等待下一个HEARTBEAT消息，超时5秒
    heartbeat = master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    if heartbeat is None:
        print("未收到心跳，重试中...")
        continue
    # 判断解锁标志是否置位
    if heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        print("飞机已成功解锁。")
        break
    time.sleep(0.5)

# --------------------------------------------
# 设置目标组件ID（飞控通常使用MAV_COMP_ID_AUTOPILOT1）
master.target_component = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1

# 构造控制指令
# 对于四旋翼，假设控制组0对应电机输出；
# controls数组共有8个元素，前4个电机设置为50%油门（0.1示例中可调），其余置0。
controls = [0.1, 0.1, 0.1, 0.1] + [0.0] * 4

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
