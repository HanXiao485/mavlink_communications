"""
Example of how to use RC_CHANNEL_OVERRIDE messages to force input channels
in Ardupilot. These effectively replace the input channels (from joystick
or radio), NOT the output channels going to thrusters and servos.
"""
import time
# Import mavutil
from pymavlink import mavutil

# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# 设置飞行模式为GUIDED（关键修正点）[1](@ref)
def set_guided_mode():
    mode_id = master.mode_mapping()['GUIDED']
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    print(f"Switching to GUIDED mode (ID:{mode_id})...")
    while True:
        ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
        if ack_msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            print(f"Mode change result: {ack_msg.result}")
            break

set_guided_mode()

# 强制覆盖RC通道（防止安全机制触发）[1](@ref)
def override_rc_channels():
    """ 
    正确调用rc_channels_override_send需要同时设置所有8个通道
    未使用的通道设为0表示不覆盖
    """
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        1800,  # 通道1
        1800,  # 通道2
        1800,  # 通道3
        1800,  # 通道4
        0,     # 通道5
        0,     # 通道6
        0,     # 通道7
        0      # 通道8
    )
    print("RC channels overridden to 1500us")

override_rc_channels()

# 解锁前延迟确保模式切换完成
time.sleep(2)

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

def set_servo_pwm(servo_n, microseconds):
    """ Sets AUX 'servo_n' output PWM pulse-width.

    Uses https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_SERVO

    'servo_n' is the AUX port to set (assumes port is configured as a servo).
        Valid values are 1-3 in a normal BlueROV2 setup, but can go up to 8
        depending on Pixhawk type and firmware.
    'microseconds' is the PWM pulse-width to set the output to. Commonly
        between 1100 and 1900 microseconds.

    """
    # master.set_servo(servo_n+8, microseconds) or:
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,            # first transmission of this command
        servo_n,      # servo instance, offset by 8 MAIN outputs
        microseconds, # PWM pulse-width
        0,0,0,0,0     # unused parameters
    )

# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# # command servo_1 to go from min to max in steps of 50us, over 2 seconds
# for us in range(1100, 1900, 50):
#     set_servo_pwm(1, us)
#     time.sleep(0.125)

while True:
    set_servo_pwm(1, 1900)
    set_servo_pwm(2, 1900)
    set_servo_pwm(3, 1900)
    set_servo_pwm(4, 1900)