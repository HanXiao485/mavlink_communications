
from pymavlink import mavutil
import time

# 1. 建立MAVLink连接（通过串口或UDP）
# 雷迅Nora+默认串口：/dev/ttyACM0（Linux）或COM3（Windows）
connection_string = "udp:0.0.0.0:14550"
master = mavutil.mavlink_connection(connection_string, source_system=255)

# 2. 等待心跳包确认连接
print("等待飞控心跳...")
master.wait_heartbeat()
print(f"已连接飞控 [System ID: {master.target_system}, Component ID: {master.target_component}]")

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

# Create a function to send RC values
# More information about Joystick channels
# here: https://www.ardusub.com/operators-manual/rc-input-and-output.html#rc-inputs
def set_rc_channel_pwm(channel_id, pwm=1500):
    """ Set RC channel pwm value
    Args:
        channel_id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    if channel_id < 1 or channel_id > 18:
        print("Channel does not exist.")
        return

    # Mavlink 2 supports up to 18 channels:
    # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[channel_id - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system,                # target_system
        master.target_component,             # target_component
        *rc_channel_values)                  # RC channel list, in microseconds.

while True:
    # Set some throttle
    set_rc_channel_pwm(3, 1600)
    time.sleep(0.001)

    # # Set some roll
    # set_rc_channel_pwm(2, 1600)

    # # Set some pitch
    # set_rc_channel_pwm(3, 1600)

    # # Set some yaw
    # set_rc_channel_pwm(4, 1600)

    # # The camera pwm value sets the servo speed of a sweep from the current angle to
    # #  the min/max camera angle. It does not set the servo position.
    # # Set camera tilt to 45º (max) with full speed
    # set_rc_channel_pwm(8, 1900)

    # # Set channel 12 to 1500us
    # # This can be used to control a device connected to a servo output by setting the
    # # SERVO[N]_Function to RCIN12 (Where N is one of the PWM outputs)
    # set_rc_channel_pwm(12, 1500)