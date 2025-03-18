"""
Example of how to use RC_CHANNEL_OVERRIDE messages to force input channels
in Ardupilot. These effectively replace the input channels (from joystick
or radio), NOT the output channels going to thrusters and servos.
"""

# Import mavutil
from pymavlink import mavutil

# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

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
    if channel_id < 1 or channel_id > 8:  # MAVLink 1仅支持8通道
        print("Channel ID must be 1-8 for MAVLink 1.")
        return
    rc_channel_values = [65535] * 8
    rc_channel_values[channel_id - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *rc_channel_values
    )
while True:
  # Set some roll
  set_rc_channel_pwm(2, 1600)

  # Set some yaw
  set_rc_channel_pwm(4, 1600)

  # The camera pwm value sets the servo speed of a sweep from the current angle to
  #  the min/max camera angle. It does not set the servo position.
  # Set camera tilt to 45º (max) with full speed
  set_rc_channel_pwm(8, 1900)

  # Set channel 12 to 1500us
  # This can be used to control a device connected to a servo output by setting the
  # SERVO[N]_Function to RCIN12 (Where N is one of the PWM outputs)
  set_rc_channel_pwm(1, 1500)