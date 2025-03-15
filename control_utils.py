from pymavlink import mavutil
import time

class MotorController:
    def __init__(self, connection, max_rpm=2000):
        self.conn = connection
        self.max_rpm = max_rpm
        self._setup_motor_channels()

    def _setup_motor_channels(self):
        """配置电机通道映射（适用于四旋翼）"""
        self.motor_map = {
            0: 0,  # param1: 电机1
            1: 1,  # param2: 电机2
            2: 2,  # param3: 电机3
            3: 3   # param4: 电机4
        }

    def _normalize_rpm(self, rpm):
        """将RPM转换为标准化输出值（0-1）"""
        return max(0.0, min(1.0, rpm / self.max_rpm))

    def set_motors(self, m1, m2, m3, m4):
        """设置四个电机的转速"""
        # 转换转速值
        outputs = [
            self._normalize_rpm(m1),
            self._normalize_rpm(m2),
            self._normalize_rpm(m3),
            self._normalize_rpm(m4)
        ]
        
        # 构造MAVLink消息
        self.conn.mav.command_long_send(
            target_system=1,
            target_component=mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
            command=mavutil.mavlink.MAV_CMD_DO_SET_ACTUATOR,
            confirmation=0,
            param1=outputs[0],  # 电机1
            param2=outputs[1],  # 电机2
            param3=outputs[2],  # 电机3
            param4=outputs[3],  # 电机4
            param5=0, param6=0, param7=0
        )
        print(f"电机输出设置：{m1}/{m2}/{m3}/{m4} RPM")