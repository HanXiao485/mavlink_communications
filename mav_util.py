from pymavlink import mavutil
import math

class MavDecoder:
    def __init__(self):
        self.gps_fix_map = {
            0: '无信号', 
            1: '定位', 
            2: '差分', 
            3: 'RTK'
        }
    
    def decode(self, msg):
        msg_type = msg.get_type()
        
        if msg_type == 'HEARTBEAT':
            return f"心跳 | 模式: {self._flight_mode(msg)} | 状态: {self._system_status(msg)}"
            
        # elif msg_type == 'GPS_RAW_INT':
        #     return self._gps_data(msg)
            
        elif msg_type == 'ATTITUDE':
            return (f"姿态 | 横滚: {math.degrees(msg.roll):.1f}° | "
                    f"俯仰: {math.degrees(msg.pitch):.1f}° | "
                    f"偏航: {math.degrees(msg.yaw):.1f}°")
            
        # elif msg_type == 'SYS_STATUS':
        #     return (f"系统 | 电压: {msg.voltage_battery/1000:.1f}V | "
        #             f"电流: {msg.current_battery/100:.1f}A | "
        #             f"剩余电量: {msg.battery_remaining}%")
            
        elif msg_type == 'HIGHRES_IMU':
            return (f"IMU | 角速度: X={math.degrees(msg.xgyro):.2f}°/s "
                    f"Y={math.degrees(msg.ygyro):.2f}°/s "
                    f"Z={math.degrees(msg.zgyro):.2f}°/s | "
                    f"线加速度: X={msg.xacc:.2f}m/s² "
                    f"Y={msg.yacc:.2f}m/s² "
                    f"Z={msg.zacc:.2f}m/s²")
        
        elif msg_type == 'LOCAL_POSITION_NED':
            return (f"位置 | 北向: {msg.x:.2f}m "
                    f"东向: {msg.y:.2f}m "
                    f"垂直: {msg.z:.2f}m")
        
        # elif msg_type == 'SYS_STATUS':
        #     # 新增信号强度显示
        #     return (f"网络 | RSSI: {msg.rssi} dBm | "
        #             f"丢包率: {msg.packet_drop}%")
        
        # return f"未知消息类型: {msg_type}"

    def _flight_mode(self, msg):
        return mavutil.mode_string_v10(msg)
    
    def _system_status(self, msg):
        status_map = {
            0: '未初始化', 1: '启动中', 2: '待机', 
            3: '活动', 4: '关键错误', 5: '关机'
        }
        return status_map.get(msg.system_status, '未知状态')
    
    def _gps_data(self, msg):
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        return (f"GPS | 纬度: {lat:.6f}° | 经度: {lon:.6f}° | "
                f"卫星数: {msg.satellites_visible} | "
                f"定位质量: {self.gps_fix_map.get(msg.fix_type, '未知')}")