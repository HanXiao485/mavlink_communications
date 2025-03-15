# #!/usr/bin/env python3
# import sys
# import math
# import time
# import configparser
# from pymavlink import mavutil
# from mav_util import MavDecoder

# class X7TelemetryMonitor:
#     def __init__(self):
#         self.config = configparser.ConfigParser()
#         self.config.read('config.ini')
#         self.decoder = MavDecoder()
        
#         self.connection = mavutil.mavlink_connection(
#             self._get_port(),
#             baud=int(self.config['DEFAULT']['BAUD_RATE']),
#             dialect='common'
#         )
        
#     def _get_port(self):
#         """自动检测操作系统选择默认端口"""
#         if sys.platform.startswith('linux'):
#             return '/dev/ttyACM0'
#         elif sys.platform == 'darwin':
#             return '/dev/cu.usbmodem1'
#         else:
#             return f"COM{self.config['DEFAULT']['WIN_COM_PORT']}"

#     def start_streaming(self):
#         print(f"开始接收数据 @ {self._get_port()}")
#         print("Ctrl+C 终止程序...")
        
#         try:
#             while True:
#                 msg = self.connection.recv_match(
#                     blocking=True,
#                     timeout=3
#                 )
#                 if msg:
#                     self._process_message(msg)
#         except KeyboardInterrupt:
#             print("\n数据流已终止")
#         except Exception as e:
#             print(f"错误: {str(e)}")

#     def _process_message(self, msg):
#         decoded = self.decoder.decode(msg)
#         timestamp = time.strftime("%H:%M:%S")
#         print(f"[{timestamp}] {decoded}")

# if __name__ == "__main__":
#     monitor = X7TelemetryMonitor()
#     monitor.start_streaming()






#!/usr/bin/env python3
import sys
import time
import math
import threading
import configparser
from pymavlink import mavutil
from mav_util import MavDecoder

class X7WifiTelemetry:
    def __init__(self):
        # 配置加载
        self.config = configparser.ConfigParser()
        self.config.read('config.ini')
        
        # 连接参数
        self.target_ip = self.config['WIFI']['FC_IP']
        self.target_port = int(self.config['WIFI']['FC_PORT'])
        self.local_port = int(self.config['NETWORK']['LOCAL_PORT'])
        self.source_system = int(self.config['MAVLINK']['SOURCE_SYSTEM'])
        
        # MAVLink连接初始化
        self.conn = self._init_mavlink_connection()
        self.decoder = MavDecoder()
        
        # 启动网络监控线程
        self.net_monitor = threading.Thread(target=self._network_monitor)
        self.net_monitor.daemon = True
        self.net_monitor.start()

    def _init_mavlink_connection(self):
        """建立UDP连接并发送初始心跳包"""
        conn = mavutil.mavlink_connection(
            f"udp:{self.target_ip}:{self.target_port}",
            source_system=self.source_system,
            source_component=1,
            input=True,
            udp_timeout=5
        )
        
        # 发送初始心跳
        conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )
        return conn

    def _network_monitor(self):
        """独立线程监控网络质量"""
        while True:
            if not self._check_connection():
                print("网络中断，尝试重连...")
                self.conn = self._init_mavlink_connection()
            time.sleep(5)

    def _check_connection(self):
        """检查UDP连接状态"""
        try:
            # 发送空包检测可达性
            self.conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
            return True
        except OSError:
            return False

    def start_streaming(self):
        """主数据接收循环"""
        print(f"监听飞控数据 @ {self.target_ip}:{self.target_port}")
        print("系统ID配置:", self.source_system)
        
        try:
            while True:
                # 接收匹配关键消息类型
                msg = self.conn.recv_match(
                    type=['HEARTBEAT', 'GPS_RAW_INT', 'ATTITUDE',
                          'SYS_STATUS', 'HIGHRES_IMU', 'LOCAL_POSITION_NED'],
                    blocking=True,
                    timeout=3
                )
                
                if msg:
                    self._process_message(msg)
        except KeyboardInterrupt:
            print("\n监控已终止")
        except Exception as e:
            print(f"致命错误: {str(e)}")

    def _process_message(self, msg):
        """消息处理与显示"""
        decoded_data = self.decoder.decode(msg)
        timestamp = time.strftime("%H:%M:%S.%f")[:-3]
        print(f"[{timestamp}] {decoded_data}")

if __name__ == "__main__":
    monitor = X7WifiTelemetry()
    monitor.start_streaming()