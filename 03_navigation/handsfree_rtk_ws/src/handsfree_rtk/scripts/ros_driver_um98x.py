#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
NMEA RTK 串口读取节点（类封装版）
- 主要功能：
  1) 从串口读取 NMEA 行
  2) 解析 GGA / RMC
  3) 发布到 ROS 话题：raw / gnss / speed / mag
- 兼容性：
  * Python2/3 兼容（线程 daemon 设置、bytes/str 解码处理）
"""

import sys
import re
import time
import threading
import serial
import rospy
from std_msgs.msg import String, Float64
from sensor_msgs.msg import NavSatFix, NavSatStatus

# ---------------------------
# 定位状态映射（UM982/通用风格）
# ---------------------------
FIX_TYPE_MAPPING = {
    0: ("Invalid Fix", NavSatStatus.STATUS_NO_FIX, 10000.0),
    1: ("GPS Fix (SPS)", NavSatStatus.STATUS_FIX, 1.0),
    2: ("DGPS Fix", NavSatStatus.STATUS_SBAS_FIX, 0.5),
    3: ("PPS Fix", NavSatStatus.STATUS_NO_FIX, 10000.0),
    4: ("RTK Fixed", NavSatStatus.STATUS_GBAS_FIX, 0.01),
    5: ("RTK Float", NavSatStatus.STATUS_GBAS_FIX, 0.1),
    6: ("Estimated", NavSatStatus.STATUS_FIX, 5.0),
    7: ("Manual", NavSatStatus.STATUS_GBAS_FIX, 0.01),
    8: ("Simulation", NavSatStatus.STATUS_FIX, 10000.0)
}

GNRMC_MODE_MAPPING = {
    'A': 'Autonomous Mode',
    'D': 'Differential Mode', 
    'E': 'INS Mode',
    'F': 'RTK Float',
    'M': 'Manual Input Mode',
    'N': 'No Fix',
    'P': 'Precision Mode',
    'R': 'RTK Fixed',
    'S': 'Simulator Mode',
    'V': 'Invalid Mode'
}

class RTKTagDriver(object):
    """NMEA RTK 节点类"""

    def __init__(self):
        # ---------------------------
        # 初始化 ROS 节点与参数
        # ---------------------------
        rospy.init_node('tag_driver', anonymous=False)

        # 串口参数（可通过 rosparam 覆盖）
        self.port = rospy.get_param("~port", "/dev/HFRobotRTK")
        self.baudrate = rospy.get_param("~baudrate", 115200)
        self.timeout = rospy.get_param("~timeout", 0.1)

        # 话题发布者
        self.raw_pub = rospy.Publisher("handsfree/rtk/raw", String, queue_size=10)
        self.gnss_pub = rospy.Publisher("handsfree/rtk/gnss", NavSatFix, queue_size=10)
        self.speed_pub = rospy.Publisher("handsfree/rtk/speed", Float64, queue_size=10)   # m/s
        self.cog_pub = rospy.Publisher("handsfree/rtk/cog", Float64, queue_size=10)       # deg (Course over ground)
        self.heading_pub = rospy.Publisher("handsfree/rtk/heading", Float64, queue_size=10)  # deg (True heading)


        # 控制线程的事件与句柄
        self._stop_event = threading.Event()
        self._read_thread = None
        self._ser = None

    # ---------------------------
    # 公共接口：启动/停止
    # ---------------------------
    def run(self):
        """运行主循环：负责打开串口、拉起读取线程、异常后重连"""
        while not rospy.is_shutdown():
            try:
                rospy.loginfo("Trying to open serial: %s @ %d", self.port, self.baudrate)
                self._ser = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout)
                rospy.loginfo("Serial opened: %s", self.port)
            except serial.SerialException as e:
                rospy.logerr("Failed to open serial: %s. Retry in 1s...", e)
                time.sleep(1.0)
                continue

            # 启动读取线程
            self._stop_event.clear()
            self._read_thread = threading.Thread(target=self._read_loop, name="nmea_reader")
            # Python2 兼容：用 setDaemon(True)
            try:
                self._read_thread.daemon = True
            except Exception:
                self._read_thread.setDaemon(True)
            self._read_thread.start()
            rospy.loginfo("NMEA reading thread started.")

            # 等待线程结束或节点关闭
            while not rospy.is_shutdown() and self._read_thread.is_alive():
                time.sleep(0.1)

            # 清理串口
            self._cleanup_serial()

            if rospy.is_shutdown():
                break

            rospy.loginfo("Reconnecting in 1s...")
            time.sleep(1.0)

    def stop(self):
        """停止读取线程与关闭串口"""
        rospy.loginfo("Stopping NMEA node...")
        self._stop_event.set()
        if self._read_thread and self._read_thread.is_alive():
            self._read_thread.join(timeout=1.0)
        self._cleanup_serial()
        rospy.loginfo("Stopped.")

    # ---------------------------
    # 内部：读取主循环
    # ---------------------------
    def _read_loop(self):
        """从串口按行读取 → 解析 → 发布 ROS 消息"""
        while not self._stop_event.is_set() and not rospy.is_shutdown():
            try:
                line = self._ser.readline()
                if not line:
                    time.sleep(0.001)  # 减少 CPU 占用
                    continue

                # Python2/3 兼容处理：bytes → str
                try:
                    if isinstance(line, bytes):
                        line = line.decode('utf-8', 'ignore')
                except Exception:
                    # 在少数情况下（特殊编码/类型），仍保持原样
                    pass

                line = line.strip()
                # 发布原始 NMEA 行
                self.raw_pub.publish(line)

                # 日志打印关键 NMEA 句子
                if line.startswith('$GNGGA') or line.startswith('$GPGGA')\
                    or line.startswith('$GNRMC') or line.startswith('$GPRMC')\
                    or line.startswith('$GNTHS') or line.startswith('$GPTHS'):
                    rospy.loginfo("%s", line)

                parsed = self._parse_nmea_sentence(line)
                if not parsed:
                    continue

                # 处理 GGA（位置/状态）
                if parsed['type'] == 'GNGGA':
                    lat = parsed.get('latitude')
                    lon = parsed.get('longitude')
                    alt = parsed.get('altitude')
                    fix_q = parsed.get('fix_quality')
                    num_sats = parsed.get('num_sats')
                    hdop = parsed.get('hdop')

                    if lat is not None and lon is not None:
                        status_str, status_value, cov_m = FIX_TYPE_MAPPING.get(
                            fix_q, ("Unknown", NavSatStatus.STATUS_NO_FIX, 10000.0))

                        rospy.loginfo("GGA: fix=%d (%s), lat=%.8f lon=%.8f alt=%.3f", fix_q, status_str, lat, lon, alt)

                        navsat = NavSatFix()
                        navsat.header.stamp = parsed['time']
                        navsat.header.frame_id = "gps"
                        navsat.status.status = status_value
                        navsat.status.service = NavSatStatus.SERVICE_GPS
                        navsat.latitude = lat
                        navsat.longitude = lon
                        navsat.altitude = alt if alt is not None else 0.0

                        var = (cov_m ** 2)  # 协方差对角（米→方差）
                        navsat.position_covariance = [
                            var, 0.0, 0.0,
                            0.0, var, 0.0,
                            0.0, 0.0, var
                        ]
                        navsat.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                        self.gnss_pub.publish(navsat)

                # 处理 RMC（地速/航向）
                elif parsed['type'] == 'GNRMC':
                    speed_mps = parsed.get('speed_mps', 0.0)
                    rmc_status = parsed.get('status', 'V')
                    cog_deg = parsed.get('cog_deg', 0.0)
                    rospy.loginfo("RMC: status=%s, speed=%.3f m/s (%.3f kn), COG=%.2f°",
                                GNRMC_MODE_MAPPING.get(rmc_status, 'Unknown'),
                                speed_mps, parsed.get('raw_speed_knots', 0.0), cog_deg)
                    if rmc_status != 'V':
                        self.speed_pub.publish(Float64(data=speed_mps))
                        self.cog_pub.publish(Float64(data=cog_deg))
                
                # 处理 THS（真航向）
                elif parsed['type'] == 'GNTHS':
                    heading = parsed.get('heading', None)
                    valid = parsed.get('valid', False)
                    if heading is not None:
                        rospy.loginfo("THS: heading=%.3f° (%s)", heading, "valid" if valid else "invalid")
                        if valid:
                            self.heading_pub.publish(Float64(data=heading))

            except serial.SerialException as e:
                rospy.logerr("Serial error: %s", e)
                self._stop_event.set()
            except Exception as e:
                rospy.logerr("Unexpected error: %s", e)
                self._stop_event.set()

    # ---------------------------
    # 内部：串口清理
    # ---------------------------
    def _cleanup_serial(self):
        """关闭串口句柄"""
        try:
            if self._ser and self._ser.isOpen():
                port_name = self._ser.port
                self._ser.close()
                rospy.loginfo("Serial closed: %s", port_name)
        except Exception as e:
            rospy.logwarn("Serial close warning: %s", e)
        finally:
            self._ser = None

    # ---------------------------
    # 静态工具：NMEA 解析/时间/坐标
    # ---------------------------
    @staticmethod
    def _parse_nmea_sentence(sentence):
        """
        解析NMEA句子：支持 GNGGA / GNRMC / GPRMC
        返回一个dict，包含 'type' 字段区分。
        """
        try:
            # ---------- GGA ----------
            if sentence.startswith('$GNGGA'):
                parts = sentence.split(',')
                if len(parts) < 10:
                    return None
                time_utc = parts[1]
                lat = RTKTagDriver._dms_to_decimal(parts[2], parts[3])
                lon = RTKTagDriver._dms_to_decimal(parts[4], parts[5])
                fix_q = int(parts[6] or 0)
                num_sats = int(parts[7] or 0)
                try:
                    hdop = float(parts[8]) if parts[8] != '' else 99.9
                except Exception:
                    hdop = 99.9
                try:
                    alt = float(parts[9]) if parts[9] != '' else 0.0
                except Exception:
                    alt = 0.0

                ros_time = RTKTagDriver._nmea_time_to_ros(time_utc)
                return {
                    'type': 'GNGGA',
                    'time': ros_time,
                    'latitude': lat,
                    'longitude': lon,
                    'altitude': alt,
                    'fix_quality': fix_q,
                    'num_sats': num_sats,
                    'hdop': hdop
                }

            # ---------- RMC ----------
            if sentence.startswith('$GNRMC') or sentence.startswith('$GPRMC'):
                # $--RMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh
                parts = sentence.split(',')
                if len(parts) < 12:
                    return None
                time_utc = parts[1]
                status = parts[2]  # 'A' 有效, 'V' 无效
                lat = RTKTagDriver._dms_to_decimal(parts[3], parts[4])
                lon = RTKTagDriver._dms_to_decimal(parts[5], parts[6])

                try:
                    speed_knots = float(parts[7]) if parts[7] != '' else 0.0
                except Exception:
                    speed_knots = 0.0

                try:
                    cog_deg = float(parts[8]) if parts[8] != '' else 0.0
                except Exception:
                    cog_deg = 0.0

                date_ddmmyy = parts[9]
                ros_time = RTKTagDriver._nmea_time_to_ros(time_utc)
                speed_mps = speed_knots * 0.514444

                return {
                    'type': 'GNRMC',
                    'time': ros_time,
                    'status': status,
                    'latitude': lat,
                    'longitude': lon,
                    'speed_mps': speed_mps,
                    'cog_deg': cog_deg,
                    'raw_speed_knots': speed_knots,
                    'date': date_ddmmyy
                }

            # ---------- THS ----------
            if sentence.startswith('$GNTHS') or sentence.startswith('GPTHS'):
                # 格式: $GNTHS,230.1081,A*10
                parts = sentence.split(',')
                if len(parts) < 3:
                    return None
                try:
                    heading = float(parts[1]) if parts[1] != '' else None
                except Exception:
                    heading = None
                valid = (parts[2].startswith('A')) if len(parts[2]) > 0 else False
                return {
                    'type': 'GNTHS',
                    'heading': heading,
                    'valid': valid
                }

            return None

        except Exception as e:
            rospy.logerr("NMEA parse error: %s", e)
            return None

    @staticmethod
    def _nmea_time_to_ros(nmea_time):
        """将 NMEA 时间(HHMMSS.SS) 转 ROS 时间戳（按当天 UTC 拼接）"""
        from datetime import datetime
        if not nmea_time:
            return rospy.Time.now()
        try:
            utc_now = datetime.utcnow()
            hours = int(nmea_time[0:2])
            minutes = int(nmea_time[2:4])
            seconds = float(nmea_time[4:]) if len(nmea_time) > 4 else 0.0
            dt = datetime(utc_now.year, utc_now.month, utc_now.day,
                          hours, minutes, int(seconds),
                          int((seconds % 1) * 1e6))
            epoch = datetime(1970, 1, 1)
            return rospy.Time.from_sec((dt - epoch).total_seconds())
        except Exception as e:
            rospy.logerr("NMEA time parse error: %s", e)
            return rospy.Time.now()

    @staticmethod
    def _dms_to_decimal(dms, direction):
        """
        度分 → 十进制度
        dms: 如 2231.86004675
        direction: N/S/E/W
        """
        try:
            if not dms:
                return None
            m = re.match(r"^(\d+)(\d\d\.\d+)$", dms)
            if not m:
                return None
            degrees = float(m.group(1))
            minutes = float(m.group(2))
            decimal = degrees + minutes / 60.0
            if direction in ('S', 'W'):
                decimal = -decimal
            return decimal
        except Exception:
            return None


# ---------------------------
# main
# ---------------------------
if __name__ == '__main__':
    try:
        node = RTKTagDriver()
        node.run()
    except KeyboardInterrupt:
        rospy.loginfo("KeyboardInterrupt received, shutting down...")
    except Exception as e:
        rospy.logerr("Fatal error: %s", e)
    finally:
        try:
            node.stop()
        except Exception:
            pass
        rospy.loginfo("Exit.")
