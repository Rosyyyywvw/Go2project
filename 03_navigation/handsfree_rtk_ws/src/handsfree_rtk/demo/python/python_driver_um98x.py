#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
NMEA RTK 串口读取（非 ROS 版本，类封装）
- 主要功能：
  1) 从串口读取 NMEA 行
  2) 解析 GGA / RMC / THS
  3) 控制台打印（原始行 + 结构化信息）
- 兼容性：
  * Python2/3 兼容（线程 daemon 设置、bytes/str 解码处理）
- 使用示例：
  Linux 环境下： python python_driver_um98x.py --port /dev/ttyUSB0 --baud 115200
  Windows 环境下：  python python_driver_um98x.py --port COM3 --baud 115200
"""

from __future__ import print_function
import sys
import re
import time
import threading
import signal
import argparse
from datetime import datetime
try:
    import serial
except Exception as e:
    print("ImportError: pyserial is required. Install with: pip install pyserial")
    raise

# ---------------------------
# 定位状态映射（UM982/通用风格）
# ---------------------------
FIX_TYPE_MAPPING = {
    0: ("Invalid Fix", "NO_FIX", 10000.0),
    1: ("GPS Fix (SPS)", "FIX", 1.0),
    2: ("DGPS Fix", "SBAS_FIX", 0.5),
    3: ("PPS Fix", "NO_FIX", 10000.0),
    4: ("RTK Fixed", "GBAS_FIX", 0.01),
    5: ("RTK Float", "GBAS_FIX", 0.1),
    6: ("Estimated", "FIX", 5.0),
    7: ("Manual", "GBAS_FIX", 0.01),
    8: ("Simulation", "FIX", 10000.0)
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

class RTKTagReader(object):
    """NMEA RTK 串口读取类（非 ROS）"""

    def __init__(self, port="/dev/HFRobotRTK", baudrate=115200, timeout=0.1, print_raw=True):
        # 串口参数
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.print_raw = print_raw

        # 线程控制
        self._stop_event = threading.Event()
        self._read_thread = None
        self._ser = None

    # ---------------------------
    # 公共接口：启动/停止
    # ---------------------------
    def run(self):
        """运行主循环：负责打开串口、拉起读取线程、异常后重连"""
        while not self._stop_event.is_set():
            try:
                self._log("INFO", "Trying to open serial: {} @ {}".format(self.port, self.baudrate))
                self._ser = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout)
                self._log("INFO", "Serial opened: {}".format(self.port))
            except serial.SerialException as e:
                self._log("ERROR", "Failed to open serial: {}. Retry in 1s...".format(e))
                if self._stop_event.wait(1.0):
                    break
                continue

            # 启动读取线程
            self._stop_event.clear()
            self._read_thread = threading.Thread(target=self._read_loop, name="nmea_reader")
            try:
                self._read_thread.daemon = True  # Py3
            except Exception:
                self._read_thread.setDaemon(True)  # Py2
            self._read_thread.start()
            self._log("INFO", "NMEA reading thread started.")

            # 主线程等待/监控
            while not self._stop_event.is_set() and self._read_thread.is_alive():
                time.sleep(0.1)

            # 清理串口
            self._cleanup_serial()
            if self._stop_event.is_set():
                break

            self._log("INFO", "Reconnecting in 1s...")
            if self._stop_event.wait(1.0):
                break

        self._log("INFO", "Reader stopped.")

    def stop(self):
        """停止读取线程与关闭串口"""
        self._log("INFO", "Stopping NMEA reader...")
        self._stop_event.set()
        if self._read_thread and self._read_thread.is_alive():
            self._read_thread.join(timeout=1.0)
        self._cleanup_serial()
        self._log("INFO", "Stopped.")

    # ---------------------------
    # 内部：读取主循环
    # ---------------------------
    def _read_loop(self):
        """从串口按行读取 → 解析 → 控制台打印"""
        while not self._stop_event.is_set():
            try:
                line = self._ser.readline()
                if not line:
                    time.sleep(0.001)
                    continue

                # Python2/3 兼容处理：bytes → str
                try:
                    if isinstance(line, bytes):
                        line = line.decode('utf-8', 'ignore')
                except Exception:
                    pass

                line = line.strip()

                # 打印原始 NMEA 行
                # if self.print_raw:
                #     print(line)

                # 打印关键 NMEA 句子
                if line.startswith('$GNGGA') or line.startswith('$GPGGA')\
                    or line.startswith('$GNRMC') or line.startswith('$GPRMC')\
                    or line.startswith('$GNTHS') or line.startswith('$GPTHS'):
                    self._log("INFO", line)

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
                        status_str, status_code, cov_m = FIX_TYPE_MAPPING.get(
                            fix_q, ("Unknown", "NO_FIX", 10000.0)
                        )
                        # 结构化打印一行
                        self._log(
                            "INFO",
                            "GGA: fix={:d} ({:s}), sats={}, hdop={:.2f}, lat={:.8f}, lon={:.8f}, alt={:.3f} m".format(
                                int(fix_q if fix_q is not None else -1),
                                status_str,
                                int(num_sats if num_sats is not None else -1),
                                float(hdop if hdop is not None else -1.0),
                                float(lat), float(lon), float(alt if alt is not None else 0.0)
                            )
                        )

                # 处理 RMC（地速/航向）
                elif parsed['type'] == 'GNRMC':
                    rmc_status = parsed.get('status', 'V')
                    speed_mps = parsed.get('speed_mps', 0.0)
                    cog_deg = parsed.get('cog_deg', 0.0)
                    self._log("INFO", "RMC: status={}, speed={:.3f} m/s ({:.3f} kn), COG={:.2f}°".format(
                        GNRMC_MODE_MAPPING.get(rmc_status, 'Unknown'),
                        float(speed_mps), float(parsed.get('raw_speed_knots', 0.0)), float(cog_deg)
                    ))

                elif parsed['type'] == 'GNTHS':
                    heading = parsed.get('heading', None)
                    valid = parsed.get('valid', False)
                    if heading is not None:
                        self._log("INFO", "THS: heading={:.3f}° ({})".format(
                            float(heading), "valid" if valid else "invalid"
                        ))

            except serial.SerialException as e:
                self._log("ERROR", "Serial error: {}".format(e))
                break
            except Exception as e:
                self._log("ERROR", "Unexpected error: {}".format(e))
                break

    # ---------------------------
    # 内部：串口清理
    # ---------------------------
    def _cleanup_serial(self):
        """关闭串口句柄"""
        try:
            if self._ser and self._ser.isOpen():
                port_name = self._ser.port
                self._ser.close()
                self._log("INFO", "Serial closed: {}".format(port_name))
        except Exception as e:
            self._log("WARN", "Serial close warning: {}".format(e))
        finally:
            self._ser = None

    # ---------------------------
    # 静态工具：NMEA 解析/时间/坐标
    # ---------------------------
    @staticmethod
    def _parse_nmea_sentence(sentence):
        """
        解析NMEA句子：支持 GNGGA / GNRMC
        返回一个dict，包含 'type' 字段区分。
        """
        try:
            # ---------- GGA ----------
            if sentence.startswith('$GNGGA') or sentence.startswith('$GPGGA') :
                parts = sentence.split(',')
                if len(parts) < 10:
                    return None
                time_utc = parts[1]
                lat = RTKTagReader._dms_to_decimal(parts[2], parts[3])
                lon = RTKTagReader._dms_to_decimal(parts[4], parts[5])
                try:
                    fix_q = int(parts[6] or 0)
                except Exception:
                    fix_q = 0
                try:
                    num_sats = int(parts[7] or 0)
                except Exception:
                    num_sats = 0
                try:
                    hdop = float(parts[8]) if parts[8] != '' else 99.9
                except Exception:
                    hdop = 99.9
                try:
                    alt = float(parts[9]) if parts[9] != '' else 0.0
                except Exception:
                    alt = 0.0

                ts = RTKTagReader._nmea_time_to_datetime(time_utc)
                return {
                    'type': 'GNGGA',
                    'time': ts,               # datetime(UTC)
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
                lat = RTKTagReader._dms_to_decimal(parts[3], parts[4])
                lon = RTKTagReader._dms_to_decimal(parts[5], parts[6])

                try:
                    speed_knots = float(parts[7]) if parts[7] != '' else 0.0
                except Exception:
                    speed_knots = 0.0

                try:
                    cog_deg = float(parts[8]) if parts[8] != '' else 0.0
                except Exception:
                    cog_deg = 0.0

                date_ddmmyy = parts[9]
                ts = RTKTagReader._nmea_time_to_datetime(time_utc)  # 仍按当天 UTC
                speed_mps = speed_knots * 0.514444

                return {
                    'type': 'GNRMC',
                    'time': ts,               # datetime(UTC)
                    'status': status,
                    'latitude': lat,
                    'longitude': lon,
                    'speed_mps': speed_mps,
                    'cog_deg': cog_deg,
                    'raw_speed_knots': speed_knots,
                    'date': date_ddmmyy
                }

            # ---------- THS ----------
            if sentence.startswith('$GNTHS') or sentence.startswith('$GPTHS'):
                # 典型格式: $GNTHS,230.1081,A*10
                parts = sentence.split(',')
                if len(parts) < 3:
                    return None
                try:
                    heading = float(parts[1]) if parts[1] != '' else None
                except Exception:
                    heading = None
                # 第三段含校验位，如 "A*10"；用 startswith('A') 判有效
                valid = parts[2].startswith('A') if len(parts[2]) > 0 else False
                return {
                    'type': 'GNTHS',
                    'heading': heading,
                    'valid': valid
                }

            return None

        except Exception as e:
            # 解析异常直接吞掉该行
            # 注意：串口环境可能出现半行/乱码，属于正常现象
            return None

    @staticmethod
    def _nmea_time_to_datetime(nmea_time):
        """将 NMEA 时间(HHMMSS.SS) 转为 UTC datetime（按当天日期）"""
        if not nmea_time:
            return datetime.utcnow()
        try:
            utc_now = datetime.utcnow()
            hours = int(nmea_time[0:2])
            minutes = int(nmea_time[2:4])
            seconds = float(nmea_time[4:]) if len(nmea_time) > 4 else 0.0
            dt = datetime(utc_now.year, utc_now.month, utc_now.day,
                          hours, minutes, int(seconds),
                          int((seconds % 1) * 1e6))
            return dt
        except Exception:
            return datetime.utcnow()

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

    @staticmethod
    def _log(level, msg):
        """简单日志打印（含 UTC 时间）"""
        ts = datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        try:
            print("[{}][{}] {}".format(ts, level, msg))
        except Exception:
            # 防止偶发编码问题
            sys.stdout.write("[{}][{}] {}\n".format(ts, level, msg))
            sys.stdout.flush()


# ---------------------------
# 命令行入口
# ---------------------------
def _build_argparser():
    p = argparse.ArgumentParser(description="NMEA RTK serial reader ")
    p.add_argument("--port", type=str, default="/dev/ttyUSB0", help="Serial port, e.g., /dev/ttyUSB0 or COM6")
    p.add_argument("--baud", type=int, default=115200, help="Baudrate, e.g., 115200")
    p.add_argument("--timeout", type=float, default=0.1, help="Serial timeout in seconds")
    p.add_argument("--no-raw", action="store_true", help="Do not print raw NMEA lines")
    return p


def main():
    args = _build_argparser().parse_args()
    reader = RTKTagReader(port=args.port, baudrate=args.baud, timeout=args.timeout, print_raw=(not args.no_raw))

    # Ctrl+C / SIGTERM 友好退出
    def _graceful_exit(signum, frame):
        reader.stop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, _graceful_exit)
        except Exception:
            pass

    try:
        reader.run()
    finally:
        reader.stop()


if __name__ == "__main__":
    main()
