#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
使用方式( --print-raw 代表是否打印原始GNGGA和GNRMC数据，加上之后输出原始数据)
Linux 使用方式：
python python_driver_um98x_ntrip.py \
  --port /dev/ttyUSB0 \
  --baudrate 115200 \
  --server 120.253.239.161 --server-port 8002 \
  --username "$NTRIP_USERNAME" --password "$NTRIP_PASSWORD" --mount "$NTRIP_MOUNTPOINT" \
  --print-raw
 Windows 使用方式：
 将 Linux 使用方式中的--port 的值改为COMX，具体是COM几，需要用户自行查看
'''

from __future__ import print_function
import sys
import os
import time
import socket
import base64
import serial
import threading
import select
import errno
import argparse

# ---------- ANSI 高亮（可关） ----------
ANSI_ENABLE = sys.stdout.isatty() and os.name != 'nt'
BOLD = "\033[1m" if ANSI_ENABLE else ""
YEL = "\033[33m" if ANSI_ENABLE else ""
RED = "\033[31m" if ANSI_ENABLE else ""
GRN = "\033[32m" if ANSI_ENABLE else ""
RST = "\033[0m" if ANSI_ENABLE else ""

def log_info(msg):
    sys.stdout.write("%s[INFO]%s %s\n" % (GRN, RST, msg)); sys.stdout.flush()

def log_warn(msg):
    sys.stdout.write("%s[WARN]%s %s\n" % (YEL, RST, msg)); sys.stdout.flush()

def log_err(msg):
    sys.stderr.write("%s[ERR ]%s %s\n" % (RED, RST, msg)); sys.stderr.flush()


class NtripGnssApp(object):
    def __init__(self, args):
        self.args = args

        self.port = args.port
        self.baudrate = args.baudrate
        self.serial_timeout = args.serial_timeout
        self.server_host = args.server
        self.server_port = args.server_port
        self.ntrip_user = args.username
        self.ntrip_pass = args.password
        self.mountpoint = args.mount
        self.gga_period = max(0.2, args.gga_period)
        self.print_raw = args.print_raw
        
        self.gngga_fix_mapping = {
            0: "Invalid Fix",
            1: "GPS Fix (SPS)",
            2: "DGPS Fix",
            3: "PPS Fix",
            4: "RTK Fixed",
            5: "RTK Float",
            6: "Estimated",
            7: "Manual",
            8: "Simulation",
        }

        self.gnrmc_status_mapping = {
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

        if args.no_color:
            global ANSI_ENABLE, BOLD, YEL, RED, GRN, RST
            ANSI_ENABLE = False
            BOLD = YEL = RED = GRN = RST = ""

        # 运行状态
        self.serial_port = None
        self.sock = None
        self.latest_gga = None
        self.have_gga = False

        # 退出控制
        self.stop_evt = threading.Event()

        # 线程
        self.gga_thread = None
        self.rtcm_thread = None

    # ---------- 主流程 ----------
    def run(self):
        self._open_serial_blocking()
        self._wait_first_gga()
        self._connect_ntrip()

        # 启动后台线程（Py2/3 通用写法）
        self.gga_thread = self._spawn_thread(self._loop_send_gga, "gga_tx")
        self.rtcm_thread = self._spawn_thread(self._loop_recv_rtcm, "rtcm_rx")

        # 主循环：读串口 → 解析 → 打印
        self._read_serial_loop()
        self._close_all()

    def _spawn_thread(self, target, name):
        t = threading.Thread(target=target, name=name)
        try:
            t.daemon = True
        except Exception:
            t.setDaemon(True)
        t.start()
        return t

    # ---------- 串口 ----------
    def _open_serial(self):
        try:
            self.serial_port = serial.Serial(self.port, self.baudrate, timeout=self.serial_timeout)
            log_info("Opened serial %s @ %d" % (self.port, self.baudrate))
        except Exception as e:
            log_err("Open serial failed: %s" % e)
            raise
    
    def _open_serial_blocking(self):
        """
        启动期阻塞式打开串口：无限重试直到成功或 stop_evt 触发。
        - 不依赖 rospy；适用于 Python2/3
        - 使用指数退避：0.5s 起、每次×1.5、封顶 5s（重试次数不封顶）
        """
        backoff = 0.5
        while not self.stop_evt.is_set():
            try:
                # 注意：若端口曾被打开过，这里先确保关闭
                try:
                    if self.serial_port and getattr(self.serial_port, "is_open", False):
                        self.serial_port.close()
                except Exception:
                    pass

                self.serial_port = serial.Serial(self.port, self.baudrate, timeout=self.serial_timeout)
                log_info("Opened serial %s @ %d (initial blocking open)" % (self.port, self.baudrate))
                return
            except Exception as e:
                log_warn("Open serial failed (will retry): %s" % e)
                time.sleep(backoff)
                backoff = 5.0 if backoff >= 5.0 else backoff * 1.5

        # 外部调用了 stop()，主动终止启动等待
        raise KeyboardInterrupt("Stopped before serial could be opened.")


    def _readline(self):
        if not self.serial_port:
            return None
        raw = self.serial_port.readline()
        if not raw:
            return None
        try:
            return raw.decode('utf-8', errors='ignore').strip()
        except Exception:
            try:
                return raw.strip()
            except Exception:
                return None

    def _reopen_serial_with_backoff(self):
        backoff = 0.5
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
        except Exception:
            pass
        self.serial_port = None
        while not self.stop_evt.is_set():
            try:
                self._open_serial()
                return
            except Exception as e:
                log_warn("Reopen serial failed: %s" % e)
                time.sleep(backoff)
                backoff = min(5.0, backoff * 1.5)

    # ---------- 等待首条 GGA ----------
    def _wait_first_gga(self):
        while not self.stop_evt.is_set():
            line = self._readline()
            if not line:
                time.sleep(0.01)
                continue
            self._handle_nmea_line(line, print_data=False)  # 首次仅缓存 GGA
            if self.have_gga:
                log_info("First GGA received, will connect NTRIP.")
                return

    # ---------- NTRIP ----------
    def _connect_ntrip(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            self.sock.connect((self.server_host, self.server_port))
            auth = base64.b64encode(("%s:%s" % (self.ntrip_user, self.ntrip_pass)).encode('utf-8')).decode('utf-8')
            req = (
                "GET /%s HTTP/1.0\r\n"
                "User-Agent: NTRIP simple_client\r\n"
                "Accept: */*\r\n"
                "Connection: close\r\n"
                "Authorization: Basic %s\r\n"
                "\r\n"
            ) % (self.mountpoint, auth)
            self.sock.send(req.encode('utf-8'))
            self.sock.settimeout(0.0) # 非阻塞
            log_info("NTRIP socket connected, waiting for data... " +
                     YEL + BOLD + "(If this sentence appears multiple times, it means that the CORS account is abnormal; or RTK cannot receive satellite signals.)" + RST)
        except Exception as e:
            log_warn("Connect NTRIP failed: %s" % e)
            self._safe_close_sock()

    def _safe_close_sock(self):
        try:
            if self.sock:
                try:
                    self.sock.shutdown(socket.SHUT_RDWR)
                except Exception:
                    pass
                self.sock.close()
        except Exception:
            pass
        finally:
            self.sock = None

    def _loop_send_gga(self):
        next_t = time.time()
        while not self.stop_evt.is_set():
            now = time.time()
            if now < next_t:
                time.sleep(min(0.05, next_t - now))
                continue
            next_t = now + self.gga_period

            if not self.sock or not self.latest_gga:
                continue
            payload = (self.latest_gga + "\r\n").encode('utf-8')
            try:
                _, w, _ = select.select([], [self.sock], [], 0)
                if w:
                    self.sock.send(payload)
            except OSError as e:
                if getattr(e, 'errno', None) in (errno.EPIPE, errno.ENOTCONN):
                    log_warn("NTRIP send broken pipe; try reconnect later.")
                    self._safe_close_sock()
                    self._try_reconnect_ntrip()
            except Exception:
                pass

    def _loop_recv_rtcm(self):
        """接收 NTRIP 的 RTCM 并透传到 GNSS 串口（如果 GNSS 支持基站注入）。"""
        while not self.stop_evt.is_set():
            if not self.sock or not self.serial_port:
                time.sleep(0.2)
                continue
            try:
                r, _, _ = select.select([self.sock], [], [], 0.2)
                if not r:
                    continue
                data = self.sock.recv(4096)
                if not data:
                    self._safe_close_sock()
                    self._try_reconnect_ntrip(delay=1.0)
                    continue
                if b"ICY 200 OK" in data:
                    log_info("NTRIP: ICY 200 OK")
                    continue
                try:
                    # 将 RTCM 原样写回 GNSS 模块的串口（若模块支持）
                    self.serial_port.write(data)
                except Exception as se:
                    log_warn("Write RTCM to serial failed: %s" % se)
            except Exception:
                pass

    def _try_reconnect_ntrip(self, delay=1.0):
        t0 = time.time()
        while not self.stop_evt.is_set():
            try:
                self._connect_ntrip()
                return
            except Exception:
                pass
            if time.time() - t0 > delay:
                return
            time.sleep(0.2)

    # ---------- 主循环 ----------
    def _read_serial_loop(self):
        while not self.stop_evt.is_set():
            try:
                line = self._readline()
            except (serial.SerialException, OSError) as e:
                log_warn("Serial error: %s, try reopen..." % e)
                self._reopen_serial_with_backoff()
                continue
            except Exception as e:
                log_err("Unknown serial read error: %s" % e)
                line = None

            if line:
                self._handle_nmea_line(line, print_data=True)
            time.sleep(0.005)

    # ---------- NMEA 处理 ----------
    def _handle_nmea_line(self, line, print_data):
        if not line or line[0] != '$':
            return

        # 缓存最新 GGA（用于 NTRIP 注入）
        if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
            if self._verify_nmea_checksum(line):
                self.latest_gga = line
                self.have_gga = True

        info = self.parse_nmea_sentence(line)
        if not info or not print_data:
            return

        t = info.get('type')
        if t == 'GNGGA':
            # 打印位置与解状态
            fix_q = int(info.get('fix_quality', 0))
            desc = self.gngga_fix_mapping.get(fix_q)
            lat = info.get('latitude')
            lon = info.get('longitude')
            alt = info.get('altitude')
            hdop = info.get('hdop')
            print("[GGA] fix=%d (%s) , lat=%.8f , lon=%.8f , alt=%.3f m , hdop=%.2f"% (fix_q, desc, lat or float('nan'), lon or float('nan'), alt or float('nan'), hdop or 99.9))
            sys.stdout.flush()

        elif t == 'GNRMC':
            # 打印地速与航向（仅在有效时）
            rmc_status = info.get('status', 'V')
            speed_mps = info.get('speed_mps', 0.0)
            cog_deg   = info.get('cog_deg', 0.0)
            print("[RMC] status=%s , speed=%.3f m/s (%.3f kn) , COG=%.2f°" %
                (self.gnrmc_status_mapping.get(rmc_status, 'Unknown'),
                speed_mps, info.get('raw_speed_knots', 0.0), cog_deg))
            sys.stdout.flush()

        elif t == 'GNTHS':
            # 打印真航向（True Heading）
            heading = info.get('heading', None)
            valid = info.get('valid', False)
            if heading is not None:
                print("[THS] heading=%.3f° (%s)" %
                    (float(heading), "valid" if valid else "invalid"))
                sys.stdout.flush()

    # ---------- 解析 ----------
    def parse_nmea_sentence(self, sentence):
        try:
            if sentence.startswith('$GNGGA') or sentence.startswith('$GPGGA'):
                # 打印原始 NMEA
                if self.print_raw:
                    print(sentence)
                parts = sentence.split(',')
                if len(parts) < 10:
                    return None
                time_utc = parts[1]
                lat = self.dms_to_decimal(parts[2], parts[3], is_lat=True)
                lon = self.dms_to_decimal(parts[4], parts[5], is_lat=False)
                fix_q = int(parts[6] or 0)
                num_sats = int(parts[7] or 0)
                hdop = self._to_float(parts[8], 99.9)
                alt = self._to_float(parts[9], 0.0)
                ros_time = self.nmea_time_to_epoch(time_utc)

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

            if sentence.startswith('$GNRMC') or sentence.startswith('$GPRMC'):
                # 打印原始 NMEA
                if self.print_raw:
                    print(sentence)
                parts = sentence.split(',')
                if len(parts) < 12:
                    return None
                time_utc = parts[1]
                status = parts[2]
                lat = self.dms_to_decimal(parts[3], parts[4], is_lat=True)
                lon = self.dms_to_decimal(parts[5], parts[6], is_lat=False)
                speed_knots = self._to_float(parts[7], 0.0)
                cog_deg = self._to_float(parts[8], 0.0)
                date_ddmmyy = parts[9]
                ros_time = self.nmea_time_to_epoch(time_utc)
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
            if sentence.startswith('$GNTHS') or sentence.startswith('$GPTHS'):
                # 典型格式: $GNTHS,230.1081,A*10
                if self.print_raw:
                    print(sentence)
                parts = sentence.split(',')
                if len(parts) < 3:
                    return None
                try:
                    heading = float(parts[1]) if parts[1] != '' else None
                except Exception:
                    heading = None
                valid = parts[2].startswith('A') if len(parts[2]) > 0 else False
                return {
                    'type': 'GNTHS',
                    'heading': heading,
                    'valid': valid
                }

            return None
        except Exception as e:
            log_warn("Parse NMEA error: %s" % e)
            return None        

    def dms_to_decimal(self, dms, direction, is_lat):
        try:
            if not dms or '.' not in dms:
                return None
            deg_width = 2 if is_lat else 3
            degrees = float(dms[:deg_width])
            minutes = float(dms[deg_width:])
            decimal = degrees + minutes / 60.0
            if direction in ('S', 'W'):
                decimal = -decimal
            return decimal
        except Exception:
            return None

    def nmea_time_to_epoch(self, hhmmss):
        try:
            if not hhmmss or len(hhmmss) < 6:
                return time.time()
            h = int(hhmmss[0:2])
            m = int(hhmmss[2:4])
            s = float(hhmmss[4:])
            sec = int(s)
            frac = (s - sec)
            now = time.gmtime()
            # 当天 UTC 的秒数
            epoch_day_start = int(time.mktime((now.tm_year, now.tm_mon, now.tm_mday, 0, 0, 0, 0, 0, 0))) - time.timezone
            return epoch_day_start + h * 3600 + m * 60 + sec + frac
        except Exception:
            return time.time()

    def _verify_nmea_checksum(self, sentence):
        try:
            if '*' not in sentence:
                return True
            body, cks = sentence[1:].split('*', 1)
            s = 0
            for ch in body:
                s ^= ord(ch)
            got = ("%02X" % s)
            return got.upper() == cks.strip().upper()[0:2]
        except Exception:
            return True

    def _to_float(self, s, default):
        try:
            return float(s) if s else default
        except Exception:
            return default

    # ---------- 关闭 ----------
    def stop(self):
        self.stop_evt.set()
        self._close_all()

    def _close_all(self):
        self._safe_close_sock()
        try:
            if self.serial_port and getattr(self.serial_port, "is_open", True):
                self.serial_port.close()
        except Exception:
            pass
        finally:
            self.serial_port = None


def main():
    parser = argparse.ArgumentParser(description="Simple NTRIP+GNSS client (print GGA/RMC)")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port (default: /dev/HFRobotRTK)")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baudrate (default: 115200)")
    parser.add_argument("--serial-timeout", type=float, default=1.0, help="Serial timeout seconds (default: 1.0)")
    parser.add_argument("--server", default="120.253.239.161", help="NTRIP caster host")
    parser.add_argument("--server-port", type=int, default=8002, help="NTRIP caster port")
    parser.add_argument("--username", default=os.environ.get("NTRIP_USERNAME", ""), help="NTRIP username")
    parser.add_argument("--password", default=os.environ.get("NTRIP_PASSWORD", ""), help="NTRIP password")
    parser.add_argument("--mount", default=os.environ.get("NTRIP_MOUNTPOINT", ""), help="NTRIP mountpoint")
    parser.add_argument("--gga-period", type=float, default=3.0, help="Seconds between GGA uploads (default: 3.0)")
    parser.add_argument("--print-raw", action="store_true", help="Print raw NMEA lines")
    parser.add_argument("--no-color", action="store_true", help="Disable ANSI colors in output")
    args = parser.parse_args()

    app = NtripGnssApp(args)
    try:
        app.run()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        log_err("Fatal: %s" % e)
    finally:
        app.stop()


if __name__ == "__main__":
    main()
