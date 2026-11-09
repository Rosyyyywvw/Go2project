#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os

import rospy
import serial
import socket
import base64
import time
import threading
import select
import errno

from std_msgs.msg import String, Float64
from sensor_msgs.msg import NavSatFix, NavSatStatus

class NtripGnssNode(object):
    def __init__(self):
        rospy.init_node('ros_driver_um98x_ntrip', anonymous=False)

        # ---- ROS pubs ----
        self.rtk_raw_publisher = rospy.Publisher("handsfree/rtk/raw", String, queue_size=10)
        self.gnss_publisher = rospy.Publisher("handsfree/rtk/gnss", NavSatFix, queue_size=10)
        self.speed_publisher = rospy.Publisher("handsfree/rtk/speed", Float64, queue_size=10) # m/s
        self.cog_pub = rospy.Publisher("handsfree/rtk/cog", Float64, queue_size=10)       # deg (Course over ground)
        self.heading_pub = rospy.Publisher("handsfree/rtk/heading", Float64, queue_size=10)  # deg (True heading)
        # ---- params ----
        self.port = rospy.get_param('~port', '/dev/HFRobotRTK')
        self.baudrate = int(rospy.get_param('~baudrate', 115200))
        self.frame_id = rospy.get_param('~frame_id', 'gps')

        self.ntrip_server = rospy.get_param('~ntrip_server', '120.253.239.161')
        self.ntrip_port = int(rospy.get_param('~ntrip_port', 8002))
        # 凭据不入库：优先用 ROS 参数，其次读环境变量，都缺失时为空（连接会失败并报错）
        self.ntrip_username = rospy.get_param(
            '~ntrip_username', os.environ.get('NTRIP_USERNAME', ''))
        self.ntrip_password = rospy.get_param(
            '~ntrip_password', os.environ.get('NTRIP_PASSWORD', ''))
        self.ntrip_mountpoint = rospy.get_param(
            '~ntrip_mountpoint', os.environ.get('NTRIP_MOUNTPOINT', ''))

        self.gga_period = float(rospy.get_param('~gga_period', 3.0))   # 秒
        self.serial_timeout = float(rospy.get_param('~serial_timeout', 1.0))

        rospy.loginfo("NTRIP server: %s:%d / %s", self.ntrip_server, self.ntrip_port, self.ntrip_mountpoint)

        # ---- runtime states ----
        self.serial_port = None
        self.ntrip_sock = None
        self.latest_gga = None # 最近一条 GGA 原文
        self.have_gga = False

        # 退出/停机控制
        self.stop_evt = threading.Event()
        rospy.on_shutdown(self._on_shutdown)

        # 线程句柄
        self.gga_thread = None
        self.rtcm_thread = None

        # fix 描述与协方差映射
        self.flag_desc = {
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

        self.gnrmc_mode_mapping = {
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

        self.cov_by_flag = {
            0: 10000.0, # invalid
            1: 10.0,
            2: 10000.0,
            3: 10000.0,
            4: 0.1, # RTK Fixed
            5: 1.0, # RTK Float
            6: 10000.0,
            7: 10000.0,
            8: 10000.0,
        }

        # ---- open serial ----
        self._open_serial_blocking()

        # 等待第一条 GGA 再去连接 NTRIP（许多播发端需要先收到 GGA 才开始下发）
        self._wait_first_gga()

        # ---- connect NTRIP ----
        self._connect_ntrip()

        # ---- start background loops (daemon threads) ----
        self.gga_thread = threading.Thread(target=self._loop_send_gga,  name="gga_tx")
        self.gga_thread.setDaemon(True)
        self.gga_thread.start()

        self.rtcm_thread = threading.Thread(target=self._loop_recv_rtcm, name="rtcm_rx")
        self.rtcm_thread.setDaemon(True)
        self.rtcm_thread.start()

        # ---- main read loop ----
        self._read_serial_loop()

        # 程序正常结束
        self._close_all()

    # ------------------- IO -------------------
    def _open_serial(self):
        try:
            self.serial_port = serial.Serial(self.port, self.baudrate, timeout=self.serial_timeout)
            rospy.loginfo("Opened serial %s @ %d", self.port, self.baudrate)
        except Exception as e:
            rospy.logerr("Open serial failed: %s", e)
            raise

    def _wait_first_gga(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and not self.stop_evt.is_set():
            try:
                line = self._readline()
                if not line:
                    rate.sleep()
                    continue
                self._handle_nmea_line(line)
                if self.have_gga:
                    rospy.loginfo("First GGA received, proceed to connect NTRIP.")
                    return
            except Exception as e:
                rospy.logerr("Error waiting GGA: %s", e)
                rate.sleep()

    def _connect_ntrip(self):
        try:
            self.ntrip_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.ntrip_sock.settimeout(5.0)
            self.ntrip_sock.connect((self.ntrip_server, self.ntrip_port))
            auth = base64.b64encode((self.ntrip_username + ":" + self.ntrip_password).encode('utf-8')).decode('utf-8')
            req = (
                "GET /%s HTTP/1.0\r\n"
                "User-Agent: NTRIP ntrip_client\r\n"
                "Accept: */*\r\n"
                "Connection: close\r\n"
                "Authorization: Basic %s\r\n"
                "\r\n"
            ) % (self.ntrip_mountpoint, auth)
            self.ntrip_sock.send(req.encode('utf-8'))
            self.ntrip_sock.settimeout(0.0) # 非阻塞
            rospy.loginfo("NTRIP socket connected, waiting for data... %s%s(If this sentence appears multiple times, it means that the CORS account is abnormal; or RTK cannot receive satellite signals.)%s", "\033[1m", "\033[33m", "\033[0m")
        except Exception as e:
            rospy.logerr("Failed to connect to NTRIP server: %s", e)
            # 允许本地 GNSS 单独运行
            self._safe_close_sock()

    def _readline(self):
        if self.serial_port is None:
            return None
        raw = self.serial_port.readline()
        if not raw:
            return None
        try:
            return raw.decode('utf-8', errors='ignore').strip()
        except Exception:
            # Python2 / 非 UTF-8 的情况
            try:
                return raw.strip()
            except Exception:
                return None

    def _close_all(self):
        self._safe_close_sock()
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
        except Exception:
            pass
        finally:
            self.serial_port = None

    def _safe_close_sock(self):
        try:
            if self.ntrip_sock:
                try:
                    self.ntrip_sock.shutdown(socket.SHUT_RDWR)
                except Exception:
                    pass
                self.ntrip_sock.close()
        except Exception:
            pass
        finally:
            self.ntrip_sock = None

    # ------------------- Shutdown -------------------
    def _on_shutdown(self):
        # Ctrl+C 或 rosnode kill 会回调这里
        self.stop_evt.set()
        # 提前打断可能的阻塞
        self._safe_close_sock()
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
        except Exception:
            pass

    # ------------------- Background loops -------------------
    def _loop_send_gga(self):
        """周期发送最新 GGA 到 NTRIP；非阻塞 + 可中断"""
        period = max(0.2, float(self.gga_period))
        next_t = time.time()
        while not rospy.is_shutdown() and not self.stop_evt.is_set():
            now = time.time()
            if now < next_t:
                time.sleep(min(0.05, next_t - now))
                continue
            next_t = now + period

            if not self.ntrip_sock or not self.latest_gga:
                continue

            payload = (self.latest_gga + "\r\n").encode('utf-8')
            try:
                # 仅在可写时发送，避免 send 阻塞
                _, w, _ = select.select([], [self.ntrip_sock], [], 0)
                if w:
                    self.ntrip_sock.send(payload)
            except OSError as e:
                if getattr(e, 'errno', None) in (errno.EPIPE, errno.ENOTCONN):
                    rospy.logwarn("NTRIP send broken pipe, will retry connect later.")
                    self._safe_close_sock()
                    self._try_reconnect_ntrip()
            except Exception as e:
                rospy.logdebug("Send GGA failed: %s", e)

    def _loop_recv_rtcm(self):
        """接收 RTCM 并透传串口；非阻塞 + 可中断"""
        while not rospy.is_shutdown() and not self.stop_evt.is_set():
            if not self.ntrip_sock or not self.serial_port:
                time.sleep(0.2)
                continue
            try:
                r, _, _ = select.select([self.ntrip_sock], [], [], 0.2)
                if not r:
                    continue
                data = self.ntrip_sock.recv(4096)
                if not data:
                    # 远端关闭
                    self._safe_close_sock()
                    self._try_reconnect_ntrip(delay=1.0)
                    continue
                if b"ICY 200 OK" in data:
                    rospy.loginfo("Connected to NTRIP server (ICY 200 OK).")
                    continue
                try:
                    self.serial_port.write(data)
                except Exception as se:
                    rospy.logwarn("Write RTCM to serial failed: %s", se)
            except Exception:
                # 非阻塞下超时/无数据/短暂错误都忽略
                pass

    def _try_reconnect_ntrip(self, delay=1.0):
        """短期重连 NTRIP（非阻塞，不影响退出）"""
        t0 = time.time()
        while not rospy.is_shutdown() and not self.stop_evt.is_set():
            try:
                self._connect_ntrip()
                return
            except Exception:
                pass
            if time.time() - t0 > delay:
                return
            time.sleep(0.2)

    # ------------------- Main loop (serial + hotplug) -------------------
    def _read_serial_loop(self):
        rate = rospy.Rate(200)
        while not rospy.is_shutdown() and not self.stop_evt.is_set():
            try:
                line = self._readline()
            except (serial.SerialException, OSError) as e:
                rospy.logwarn("Serial error: %s, try reopen...", e)
                self._reopen_serial_with_backoff()
                continue
            except Exception as e:
                rospy.logerr("Unknown serial read error: %s", e)
                line = None

            if line:
                self._handle_nmea_line(line)
            rate.sleep()
    
    def _open_serial_blocking(self):
        """
        启动期阻塞式打开串口：无限重试，直到成功或节点退出。
        使用指数退避（0.5s 起，5s 封顶），重试次数不封顶。
        """
        backoff = 0.5
        while not rospy.is_shutdown() and not self.stop_evt.is_set():
            try:
                self.serial_port = serial.Serial(self.port, self.baudrate, timeout=self.serial_timeout)
                rospy.loginfo("Opened serial %s @ %d (initial blocking open)", self.port, self.baudrate)
                return
            except Exception as e:
                rospy.logwarn("Open serial failed (will retry): %s", e)
                try:
                    rospy.sleep(backoff)
                except rospy.ROSInterruptException:
                    pass
                backoff = min(5.0, backoff * 1.5)  # 次数无限，但等待时长封顶避免过大

        raise rospy.ROSInterruptException("Node shutdown before serial could be opened.")

    def _reopen_serial_with_backoff(self):
        """串口热重连，期间可被 stop_evt 立即打断"""
        backoff = 0.5
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
        except Exception:
            pass
        self.serial_port = None

        while not rospy.is_shutdown() and not self.stop_evt.is_set():
            try:
                self._open_serial()
                return
            except Exception as e:
                rospy.logwarn("Reopen serial failed: %s", e)
                time.sleep(backoff)
                backoff = min(5.0, backoff * 1.5)

    # ------------------- NMEA handling -------------------
    def _handle_nmea_line(self, line):
        # 发布原始 NMEA
        try:
            self.rtk_raw_publisher.publish(line)
        except Exception:
            pass

        if not line or line[0] != '$':
            return

        # 记录最新 GGA（供 NTRIP 回传）
        if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
            if self._verify_nmea_checksum(line):
                parts = line.split(',')
                has_latlon = len(parts) > 5 and parts[2] != '' and parts[4] != ''
                if has_latlon:
                    self.latest_gga = line
                    self.have_gga = True

        # 结构化解析（GGA/RMC）
        info = self.parse_nmea_sentence(line)
        if not info:
            return

        if info.get('type') == 'GNGGA':
            # 发布 NavSatFix
            self._publish_gngga_data(info)

        elif info.get('type') == 'GNRMC':
            # 发布 航向角和地速
            self._publish_gnrmc_data(info)
        
        elif info.get('type') == 'GNTHS':
            # 发布真航向角
            self._publish_gnths_data(info)

    def _publish_gngga_data(self, gga):
        try:
            fix_q = int(gga.get('fix_quality', 0))
            lat = gga.get('latitude', None)
            lon = gga.get('longitude', None)
            alt = gga.get('altitude', None)

            # 只有 fix!=0 且 三要素都存在才发布
            if fix_q == 0 or lat is None or lon is None or alt is None:
                rospy.logdebug("GGA skipped: no fix or missing fields (fix=%s).", fix_q)
                return  # 不发布

            nav = NavSatFix()
            nav.header.stamp = rospy.Time.now()
            nav.header.frame_id = self.frame_id
            nav.status.status = self._ros_status_from_gga(fix_q)
            nav.status.service = NavSatStatus.SERVICE_GPS
            nav.latitude = float(lat)
            nav.longitude = float(lon)
            nav.altitude = float(alt)

            cov = self.cov_by_flag.get(fix_q, 10000.0)
            nav.position_covariance = [cov, 0.0, 0.0,
                                    0.0, cov, 0.0,
                                    0.0, 0.0, cov]
            nav.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

            self.gnss_publisher.publish(nav)

            desc = self.flag_desc.get(fix_q, "Unknown")
            rospy.loginfo("GGA: fix=%d (%s), lat=%.8f lon=%.8f alt=%.3f",
                        fix_q, desc, nav.latitude, nav.longitude, nav.altitude)
        except Exception as e:
            rospy.logwarn("Publish NavSatFix failed (skipped): %s", e)

    def _publish_gnrmc_data(self, rmc):
        try:
            rmc_status = rmc.get('status', 'V')
            speed_mps = rmc.get('speed_mps', 0.0)
            cog_deg = rmc.get('cog_deg', 0.0)

            if rmc_status != 'A':
                rospy.logdebug("RMC skipped: invalid.")
                return  # 不发布

            self.speed_publisher.publish(Float64(data=speed_mps))
            self.cog_pub.publish(Float64(data=cog_deg))
            rospy.loginfo("RMC: valid, speed=%.3f m/s (%.3f kn), COG=%.2f°",
                        speed_mps, rmc.get('raw_speed_knots', 0.0), cog_deg)
        except Exception as e:
            rospy.logwarn("Publish RMC failed (skipped): %s", e)

    def _publish_gnths_data(self, ths):
        try:
            heading = ths.get('heading', None)
            valid = ths.get('valid', False)
            if not valid or heading is None:
                rospy.logdebug("THS skipped: invalid or empty.")
                return  # 不发布
            self.heading_pub.publish(Float64(data=heading))
            rospy.loginfo("THS: heading=%.3f° (valid)", heading)
        except Exception as e:
            rospy.logwarn("Publish GNTHS failed (skipped): %s", e)

    # ------------------- Parsers -------------------
    def parse_nmea_sentence(self, sentence):
        """
        解析NMEA句子：支持 GNGGA / GNRMC
        返回包含 'type' 的 dict
        """
        try:
            if sentence.startswith('$GNGGA') or sentence.startswith('$GPGGA'):
                # $--GGA,1:UTC,2:lat,3:NS,4:lon,5:EW,6:fix,7:num,8:hdop,9:alt,10:unit,...
                rospy.loginfo(sentence)
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
                desc = self.flag_desc.get(fix_q, "Unknown")
                ros_time = self.nmea_time_to_ros(time_utc)

                return {
                    'type': 'GNGGA',
                    'time': ros_time,
                    'latitude': lat,
                    'longitude': lon,
                    'altitude': alt,
                    'fix_quality': fix_q,
                    'num_sats': num_sats,
                    'hdop': hdop,
                    'status_description': desc
                }

            if sentence.startswith('$GNRMC') or sentence.startswith('$GPRMC'):
                # $--RMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh
                rospy.loginfo(sentence)
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
                ros_time = self.nmea_time_to_ros(time_utc)
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
                # 格式: $GNTHS,230.1081,A*10
                rospy.loginfo(sentence)
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
            rospy.logerr("解析NMEA句子错误: %s", e)
            return None

    # ------------------- Utilities -------------------
    def dms_to_decimal(self, dms, direction, is_lat):
        """
        dms: 'ddmm.mmmm' (lat) 或 'dddmm.mmmm' (lon)
        """
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

    def nmea_time_to_ros(self, hhmmss):
        """
        把 NMEA 的 UTC 时间（hhmmss[.ss]）粗略转换为 rospy.Time（按当天 UTC 秒数拼接，不做日期对齐）。
        如需严格 UTC 日期拼接，可在此加入日期解析（结合 RMC 的 ddmmyy）。
        """
        try:
            if not hhmmss or len(hhmmss) < 6:
                return rospy.Time.now()
            h = int(hhmmss[0:2])
            m = int(hhmmss[2:4])
            s = float(hhmmss[4:])
            sec = int(s)
            nsec = int((s - sec) * 1e9)
            # 以当前日期的 UTC 起点 + h:m:s 构造（近似）
            now = time.gmtime()
            day_start = time.struct_time((now.tm_year, now.tm_mon, now.tm_mday,
                                          0, 0, 0, now.tm_wday, now.tm_yday, now.tm_isdst))
            epoch_day_start = int(time.mktime(day_start)) - time.timezone  # 转 UTC
            return rospy.Time.from_sec(epoch_day_start + h * 3600 + m * 60 + sec + nsec / 1e9)
        except Exception:
            return rospy.Time.now()

    def _ros_status_from_gga(self, flag):
        if flag in (4,): # RTK Fixed
            return NavSatStatus.STATUS_GBAS_FIX
        if flag in (5, 2, 1, 3): # Float/DGPS/GPS/PPS -> treat as FIX
            return NavSatStatus.STATUS_FIX
        return NavSatStatus.STATUS_NO_FIX

    def _verify_nmea_checksum(self, sentence):
        """
        简单 NMEA 校验：'$' 与 '*' 之间 XOR，和 '*' 后两位十六进制比对
        """
        try:
            if '*' not in sentence:
                return True # 没有校验字段时放行
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


if __name__ == '__main__':
    try:
        node = NtripGnssNode()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
