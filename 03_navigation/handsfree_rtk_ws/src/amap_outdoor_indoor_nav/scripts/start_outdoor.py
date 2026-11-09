#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""室外采集一键编排：RTK 驱动 -> 坐标转换 -> 锁原点 -> 导航栈 -> 对齐 -> 录包。

对应的人工顺序（见 README/对话记录）：
  1) roslaunch handsfree_rtk handsfree_rtk.launch
  2) 确认 GGA 质量 >= 1、卫星数 >= 8
  3) roslaunch amap_outdoor_indoor_nav geo_coordinate_layer_real.launch
  4) rosservice call /geo_coordinate_layer/set_origin_from_latest   <-- 漏了这步就没有 enu_pose
  5) python3 go2w_start.py start
  6) roslaunch amap_outdoor_indoor_nav enu_lio_align.launch
  7) record_outdoor.sh --planning

任何一步失败都会把已启动的进程一起收掉，不会留下半启动的现场。

常用：
  ./start_outdoor.py --dry-run              # 只打印将要做什么
  ./start_outdoor.py                        # 完整流程，Ctrl-C 结束并清理
  ./start_outdoor.py --no-record            # 不录包，只把数据流拉起来
  ./start_outdoor.py --drive                # 允许 cmd_vel 下发到底盘（默认 --no-drive）
  ./start_outdoor.py --keep                 # Ctrl-C 后保留节点继续跑
"""

import argparse
import os
import re
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String as StringMsg
from std_srvs.srv import Trigger

SCRIPT_DIR = Path(__file__).resolve().parent
# .../Go2W_Project/03_navigation/handsfree_rtk_ws/src/amap_outdoor_indoor_nav/scripts
PROJECT_ROOT = Path(os.environ.get('GO2W_PROJECT_ROOT', str(SCRIPT_DIR.parents[4])))
RECORD_SCRIPT = SCRIPT_DIR / 'record_outdoor.sh'

GGA_PATTERN = re.compile(r'\$[A-Z]{2}GGA,([^*]*)')


class SequenceError(RuntimeError):
    pass


class OutdoorSequence(object):
    def __init__(self, args):
        self.args = args
        self.children = []          # [(name, Popen)]
        self.log_dir = Path(args.log_dir)
        self.stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.stopping = False
        self.stack_started = False

        # 观测缓存
        self.last_gga_quality = -1
        self.last_gga_sats = 0
        self.last_gga_time = 0.0
        self.last_enu_pose_time = 0.0

    # ------------------------------------------------------------ 基础设施
    def log(self, message):
        print('[%s] %s' % (datetime.now().strftime('%H:%M:%S'), message), flush=True)

    def step(self, text):
        self.log('>>> ' + text)

    def spawn(self, name, command, env=None):
        """后台启动一个进程，stdout/stderr 落盘到日志目录。"""
        self.log_dir.mkdir(parents=True, exist_ok=True)
        log_path = self.log_dir / ('%s.log' % name)
        handle = open(log_path, 'ab')
        handle.write(('\n===== %s =====\n%s\n'
                      % (datetime.now().isoformat(), ' '.join(command))).encode('utf-8'))
        handle.flush()
        env_full = dict(os.environ)
        if env:
            env_full.update(env)
        process = subprocess.Popen(
            command, stdout=handle, stderr=subprocess.STDOUT,
            start_new_session=True, env=env_full)
        self.children.append((name, process))
        self.log('启动 %s (pid=%d)，日志 %s' % (name, process.pid, log_path))
        return process

    def stop_children(self):
        if self.stopping:
            return
        self.stopping = True

        # 逆序收：先给录包发 SIGINT，bag 才能正常收尾
        for name, process in reversed(self.children):
            if process.poll() is not None:
                continue
            self.log('停止 %s (pid=%d)' % (name, process.pid))
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGINT)
            except OSError:
                continue

        # 每个进程单独计时，避免前面的慢进程把后面的挤到强杀
        for name, process in reversed(self.children):
            deadline = time.time() + 15.0
            while process.poll() is None and time.time() < deadline:
                time.sleep(0.2)
            if process.poll() is None:
                self.log('%s 未在 15s 内退出，强制结束' % name)
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                except OSError:
                    pass

        # go2w_start 是外部编排器，不在 children 里，要单独停
        if self.stack_started and not self.args.keep:
            self.log('停止导航栈（go2w_start.py stop）')
            try:
                subprocess.run(
                    [sys.executable, str(PROJECT_ROOT / 'go2w_start.py'), 'stop'],
                    cwd=str(PROJECT_ROOT), timeout=90)
            except (subprocess.SubprocessError, OSError) as error:
                self.log('停止导航栈时出错：%s' % error)

    # -------------------------------------------------------------- 观测
    def topic_names(self):
        try:
            result = subprocess.run(['rostopic', 'list'], capture_output=True,
                                    text=True, timeout=8)
            return set(result.stdout.split())
        except (subprocess.SubprocessError, OSError):
            return set()

    def on_raw(self, msg):
        match = GGA_PATTERN.search(msg.data)
        if not match:
            return
        fields = match.group(1).split(',')
        if len(fields) < 7:
            return
        try:
            self.last_gga_quality = int(fields[5])
            self.last_gga_sats = int(fields[6])
            self.last_gga_time = time.time()
        except ValueError:
            return

    def on_enu_pose(self, _msg):
        self.last_enu_pose_time = time.time()

    def wait_until(self, predicate, timeout, description, hint=''):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if predicate():
                return
            rospy.sleep(0.3)
        message = '等待超时（%.0fs）：%s' % (timeout, description)
        if hint:
            message += '\n提示：' + hint
        raise SequenceError(message)

    # ---------------------------------------------------------------- 步骤
    def ensure_master(self):
        if self.topic_names():
            self.log('ROS master 已在运行')
            return
        self.step('ROS master 未运行，启动 roscore')
        if self.args.dry_run:
            return
        self.spawn('roscore', ['roscore'])
        for _ in range(40):
            if self.topic_names():
                self.log('roscore 就绪')
                return
            time.sleep(0.5)
        raise SequenceError('roscore 没起来，检查端口 11311 是否被占用')

    def start_rtk_driver(self):
        if '/handsfree/rtk/raw' in self.topic_names():
            self.log('RTK 驱动已在运行，跳过')
            return
        self.step('启动 RTK 驱动（921600）')
        if self.args.dry_run:
            return
        if self.args.ntrip:
            command = ['roslaunch', 'handsfree_rtk', 'handsfree_rtk_ntrip.launch',
                       'baudrate:=921600']   # 该 launch 默认 115200，对这个口会乱码
            for key, value in (('ntrip_server', self.args.ntrip_server),
                               ('ntrip_port', self.args.ntrip_port),
                               ('ntrip_username', self.args.ntrip_username),
                               ('ntrip_password', self.args.ntrip_password),
                               ('ntrip_mountpoint', self.args.ntrip_mountpoint)):
                if value:
                    command.append('%s:=%s' % (key, value))
            self.log('NTRIP 差分已启用')
        else:
            command = ['roslaunch', 'handsfree_rtk', 'handsfree_rtk.launch']
            self.log('串口直读模式（假定差分由 4G 模块/外部 DTU 提供）')
        self.spawn('rtk_driver', command)
        self.wait_until(lambda: '/handsfree/rtk/raw' in self.topic_names(),
                        30, '/handsfree/rtk/raw 出现',
                        '检查 /dev/HFRobotRTK 是否存在、用户是否在 dialout 组')

    def wait_rtk_quality(self):
        self.step('等待 RTK 定位：GGA 质量 >= %d 且卫星数 >= %d'
                  % (self.args.min_quality, self.args.min_sats))
        if self.args.dry_run:
            return

        rospy.Subscriber('/handsfree/rtk/raw', StringMsg, self.on_raw, queue_size=200)

        last_report = 0.0

        def good():
            nonlocal last_report
            if time.time() - last_report > 5.0:
                last_report = time.time()
                age = time.time() - self.last_gga_time if self.last_gga_time else -1
                self.log('当前：质量=%d 卫星=%d (数据龄 %.1fs)'
                         % (self.last_gga_quality, self.last_gga_sats, age))
            if not self.last_gga_time or time.time() - self.last_gga_time > 2.0:
                return False
            return (self.last_gga_quality >= self.args.min_quality
                    and self.last_gga_sats >= self.args.min_sats)

        self.wait_until(good, self.args.wait_fix_timeout,
                        'RTK 达到质量门限',
                        '质量恒为 0 / 卫星数为 0 时先查天线：接头是否拧紧、是否有源天线供电、'
                        '是否在室外无遮挡处')

    def start_geo_layer(self):
        if '/amap_nav/geo_status' in self.topic_names():
            self.log('geo_coordinate_layer 已在运行，跳过')
            return
        self.step('启动 geo_coordinate_layer')
        if self.args.dry_run:
            return
        self.spawn('geo_layer',
                   ['roslaunch', 'amap_outdoor_indoor_nav',
                    'geo_coordinate_layer_real.launch'])
        self.wait_until(lambda: '/amap_nav/enu_pose' in self.topic_names()
                        or '/amap_nav/geo_status' in self.topic_names(),
                        20, 'geo_coordinate_layer 启动',
                        '确认 amap_outdoor_indoor_nav 已 catkin_make 过')

    def lock_origin(self):
        """profile 里 origin_mode=service，不锁原点就永远不发 /amap_nav/enu_pose。"""
        self.step('锁定 ENU 原点')

        if self.args.dry_run:
            return

        # 判据是"最近有数据"，与是否已在发布无关，直接订阅即可
        rospy.Subscriber('/amap_nav/enu_pose', PoseStamped, self.on_enu_pose, queue_size=5)

        service_name = '/geo_coordinate_layer/set_origin_from_latest'
        try:
            rospy.wait_for_service(service_name, timeout=15)
        except rospy.ROSException:
            raise SequenceError('找不到服务 %s，geo_coordinate_layer 可能没启动' % service_name)

        # 服务内部会用最近一条合格 fix，先确认真的有数据进来
        self.wait_until(lambda: self.last_gga_time > 0
                        and time.time() - self.last_gga_time < 3.0,
                        15, '收到新鲜 GGA 数据（锁原点需要）')

        call = rospy.ServiceProxy(service_name, Trigger)
        for attempt in range(3):
            try:
                response = call()
            except rospy.ServiceException as error:
                raise SequenceError('调用锁原点服务失败：%s' % error)
            if response.success:
                self.log('原点已锁定：%s' % response.message)
                break
            self.log('锁原点被拒（第 %d 次）：%s' % (attempt + 1, response.message))
            rospy.sleep(1.0)
        else:
            raise SequenceError('锁原点连续失败，检查 GGA 质量是否满足 min_gga_quality')

        # 原点锁定后 enu_pose 应该立刻开始发
        self.wait_until(lambda: self.last_enu_pose_time > 0,
                        10, '/amap_nav/enu_pose 开始发布',
                        '检查 /amap_nav/geo_status 的提示文本')

    def start_stack(self):
        if self.args.no_stack:
            self.log('--no-stack：跳过导航栈')
            return
        if '/lio/odom' in self.topic_names():
            self.log('导航栈已在运行，跳过')
            return
        self.step('启动导航栈（go2w_start.py start）')
        command = [sys.executable, str(PROJECT_ROOT / 'go2w_start.py'), 'start']
        if not self.args.drive:
            command.append('--no-drive')
        if self.args.dry_run:
            self.log('将执行：' + ' '.join(command))
            return
        result = subprocess.run(command, cwd=str(PROJECT_ROOT), text=True,
                                capture_output=True)
        if result.stdout.strip():
            print(result.stdout.strip())
        if result.returncode != 0:
            if result.stderr.strip():
                print(result.stderr.strip(), file=sys.stderr)
            raise SequenceError('导航栈启动失败，先修 go2w_start.py 的前置检查')
        self.stack_started = True
        self.wait_until(lambda: '/lio/odom' in self.topic_names()
                        and '/tf' in self.topic_names(),
                        self.args.wait_stack_timeout, '/lio/odom 与 /tf 出现',
                        '查看 python3 go2w_start.py status')

    def start_align(self):
        if '/enu_lio_align/status' in self.topic_names():
            self.log('enu_lio_align 已在运行，跳过')
            return
        self.step('启动 ENU/LIO 对齐节点')
        if self.args.dry_run:
            return
        self.spawn('enu_lio_align',
                   ['roslaunch', 'amap_outdoor_indoor_nav', 'enu_lio_align.launch'])

    def start_record(self):
        if self.args.no_record:
            self.log('--no-record：不录包')
            return
        self.step('开始录包')
        command = ['bash', str(RECORD_SCRIPT)]
        if self.args.planning:
            command.append('--planning')
        if self.args.no_cloud:
            command.append('--no-cloud')
        if self.args.duration > 0:
            command += ['--duration', str(int(self.args.duration))]
        if self.args.bag_prefix:
            command += ['--prefix', self.args.bag_prefix]
        if self.args.dry_run:
            self.log('将执行：' + ' '.join(command))
            return
        self.spawn('record', command)


def build_parser():
    parser = argparse.ArgumentParser(
        description='室外采集一键编排（RTK -> ENU -> 导航栈 -> 对齐 -> 录包）')
    parser.add_argument('--ntrip', action='store_true',
                        help='仅当机器人自己联网做软件 NTRIP 时使用（会启动 handsfree_rtk_ntrip.launch）。'
                             '若 4G 模块已内置 NTRIP，不要加这个参数，保持串口直读即可')
    parser.add_argument('--ntrip-server', default='', help='覆盖 NTRIP 服务器地址')
    parser.add_argument('--ntrip-port', default='', help='覆盖 NTRIP 端口')
    parser.add_argument('--ntrip-username', default='', help='覆盖 CORS 账号')
    parser.add_argument('--ntrip-password', default='', help='覆盖 CORS 密码')
    parser.add_argument('--ntrip-mountpoint', default='', help='覆盖挂载点')
    parser.add_argument('--min-quality', type=int, default=1,
                        help='接受的最低 GGA 质量因子，1=单点 2=差分 4=RTK Fixed（默认 1）')
    parser.add_argument('--min-sats', type=int, default=8,
                        help='最低卫星数（默认 8）')
    parser.add_argument('--wait-fix-timeout', type=float, default=120.0,
                        help='等待 RTK 达到门限的秒数（默认 120）')
    parser.add_argument('--wait-stack-timeout', type=float, default=90.0,
                        help='等待导航栈出 /lio/odom 的秒数（默认 90）')
    parser.add_argument('--planning', action='store_true',
                        help='录包时带上 far_planner/地形图话题')
    parser.add_argument('--no-cloud', action='store_true',
                        help='录包时不录 /lio/cloud_world')
    parser.add_argument('--no-record', action='store_true', help='只拉数据流，不录包')
    parser.add_argument('--no-stack', action='store_true', help='不启动导航栈')
    parser.add_argument('--drive', action='store_true',
                        help='允许 cmd_vel 下发底盘（默认 --no-drive，采集时更安全）')
    parser.add_argument('--duration', type=float, default=0.0,
                        help='录制时长秒数，0 表示一直录到 Ctrl-C')
    parser.add_argument('--bag-prefix', default='',
                        help='传给 record_outdoor.sh 的 --prefix')
    parser.add_argument('--log-dir', default='',
                        help='各节点日志目录，默认 /tmp/outdoor_start_<时间>')
    parser.add_argument('--keep', action='store_true',
                        help='Ctrl-C 后保留已启动的节点（需要自己 go2w_start.py stop）')
    parser.add_argument('--dry-run', action='store_true', help='只打印，不执行')
    return parser


def main():
    args = build_parser().parse_args()
    if not args.log_dir:
        args.log_dir = '/tmp/outdoor_start_%s' % datetime.now().strftime('%Y%m%d_%H%M%S')

    sequence = OutdoorSequence(args)

    def on_signal(_signum, _frame):
        sequence.log('收到中断，开始清理')
        sequence.stop_children()   # 内部会按需调用 go2w_start.py stop
        sys.exit(0)

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    try:
        sequence.log('日志目录：%s' % args.log_dir)
        # 必须先有 master 再 init_node：rospy 连不上 master 会一直重试，不会自己报错退出
        sequence.ensure_master()
        if not args.dry_run:
            rospy.init_node('outdoor_sequence', anonymous=True, disable_signals=True)
        sequence.start_rtk_driver()
        sequence.wait_rtk_quality()
        sequence.start_geo_layer()
        sequence.lock_origin()
        sequence.start_stack()
        sequence.start_align()
        sequence.start_record()
    except SequenceError as error:
        sequence.log('失败：%s' % error)
        sequence.stop_children()
        return 1

    # --dry-run 只是预演，走完就退出，不能进下面的等待循环
    if args.dry_run:
        print()
        sequence.log('--dry-run 完成：以上为将要执行的步骤，未启动或停止任何进程')
        return 0

    print()
    sequence.log('全部就绪。现在可以开车采集，建议：')
    sequence.log('  - 路线 >= 100m，且带明显转弯（对齐需要横向铺开 >= 2m）')
    sequence.log('  - 保持速度平稳，避免原地长时间停留（静止样本会被丢弃）')
    sequence.log('  - Ctrl-C 结束：先停录包收尾 bag，再清理节点')
    print()

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
