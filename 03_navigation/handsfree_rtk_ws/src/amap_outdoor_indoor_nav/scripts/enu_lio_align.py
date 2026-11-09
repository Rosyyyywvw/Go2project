#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ENU(geo_enu) <-> Super-LIO(world) 对齐节点。

用同一段轨迹上同时采到的 /amap_nav/enu_pose 与 /lio/odom 拟合一个 2D 刚体变换
（yaw + 平移），然后按 TF 语义发布 world -> geo_enu：

    p_world = R(yaw) * p_geo_enu + t

之所以是这个方向，是因为 /goal_point 必须是 world 帧，而目标点通常先以
经纬度/ENU 形式给出，需要从 geo_enu 转到 world。

用法：
  1) 室外跑起来 geo_coordinate_layer，并确保已设过 ENU 原点；
  2) 正常开 Super-LIO，让机器人走一段 >= min_baseline 的路线（带点转弯更好）；
  3) 节点自动收样本，样本够且不共线时自动拟合并持续发布 TF；
  4) 想手动重算：rosservice call /enu_lio_align/fit
"""

import bisect
import json
import math
import os

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse


def _yaw_to_quaternion(yaw):
    """只绕 z 轴旋转的四元数，返回 (z, w)。"""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def solve_2d_transform(enu_xy, world_xy):
    """最小二乘求 w = R(yaw) * e + t 的 yaw、t 与残差 RMS。

    输入是 Nx2 的 numpy 数组（ENU 平面坐标与 LIO world 平面坐标）。
    返回 (yaw_rad, t_xy(Nx2 的 2 元素数组), rms_m)。
    """
    enu_xy = np.asarray(enu_xy, dtype=float)
    world_xy = np.asarray(world_xy, dtype=float)

    enu_mean = enu_xy.mean(axis=0)
    world_mean = world_xy.mean(axis=0)
    ec = enu_xy - enu_mean
    wc = world_xy - world_mean

    numerator = float(np.sum(ec[:, 0] * wc[:, 1] - ec[:, 1] * wc[:, 0]))
    denominator = float(np.sum(ec[:, 0] * wc[:, 0] + ec[:, 1] * wc[:, 1]))
    yaw = math.atan2(numerator, denominator)

    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
    t_xy = world_mean - rotation.dot(enu_mean)

    residual = world_xy - (enu_xy.dot(rotation.T) + t_xy)
    rms = float(math.sqrt(np.mean(np.sum(residual ** 2, axis=1))))
    return yaw, t_xy, rms


class EnuLioAlign(object):
    def __init__(self):
        # ---- 参数 -------------------------------------------------------
        self.parent_frame = rospy.get_param('~parent_frame', 'world')
        self.child_frame = rospy.get_param('~child_frame', 'geo_enu')
        self.enu_topic = rospy.get_param('~enu_topic', '/amap_nav/enu_pose')
        self.odom_topic = rospy.get_param('~odom_topic', '/lio/odom')

        # 拟合所需条件
        self.min_samples = int(rospy.get_param('~min_samples', 60))
        self.max_time_diff = float(rospy.get_param('~max_time_diff', 0.15))
        self.min_baseline = float(rospy.get_param('~min_baseline', 10.0))
        self.min_lateral_spread = float(rospy.get_param('~min_lateral_spread', 2.0))
        self.max_rms = float(rospy.get_param('~max_rms', 1.0))
        self.min_sample_spacing = float(rospy.get_param('~min_sample_spacing', 0.2))
        self.enu_buffer_sec = float(rospy.get_param('~enu_buffer_sec', 5.0))

        self.auto_fit = rospy.get_param('~auto_fit', True)
        self.tf_rate = float(rospy.get_param('~tf_rate', 10.0))

        # 标定结果持久化（空字符串表示不落盘）
        self.calibration_file = os.path.expanduser(
            rospy.get_param('~calibration_file', ''))
        self.save_calibration = rospy.get_param('~save_calibration', True)

        # ---- 状态 -------------------------------------------------------
        self.enu_stamps = []      # 单调递增的 stamp(秒)
        self.enu_points = []      # [(x, y, z)]
        self.pairs = []           # [((ex, ey, ez), (wx, wy, wz))]
        self.last_world_xy = None

        self.fitted = False
        self.yaw = 0.0
        self.translation = np.zeros(3)
        self.rms = float('nan')
        self.fit_reason = 'not fitted'

        # ---- 接口 -------------------------------------------------------
        self.status_pub = rospy.Publisher('~status', String, queue_size=10, latch=True)
        self.report_pub = rospy.Publisher('~report', String, queue_size=10, latch=True)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.enu_sub = rospy.Subscriber(
            self.enu_topic, PoseStamped, self.enu_callback, queue_size=200)
        self.odom_sub = rospy.Subscriber(
            self.odom_topic, Odometry, self.odom_callback, queue_size=200)

        self.fit_srv = rospy.Service('~fit', Trigger, self.handle_fit)
        self.reset_srv = rospy.Service('~reset', Trigger, self.handle_reset)

        if self.calibration_file:
            self.load_calibration()

        if self.tf_rate > 0.0:
            self.timer = rospy.Timer(rospy.Duration(1.0 / self.tf_rate), self.publish_tf)

        rospy.loginfo(
            'enu_lio_align: %s(%s) <- %s(%s), auto_fit=%s, min_samples=%d, '
            'min_baseline=%.1fm, min_lateral=%.1fm',
            self.parent_frame, self.odom_topic, self.child_frame, self.enu_topic,
            self.auto_fit, self.min_samples, self.min_baseline, self.min_lateral_spread)

    # ------------------------------------------------------------------ 输入
    def enu_callback(self, msg):
        """缓存 ENU 观测；geo_coordinate_layer 只在通过质量门限时才发这个话题。"""
        stamp = msg.header.stamp.to_sec()
        if stamp <= 0.0:
            stamp = rospy.get_time()
        index = bisect.bisect_right(self.enu_stamps, stamp)
        self.enu_stamps.insert(index, stamp)
        self.enu_points.insert(
            index, (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))

        # 丢掉过期样本，避免无界增长
        horizon = stamp - self.enu_buffer_sec
        drop = 0
        while drop < len(self.enu_stamps) and self.enu_stamps[drop] < horizon:
            drop += 1
        if drop:
            del self.enu_stamps[:drop]
            del self.enu_points[:drop]

    def _match_enu(self, stamp):
        """在 ENU 缓存里找时间最接近的一条，超过 max_time_diff 视为无匹配。"""
        if not self.enu_stamps:
            return None
        index = bisect.bisect_left(self.enu_stamps, stamp)
        best = None
        best_dt = None
        for candidate in (index - 1, index):
            if 0 <= candidate < len(self.enu_stamps):
                dt = abs(self.enu_stamps[candidate] - stamp)
                if best_dt is None or dt < best_dt:
                    best_dt = dt
                    best = candidate
        if best is None or best_dt > self.max_time_diff:
            return None
        return self.enu_points[best]

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        world_xyz = (position.x, position.y, position.z)

        # 原地不动时不重复收样本，否则静止段会主导最小二乘
        if self.last_world_xy is not None:
            dx = world_xyz[0] - self.last_world_xy[0]
            dy = world_xyz[1] - self.last_world_xy[1]
            if math.hypot(dx, dy) < self.min_sample_spacing:
                return

        enu_xyz = self._match_enu(msg.header.stamp.to_sec())
        if enu_xyz is None:
            return

        self.last_world_xy = (world_xyz[0], world_xyz[1])
        self.pairs.append((enu_xyz, world_xyz))
        self.publish_status()

        if self.auto_fit and not self.fitted and len(self.pairs) >= self.min_samples:
            ok, _ = self.fit()
            if not ok:
                # 样本还不够"不共线"，继续走就行，不必刷屏
                pass

    # ------------------------------------------------------------------ 拟合
    def fit(self):
        if len(self.pairs) < self.min_samples:
            self.fit_reason = 'need >= %d samples, have %d' % (
                self.min_samples, len(self.pairs))
            self.publish_status()
            return False, self.fit_reason

        enu = np.array([p[0] for p in self.pairs], dtype=float)
        world = np.array([p[1] for p in self.pairs], dtype=float)

        # 简并检查 1：轨迹长度
        span = enu[:, :2].max(axis=0) - enu[:, :2].min(axis=0)
        baseline = float(np.hypot(span[0], span[1]))
        if baseline < self.min_baseline:
            self.fit_reason = 'baseline %.2fm < %.2fm' % (baseline, self.min_baseline)
            self.publish_status()
            return False, self.fit_reason

        # 简并检查 2：共线时 yaw 有 180 度歧义，要求横向有足够铺开
        centered = enu[:, :2] - enu[:, :2].mean(axis=0)
        covariance = centered.T.dot(centered) / max(1, len(centered))
        eigenvalues = np.linalg.eigvalsh(covariance)
        lateral = float(math.sqrt(max(eigenvalues[0], 0.0)))
        if lateral < self.min_lateral_spread:
            self.fit_reason = ('trajectory too straight: lateral spread %.2fm < %.2fm'
                               % (lateral, self.min_lateral_spread))
            self.publish_status()
            return False, self.fit_reason

        # 2D Kabsch：求 w = R(yaw) * e + t
        yaw, t_xy, rms = solve_2d_transform(enu[:, :2], world[:, :2])

        if rms > self.max_rms:
            self.fit_reason = 'RMS %.3fm > max_rms %.3fm' % (rms, self.max_rms)
            self.publish_status()
            return False, self.fit_reason

        # 高度单独取中位数，避免个别跳变拉偏（LIO 的 z 漂移与 RTK 高程不同源）
        t_z = float(np.median(world[:, 2] - enu[:, 2]))

        self.yaw = yaw
        self.translation = np.array([t_xy[0], t_xy[1], t_z])
        self.rms = rms
        self.fitted = True
        self.fit_reason = 'ok'

        rospy.loginfo(
            'enu_lio_align fitted: samples=%d yaw=%.2fdeg t=(%.2f, %.2f, %.2f) rms=%.3fm',
            len(self.pairs), math.degrees(yaw), self.translation[0],
            self.translation[1], self.translation[2], rms)

        self.publish_status()
        self.publish_report(baseline, lateral)
        if self.save_calibration:
            self.save_calibration_file()
        self.publish_tf(None)
        return True, 'ok'

    # ------------------------------------------------------------------ 输出
    def publish_tf(self, _event):
        if not self.fitted:
            return
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame
        transform.transform.translation.x = float(self.translation[0])
        transform.transform.translation.y = float(self.translation[1])
        transform.transform.translation.z = float(self.translation[2])
        quaternion_z, quaternion_w = _yaw_to_quaternion(self.yaw)
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = quaternion_z
        transform.transform.rotation.w = quaternion_w
        self.tf_broadcaster.sendTransform(transform)

    def publish_status(self):
        payload = {
            'fitted': self.fitted,
            'samples': len(self.pairs),
            'reason': self.fit_reason,
            'yaw_deg': math.degrees(self.yaw) if self.fitted else None,
            'translation': list(self.translation) if self.fitted else None,
            'rms_m': self.rms if self.fitted else None,
        }
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def publish_report(self, baseline, lateral):
        report = {
            'parent_frame': self.parent_frame,
            'child_frame': self.child_frame,
            'samples': len(self.pairs),
            'baseline_m': round(baseline, 3),
            'lateral_spread_m': round(lateral, 3),
            'yaw_deg': round(math.degrees(self.yaw), 4),
            'translation_xyz': [round(float(v), 4) for v in self.translation],
            'rms_m': round(self.rms, 4),
        }
        self.report_pub.publish(String(data=json.dumps(report, ensure_ascii=False)))

    # -------------------------------------------------------------- 服务/持久化
    def handle_fit(self, _request):
        ok, reason = self.fit()
        if ok:
            return TriggerResponse(success=True, message=(
                'yaw=%.2fdeg t=(%.2f, %.2f, %.2f) rms=%.3fm'
                % (math.degrees(self.yaw), self.translation[0],
                   self.translation[1], self.translation[2], self.rms)))
        return TriggerResponse(success=False, message=reason)

    def handle_reset(self, _request):
        self.pairs = []
        self.last_world_xy = None
        self.fitted = False
        self.yaw = 0.0
        self.translation = np.zeros(3)
        self.rms = float('nan')
        self.fit_reason = 'reset'
        self.publish_status()
        return TriggerResponse(success=True, message='samples cleared')

    def save_calibration_file(self):
        payload = {
            'parent_frame': self.parent_frame,
            'child_frame': self.child_frame,
            'yaw_rad': self.yaw,
            'yaw_deg': math.degrees(self.yaw),
            'translation_xyz': [float(v) for v in self.translation],
            'rms_m': self.rms,
            'samples': len(self.pairs),
        }
        try:
            directory = os.path.dirname(self.calibration_file)
            if directory and not os.path.isdir(directory):
                os.makedirs(directory)
            with open(self.calibration_file, 'w') as handle:
                json.dump(payload, handle, indent=2, ensure_ascii=False)
            rospy.loginfo('enu_lio_align: calibration saved to %s', self.calibration_file)
        except (IOError, OSError) as error:
            rospy.logwarn('enu_lio_align: failed to save calibration: %s', error)

    def load_calibration(self):
        if not os.path.isfile(self.calibration_file):
            return
        try:
            with open(self.calibration_file) as handle:
                payload = json.load(handle)
            self.yaw = float(payload['yaw_rad'])
            self.translation = np.array(payload['translation_xyz'], dtype=float)
            self.rms = float(payload.get('rms_m', float('nan')))
            self.fitted = True
            self.fit_reason = 'loaded from %s' % self.calibration_file
            rospy.loginfo('enu_lio_align: loaded calibration from %s (yaw=%.2fdeg)',
                          self.calibration_file, math.degrees(self.yaw))
            self.publish_status()
        except (KeyError, ValueError, IOError, OSError) as error:
            rospy.logwarn('enu_lio_align: ignoring bad calibration file: %s', error)


def main():
    rospy.init_node('enu_lio_align')
    EnuLioAlign()
    rospy.spin()


if __name__ == '__main__':
    main()
