#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把经纬度目标点转成 far_planner 认识的 world 帧 /goal_point。

链路：经纬度 -> ENU(geo_enu，与 geo_coordinate_layer 完全同一套 WGS-84/ECEF/ENU
公式) -> 经 TF 变换到 world -> 发布 geometry_msgs/PointStamped。

为什么不用 service：ROS 里没有现成的"带两个 double 的 Trigger"，而 geographic_msgs
在本机没装。所以用话题接目标点，命令行就能直接发：

    rostopic pub -1 /geo_goal/target sensor_msgs/NavSatFix \
      '{latitude: 22.5410000, longitude: 113.9290000}'

也可以设参数后用服务重发：
    rosparam set /geo_goal_publisher/target_latitude 22.541
    rosparam set /geo_goal_publisher/target_longitude 113.929
    rosservice call /geo_goal_publisher/send
"""

import math

import rospy
import tf2_ros
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

# 与 geo_coordinate_layer.cpp 保持完全一致，否则 ENU 会与 AMap 面板对不上
WGS84_SEMI_MAJOR_AXIS = 6378137.0
WGS84_FLATTENING = 1.0 / 298.257223563
WGS84_ECCENTRICITY_SQ = WGS84_FLATTENING * (2.0 - WGS84_FLATTENING)


def geodetic_to_ecef(latitude_deg, longitude_deg, altitude):
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    sin_latitude = math.sin(latitude)
    cos_latitude = math.cos(latitude)
    sin_longitude = math.sin(longitude)
    cos_longitude = math.cos(longitude)

    prime_vertical = WGS84_SEMI_MAJOR_AXIS / math.sqrt(
        1.0 - WGS84_ECCENTRICITY_SQ * sin_latitude * sin_latitude)

    x = (prime_vertical + altitude) * cos_latitude * cos_longitude
    y = (prime_vertical + altitude) * cos_latitude * sin_longitude
    z = (prime_vertical * (1.0 - WGS84_ECCENTRICITY_SQ) + altitude) * sin_latitude
    return x, y, z


def quaternion_to_matrix(x, y, z, w):
    """四元数转 3x3 旋转矩阵。不依赖 tf2_geometry_msgs 的消息注册。"""
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def ecef_to_enu(target_ecef, origin_ecef, origin_latitude_deg, origin_longitude_deg):
    delta_x = target_ecef[0] - origin_ecef[0]
    delta_y = target_ecef[1] - origin_ecef[1]
    delta_z = target_ecef[2] - origin_ecef[2]

    latitude = math.radians(origin_latitude_deg)
    longitude = math.radians(origin_longitude_deg)
    sin_latitude = math.sin(latitude)
    cos_latitude = math.cos(latitude)
    sin_longitude = math.sin(longitude)
    cos_longitude = math.cos(longitude)

    east = -sin_longitude * delta_x + cos_longitude * delta_y
    north = (-sin_latitude * cos_longitude * delta_x
             - sin_latitude * sin_longitude * delta_y
             + cos_latitude * delta_z)
    up = (cos_latitude * cos_longitude * delta_x
          + cos_latitude * sin_longitude * delta_y
          + sin_latitude * delta_z)
    return east, north, up


class GeoGoalPublisher(object):
    def __init__(self):
        self.parent_frame = rospy.get_param('~parent_frame', 'world')
        self.enu_frame = rospy.get_param('~enu_frame', 'geo_enu')
        self.target_topic = rospy.get_param('~target_topic', '/geo_goal/target')
        self.goal_topic = rospy.get_param('~goal_topic', '/goal_point')
        self.origin_topic = rospy.get_param('~origin_topic', '/amap_nav/enu_origin')
        self.status_topic = rospy.get_param('~status_topic', '/geo_goal/status')
        self.debug_topic = rospy.get_param('~debug_topic', '/geo_goal/world')
        self.repeat_count = int(rospy.get_param('~repeat_count', 3))
        self.repeat_interval = float(rospy.get_param('~repeat_interval_sec', 0.1))
        self.tf_timeout = float(rospy.get_param('~tf_timeout_sec', 1.0))

        # 参数兜底：没有 enu_origin 话题时可用参数直接给原点
        self.origin_latitude = rospy.get_param('~origin_latitude', float('nan'))
        self.origin_longitude = rospy.get_param('~origin_longitude', float('nan'))
        self.origin_altitude = rospy.get_param('~origin_altitude', 0.0)
        self.has_origin = (math.isfinite(self.origin_latitude)
                           and math.isfinite(self.origin_longitude))
        self.origin_ecef = None
        if self.has_origin:
            self.origin_ecef = geodetic_to_ecef(
                self.origin_latitude, self.origin_longitude, self.origin_altitude)

        # 最近一次显式指定的目标（供 send 服务重发）
        self.last_target = None

        self.goal_pub = rospy.Publisher(self.goal_topic, PointStamped, queue_size=5)
        self.debug_pub = rospy.Publisher(self.debug_topic, PointStamped, queue_size=5)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10, latch=True)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.origin_sub = rospy.Subscriber(
            self.origin_topic, NavSatFix, self.origin_callback, queue_size=1)
        self.target_sub = rospy.Subscriber(
            self.target_topic, NavSatFix, self.target_callback, queue_size=5)

        self.send_srv = rospy.Service('~send', Trigger, self.handle_send)

        if self.has_origin:
            rospy.loginfo('geo_goal_publisher: origin from params (%.7f, %.7f)',
                          self.origin_latitude, self.origin_longitude)
        else:
            rospy.loginfo('geo_goal_publisher: waiting for ENU origin on %s',
                          self.origin_topic)

    # ---------------------------------------------------------------- 原点
    def origin_callback(self, msg):
        if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
            return
        altitude = msg.altitude if math.isfinite(msg.altitude) else 0.0
        self.origin_latitude = msg.latitude
        self.origin_longitude = msg.longitude
        self.origin_altitude = altitude
        self.origin_ecef = geodetic_to_ecef(msg.latitude, msg.longitude, altitude)
        self.has_origin = True

    # ---------------------------------------------------------------- 目标
    def target_callback(self, msg):
        self.publish_goal(msg.latitude, msg.longitude, msg.altitude)

    def handle_send(self, _request):
        latitude = rospy.get_param('~target_latitude', float('nan'))
        longitude = rospy.get_param('~target_longitude', float('nan'))
        altitude = rospy.get_param('~target_altitude', 0.0)
        if not (math.isfinite(latitude) and math.isfinite(longitude)):
            return TriggerResponse(
                success=False,
                message='set ~target_latitude and ~target_longitude first')
        ok, message = self.publish_goal(latitude, longitude, altitude)
        return TriggerResponse(success=ok, message=message)

    def publish_goal(self, latitude, longitude, altitude):
        if not (math.isfinite(latitude) and math.isfinite(longitude)):
            self.publish_status('rejected: latitude/longitude is not finite', False)
            return False, 'latitude/longitude is not finite'
        if not (-90.0 <= latitude <= 90.0 and -180.0 <= longitude <= 180.0):
            self.publish_status('rejected: latitude/longitude out of range', False)
            return False, 'latitude/longitude out of range'
        if not self.has_origin or self.origin_ecef is None:
            self.publish_status('rejected: ENU origin not available yet', False)
            rospy.logwarn('geo_goal_publisher: no ENU origin, goal dropped')
            return False, 'ENU origin not available'

        altitude = altitude if math.isfinite(altitude) else 0.0
        target_ecef = geodetic_to_ecef(latitude, longitude, altitude)
        east, north, up = ecef_to_enu(
            target_ecef, self.origin_ecef, self.origin_latitude, self.origin_longitude)

        # ENU -> world 交给 TF，避免在这里重复对齐参数。
        # 这里直接取 TF 自己做旋转：tf2_ros.transform() 需要 tf2_geometry_msgs
        # 注册 PointStamped 的转换器，而 Noetic 的 Python 侧默认没有这项注册。
        try:
            transform = self.tf_buffer.lookup_transform(
                self.parent_frame, self.enu_frame, rospy.Time(0),
                rospy.Duration(self.tf_timeout))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as error:
            message = 'TF %s <- %s unavailable: %s' % (
                self.parent_frame, self.enu_frame, error)
            self.publish_status(message, False)
            rospy.logwarn('geo_goal_publisher: %s', message)
            return False, message

        rotation = quaternion_to_matrix(
            transform.transform.rotation.x, transform.transform.rotation.y,
            transform.transform.rotation.z, transform.transform.rotation.w)
        translation = transform.transform.translation
        world_x = (rotation[0][0] * east + rotation[0][1] * north
                   + rotation[0][2] * up + translation.x)
        world_y = (rotation[1][0] * east + rotation[1][1] * north
                   + rotation[1][2] * up + translation.y)
        world_z = (rotation[2][0] * east + rotation[2][1] * north
                   + rotation[2][2] * up + translation.z)

        world_point = PointStamped()
        world_point.header.stamp = rospy.Time.now()
        world_point.header.frame_id = self.parent_frame
        world_point.point.x = world_x
        world_point.point.y = world_y
        world_point.point.z = world_z
        for index in range(max(1, self.repeat_count)):
            self.goal_pub.publish(world_point)
            self.debug_pub.publish(world_point)
            if index + 1 < max(1, self.repeat_count):
                rospy.sleep(self.repeat_interval)

        message = ('goal lat=%.7f lon=%.7f -> enu=(%.2f, %.2f, %.2f) -> %s=(%.2f, %.2f, %.2f)'
                   % (latitude, longitude, east, north, up, self.parent_frame,
                      world_point.point.x, world_point.point.y, world_point.point.z))
        rospy.loginfo('geo_goal_publisher: %s', message)
        self.publish_status(message, True)
        return True, message

    def publish_status(self, message, success):
        prefix = 'ok: ' if success else 'error: '
        self.status_pub.publish(String(data=prefix + message))


def main():
    rospy.init_node('geo_goal_publisher')
    GeoGoalPublisher()
    rospy.spin()


if __name__ == '__main__':
    main()
