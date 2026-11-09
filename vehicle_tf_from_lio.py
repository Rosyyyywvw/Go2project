#!/usr/bin/env python3
"""Publish planner and sensor TFs from Super-LIO's IMU pose and calibration.

world -> vehicle is the yaw-only virtual frame used by the local planner.
world -> imu_link is the full IMU pose. imu_link -> rslidar uses the same
LiDAR-to-IMU calibration as Super-LIO, and rslidar -> velodyne is identity
because the point converter copies XYZ without a geometric transformation.
"""

import math

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_matrix


def make_transform(stamp, parent, child, position, quaternion):
    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = parent
    transform.child_frame_id = child
    translation = transform.transform.translation
    translation.x, translation.y, translation.z = map(float, position)
    rotation = transform.transform.rotation
    rotation.x, rotation.y, rotation.z, rotation.w = map(float, quaternion)
    return transform


def lidar_static_transforms(values, stamp):
    calibration = np.asarray(values, dtype=float)
    if calibration.shape != (12,) or not np.isfinite(calibration).all():
        raise ValueError("lio/extrinsic/lidar_imu must contain 12 finite values")
    matrix = np.eye(4)
    # ROSWrapper.cpp constructs an Eigen matrix from these column-major values.
    matrix[:3, :3] = calibration[3:].reshape((3, 3), order="F")
    rotation = matrix[:3, :3]
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-4, rtol=0) or
            not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-4)):
        raise ValueError("lio/extrinsic/lidar_imu rotation is invalid")
    quaternion = quaternion_from_matrix(matrix)
    quaternion /= np.linalg.norm(quaternion)
    # A TF with parent IMU and child LiDAR maps LiDAR points into IMU coordinates:
    # p_IMU = R * p_LiDAR + t. Do not invert this calibration.
    return [
        make_transform(stamp, "imu_link", "rslidar", calibration[:3], quaternion),
        make_transform(stamp, "rslidar", "velodyne", (0, 0, 0), (0, 0, 0, 1)),
    ]


def main():
    rospy.init_node("vehicle_tf_from_lio")
    offset_x = float(rospy.get_param("/localPlanner/sensorOffsetX"))
    offset_y = float(rospy.get_param("/localPlanner/sensorOffsetY"))
    follower_x = float(rospy.get_param("/pathFollower/sensorOffsetX"))
    follower_y = float(rospy.get_param("/pathFollower/sensorOffsetY"))
    if (offset_x, offset_y) != (follower_x, follower_y):
        raise RuntimeError("localPlanner 和 pathFollower 的 sensorOffsetX/Y 不一致")

    broadcaster = tf2_ros.TransformBroadcaster()
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_broadcaster.sendTransform(lidar_static_transforms(
        rospy.get_param("/lio/extrinsic/lidar_imu"), rospy.Time.now()))

    def on_odom(msg):
        if msg.header.frame_id != "world" or msg.header.stamp == rospy.Time(0):
            rospy.logwarn_throttle(5, "Expected stamped /lio/odom in world frame")
            return

        position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        values = (position.x, position.y, position.z, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(value) for value in values):
            rospy.logwarn_throttle(5, "Ignoring non-finite /lio/odom pose")
            return
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm < 1e-9:
            rospy.logwarn_throttle(5, "Ignoring zero /lio/odom quaternion")
            return
        x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

        # localPlanner defines its virtual vehicle frame at the odometry height.
        # The URDF's IMU-to-base Z offset belongs to a physical base frame, not /path.
        vehicle_position = (position.x - math.cos(yaw) * offset_x + math.sin(yaw) * offset_y,
                            position.y - math.sin(yaw) * offset_x - math.cos(yaw) * offset_y,
                            position.z)
        vehicle_rotation = (0, 0, math.sin(yaw / 2), math.cos(yaw / 2))
        broadcaster.sendTransform([
            make_transform(msg.header.stamp, "world", "vehicle", vehicle_position, vehicle_rotation),
            make_transform(msg.header.stamp, "world", "imu_link", (position.x, position.y, position.z),
                           (x, y, z, w)),
        ])

    rospy.Subscriber("/lio/odom", Odometry, on_odom, queue_size=10)
    rospy.loginfo("Publishing world -> vehicle (offset X=%s, Y=%s), world -> imu_link; "
                  "calibrated imu_link -> rslidar -> velodyne", offset_x, offset_y)
    rospy.spin()


if __name__ == "__main__":
    main()
