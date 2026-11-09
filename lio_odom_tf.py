#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

def main():
    rospy.init_node("lio_odom_tf")
    broadcaster = tf2_ros.TransformBroadcaster()

    def on_odom(msg):
        if msg.header.frame_id != "world":
            rospy.logwarn_throttle(5, "Expected /lio/odom in world frame")
            return

        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = "world"
        tf_msg.child_frame_id = "lio_pose"

        p = msg.pose.pose.position
        tf_msg.transform.translation.x = p.x
        tf_msg.transform.translation.y = p.y
        tf_msg.transform.translation.z = p.z
        tf_msg.transform.rotation = msg.pose.pose.orientation

        broadcaster.sendTransform(tf_msg)

    rospy.Subscriber("/lio/odom", Odometry, on_odom, queue_size=10)
    rospy.spin()

if __name__ == "__main__":
    main()
