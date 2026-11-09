#!/usr/bin/env python3
"""Publish a 2D-only upright preview; keep raw images and calibration unchanged."""

import copy
import json
from pathlib import Path

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image


def main():
    root = Path(__file__).resolve().parent
    with (root / "orientation.json").open(encoding="utf-8") as stream:
        rotation = json.load(stream)["rotation_degrees"]
    if rotation not in (0, 180):
        raise ValueError("rotation_degrees must be 0 or 180")
    cv2.setNumThreads(1)
    rospy.init_node("d435i_upright_preview")
    publisher = rospy.Publisher("/camera/color/image_upright", Image, queue_size=2)

    def callback(msg):
        if not publisher.get_num_connections():
            return
        channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4,
                    "bgra8": 4, "mono8": 1}.get(msg.encoding)
        if channels is None:
            rospy.logwarn_throttle(10, "Unsupported preview encoding: %s", msg.encoding)
            return
        try:
            rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
            pixels = rows[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
            if rotation == 180:
                # Native rotation avoids NumPy's expensive strided pixel copy.
                pixels = cv2.rotate(pixels, cv2.ROTATE_180)
            result = Image()
            result.header = copy.deepcopy(msg.header)
            # This display-only frame has no TF or CameraInfo; it is not calibrated.
            result.header.frame_id = "camera_color_upright_image"
            result.height = msg.height
            result.width = msg.width
            result.encoding = msg.encoding
            result.is_bigendian = msg.is_bigendian
            result.step = msg.width * channels
            result.data = np.ascontiguousarray(pixels).tobytes()
            publisher.publish(result)
        except ValueError as error:
            rospy.logerr_throttle(10, "Invalid color image: %s", error)

    rospy.Subscriber("/camera/color/image_raw", Image, callback,
                     queue_size=4, buff_size=32 * 1024 * 1024,
                     tcp_nodelay=True)
    rospy.loginfo("Upright preview: /camera/color/image_upright; rotation=%d degrees", rotation)
    rospy.spin()


if __name__ == "__main__":
    main()
