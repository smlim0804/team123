#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
import cv2


class BlackLaneFollower:
    def __init__(self):
        rospy.init_node("black_lane_follower")

        rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback, queue_size=1)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.cmd = Twist()
        self.encoding = None

        # ===== 파라미터 =====
        self.forward_speed = 0.12
        self.search_spin_speed = 0.25
        self.k_angle = 0.010

        self.dark_min_pixels = 5
        self.dark_col_ratio = 0.3

        rospy.loginfo(" Black lane follower started")

    # Image → cv2
    
    def msg_to_cv2(self, msg: Image):
        if self.encoding is None:
            self.encoding = msg.encoding
            rospy.loginfo("📷 image encoding: %s", self.encoding)

        h, w = msg.height, msg.width

        if self.encoding in ("rgb8", "bgr8"):
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            img = arr.reshape(h, msg.step // 3, 3)[:, :w, :]
            if self.encoding == "rgb8":
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            return img

        if self.encoding == "mono8":
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            img = arr.reshape(h, msg.step)[:, :w]
            return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        return None

    
    # 검은 트랙 라인트레이싱
    def image_callback(self, msg: Image):
        img = self.msg_to_cv2(msg)
        if img is None:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = self.search_spin_speed
            return

        h, w, _ = img.shape
        center = w / 2.0

        # ROI (하단 50%)
        roi = img[int(h * 0.5):, :]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        # 검은 트랙 강조
        _, binary = cv2.threshold(
            gray, 0, 255,
            cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
        )

        kernel = np.ones((3, 3), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

        # 열별 검은 픽셀 수
        mask = (binary > 0)
        col_sum = np.sum(mask, axis=0)
        max_val = int(np.max(col_sum)) if col_sum.size > 0 else 0

        if max_val < self.dark_min_pixels:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = self.search_spin_speed
            return

        threshold_val = max(
            self.dark_min_pixels,
            int(max_val * self.dark_col_ratio)
        )
        candidates = np.where(col_sum >= threshold_val)[0]

        if candidates.size == 0:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = self.search_spin_speed
            return

        x = np.arange(len(col_sum))
        track_center = np.sum(x[candidates] * col_sum[candidates]) / np.sum(col_sum[candidates])

        offset = track_center - center
        ang = -self.k_angle * offset
        ang = max(min(ang, 0.8), -0.8)

        self.cmd.linear.x = self.forward_speed
        self.cmd.angular.z = ang

    # cmd_vel publish
    def spin(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.cmd_pub.publish(self.cmd)
            rate.sleep()


if __name__ == "__main__":
    node = BlackLaneFollower()
    try:
        node.spin()
    except rospy.ROSInterruptException:
        pass