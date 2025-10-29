#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class LaneNode:
    def __init__(self):
        rospy.init_node("lane_detection_node", anonymous=True)
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback, queue_size=1)
        rospy.loginfo("Lane detection node started (Melodic).")
        rospy.spin()

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_white = cv2.inRange(hsv, (0, 0, 200), (180, 30, 255))
        mask_yellow = cv2.inRange(hsv, (15, 100, 100), (35, 255, 255))
        mask = cv2.bitwise_or(mask_white, mask_yellow)
        masked = cv2.bitwise_and(frame, frame, mask=mask)

        gray = cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 100, 200)

        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, minLineLength=50, maxLineGap=20)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        cv2.imshow("Lane Detection", frame)
        cv2.waitKey(1)

if __name__ == "__main__":
    try:
        LaneNode()
    except rospy.ROSInterruptException:
        pass
