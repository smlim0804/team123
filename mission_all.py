#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
import cv2
from cv_bridge import CvBridge
import numpy as np

class LimoAutonomousDrive:
    def __init__(self):
        rospy.init_node("limo_autonomous_drive")
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)

        self.bridge = CvBridge()
        self.cmd = Twist()
        self.mode = "LANE"   # 처음엔 차선 따라가기
        self.front_dist = 1.0

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # 색 공간 변환
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # ---------------------- (1) 흰색 차선 인식 ----------------------
        lower_white = np.array([0, 0, 180])
        upper_white = np.array([180, 60, 255])
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        # ---------------------- (2) 노란색 차선 인식 ----------------------
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # 모드 전환 조건
        if np.sum(mask_yellow) > 20000:  # 노란색 감지되면
            self.mode = "YELLOW"
        elif self.front_dist < 0.5:  # 라이다로 앞 벽 가까우면
            self.mode = "AVOID"
        else:
            self.mode = "LANE"

        # 모드별 제어
        if self.mode == "LANE":
            self.follow_line(mask_white, color="white")
        elif self.mode == "YELLOW":
            self.follow_line(mask_yellow, color="yellow")
        elif self.mode == "AVOID":
            self.avoid_wall()

    def lidar_callback(self, scan):
        # 라이다 앞부분 평균 거리 계산
        front_ranges = list(scan.ranges[0:10]) + list(scan.ranges[-10:])
        valid_ranges = [r for r in front_ranges if not np.isnan(r)]
        if valid_ranges:
            self.front_dist = sum(valid_ranges) / len(valid_ranges)
        else:
            self.front_dist = 1.0

    def follow_line(self, mask, color="white"):
        # 영상 중앙선 기준으로 라인 위치 찾기
        h, w = mask.shape
        roi = mask[int(h*0.6):, :]  # 아래쪽만 사용
        M = cv2.moments(roi)
        if M["m00"] > 0:
            cx = int(M["m10"]/M["m00"])
            error = cx - w/2

            self.cmd.linear.x = 0.18
            self.cmd.angular.z = -float(error) / 300
        else:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0

        rospy.loginfo(f"{color} line mode | error={error:.1f}")
        self.cmd_pub.publish(self.cmd)

    def avoid_wall(self):
        rospy.loginfo("Avoiding wall")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.5
        self.cmd_pub.publish(self.cmd)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    LimoAutonomousDrive().run()