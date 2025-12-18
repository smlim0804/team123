#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


class LineTracerWithObstacleAvoidance:
    def __init__(self):
        rospy.init_node("line_tracer_with_obstacle_avoidance")
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        rospy.Subscriber("/usb_cam/image_raw/compressed",
                         CompressedImage, self.camera_cb)
        rospy.Subscriber("/scan", LaserScan, self.lidar_cb)

        self.bridge = CvBridge()

        # LIDAR
        self.scan_ranges = []
        self.front = 999.0

        # 상태
        self.state = "LANE"
        self.escape_angle = 0.0
        self.state_start = rospy.Time.now().to_sec()

        # 라바콘 모드 여부
        self.in_cone_mode = False

        # 파라미터
        self.robot_width = 0.14
        self.base_gain = 1.0 / 220.0
        self.corner_scale = 140.0
        self.max_steer = 0.85

        self.left_delay_start = None
        self.left_delay_time = 0.6
        self.min_line_area = 300

    # ============================================================
    # LIDAR CALLBACK
    # ============================================================
    def lidar_cb(self, scan):
        raw = np.array(scan.ranges)
        self.scan_ranges = raw

        # 전방 ±10도
        front_zone = np.concatenate([raw[:10], raw[-10:]])
        cleaned = [d for d in front_zone if d > 0.20 and not np.isnan(d)]
        self.front = np.median(cleaned) if cleaned else 999.0

    # ============================================================
    # CAMERA CALLBACK
    # ============================================================
    def camera_cb(self, msg):
        twist = Twist()
        now = rospy.Time.now().to_sec()

        if self.state == "ESCAPE":
            self.escape_control()
            return

        if self.state == "BACK":
            self.back_control()
            return

        # ========================================================
        # LANE MODE
        # ========================================================
        if self.state == "LANE":

            # 아주 가까울 때만 후진
            if self.front < 0.30:
                rospy.loginfo("장애물 감지됨")
                self.state = "BACK"
                self.state_start = now
                self.left_delay_start = None
                self.in_cone_mode = False
                return

            frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            h, w = frame.shape[:2]
            roi = frame[int(h * 0.55):h, :]
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

            # 흰색 차선
            lower_white = np.array([0, 0, 180])
            upper_white = np.array([180, 40, 255])
            mask_white = cv2.inRange(hsv, lower_white, upper_white)

            contours, _ = cv2.findContours(
                mask_white, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
            )

            # 라바콘 (빨간색)
            lower_r1 = np.array([0, 120, 80])
            upper_r1 = np.array([10, 255, 255])
            lower_r2 = np.array([170, 120, 80])
            upper_r2 = np.array([180, 255, 255])

            mask_r1 = cv2.inRange(hsv, lower_r1, upper_r1)
            mask_r2 = cv2.inRange(hsv, lower_r2, upper_r2)
            red_mask = cv2.bitwise_or(mask_r1, mask_r2)

            red_contours, _ = cv2.findContours(
                red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            centers = []
            for cnt in red_contours:
                if cv2.contourArea(cnt) < 200:
                    continue
                M = cv2.moments(cnt)
                if M["m00"] == 0:
                    continue
                centers.append(int(M["m10"] / M["m00"]))

            # 라바콘 주행
            if len(centers) >= 1:
                rospy.loginfo("라바콘 감지")
                self.in_cone_mode = True

                if len(centers) >= 2:
                    centers.sort()
                    mid = (centers[0] + centers[-1]) // 2
                else:
                    mid = centers[0]

                error = mid - (w // 2)
                twist.linear.x = 0.21
                twist.angular.z = error / 180.0
                self.pub.publish(twist)
                return

            # 라바콘 종료
            self.in_cone_mode = False

            # ====================================================
            # 라인 안 보임 → 우회전 탐색 강화
            # ====================================================
            if len(contours) == 0:
                rospy.loginfo_throttle(1.0, "라인 없음")

                twist.linear.x = 0.12

                if self.in_cone_mode:
                    twist.angular.z = 0.0
                else:
                    twist.angular.z = -0.30  # 우회전 강화

                self.pub.publish(twist)
                return

            c = max(contours, key=cv2.contourArea)

            if cv2.contourArea(c) < self.min_line_area:
                twist.linear.x = 0.12
                twist.angular.z = 0.0
                self.pub.publish(twist)
                return

            # 라인트레이싱
            rospy.loginfo_throttle(1.0, "라인 추종")

            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                error = cx - (w // 2)

                if error > 0:
                    if self.left_delay_start is None:
                        self.left_delay_start = now
                    if now - self.left_delay_start < self.left_delay_time:
                        twist.linear.x = 0.22
                        twist.angular.z = 0.0
                        self.pub.publish(twist)
                        return
                else:
                    self.left_delay_start = None

                gain = self.base_gain * (1.0 + abs(error) / self.corner_scale)
                twist.linear.x = 0.22
                twist.angular.z = gain * error
                twist.angular.z = max(
                    min(twist.angular.z, self.max_steer),
                    -self.max_steer
                )
                self.pub.publish(twist)

    # ============================================================
    # BACK MODE
    # ============================================================
    def back_control(self):
        twist = Twist()
        now = rospy.Time.now().to_sec()

        if now - self.state_start < 1.4:
            twist.linear.x = -0.24
            twist.angular.z = 0.0
            self.pub.publish(twist)
        else:
            rospy.loginfo("회피 모드 진입")
            self.escape_angle = self.find_gap_max()
            self.state = "ESCAPE"
            self.state_start = now

    # ============================================================
    # ESCAPE MODE
    # ============================================================
    def escape_control(self):
        twist = Twist()
        now = rospy.Time.now().to_sec()

        if now - self.state_start < 1.0:
            twist.linear.x = 0.19
            twist.angular.z = self.escape_angle * 1.3
            self.pub.publish(twist)
        else:
            self.state = "LANE"

    # ============================================================
    # GAP 탐색
    # ============================================================
    def find_gap_max(self):
        if len(self.scan_ranges) == 0:
            return 0.0

        raw = np.array(self.scan_ranges)
        ranges = np.concatenate([raw[-60:], raw[:60]])
        ranges = np.where((ranges < 0.20) | np.isnan(ranges), 0.0, ranges)

        idx = np.argmax(ranges)
        angle_deg = idx - 60
        return angle_deg * np.pi / 180.0


if __name__ == "__main__":
    LineTracerWithObstacleAvoidance()
    rospy.spin()