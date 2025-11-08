#!/usr/bin/env python
# -*- coding: utf-8 -*-

# -------------------------------
# OpenCV, ROS를 이용한 차선 인식 노드 (Melodic 버전)
# 작성자: DD
# 설명: ROS 카메라 토픽(/camera/color/image_raw)으로부터 이미지를 받아
#       흰색/노란색 차선을 감지하고, 화면에 표시하는 코드입니다.
# -------------------------------

import cv2                  # OpenCV 라이브러리 (영상 처리용)
import numpy as np          # 수학 연산 (배열, 행렬 등)
import rospy                # ROS Python 클라이언트 라이브러리
from sensor_msgs.msg import Image       # ROS의 이미지 메시지 타입
from cv_bridge import CvBridge, CvBridgeError  # ROS Image <-> OpenCV 변환 도구

# -------------------------------
# LaneNode 클래스 정의 (ROS 노드)
# -------------------------------
class LaneNode:
    def __init__(self):
        # ROS 노드 초기화 ("lane_detection_node"라는 이름으로 실행)
        rospy.init_node("lane_detection_node", anonymous=True)

        # CvBridge 객체 생성 (ROS Image 메시지를 OpenCV 이미지로 바꿔줌)
        self.bridge = CvBridge()

        # ROS Subscriber: 카메라 이미지 토픽 구독
        # "/camera/color/image_raw" → 카메라 노드에서 송신하는 이미지 데이터
        # callback 함수: 새 이미지가 들어올 때마다 호출됨
        self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback, queue_size=1)

        # 터미널에 로그 출력 (노드 실행 시작 알림)
        rospy.loginfo("Lane detection node started (Melodic).")

        # 노드를 계속 실행 상태로 유지 (종료될 때까지 대기)
        rospy.spin()

    # -------------------------------
    # 콜백 함수 (새 이미지 수신 시 실행됨)
    # -------------------------------
    def callback(self, msg):
        try:
            # ROS 이미지 메시지를 OpenCV 형식으로 변환 ("bgr8" = 컬러)
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            # 변환 중 에러 발생 시 에러 로그 출력
            rospy.logerr(e)
            return

        # BGR(기본 색공간) → HSV 색공간으로 변환
        # HSV는 색깔(Hue), 채도(Saturation), 밝기(Value)로 구성되어 색 분리하기 편함
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 흰색 차선 마스크 설정 (밝은 영역)
        # HSV 범위: (H=0~180, S=0~30, V=200~255)
        mask_white = cv2.inRange(hsv, (0, 0, 200), (180, 30, 255))

        # 노란색 차선 마스크 설정
        # HSV 범위: (H=15~35, S=100~255, V=100~255)
        mask_yellow = cv2.inRange(hsv, (15, 100, 100), (35, 255, 255))

        # 흰색 + 노란색 마스크를 합침 (bitwise_or)
        mask = cv2.bitwise_or(mask_white, mask_yellow)

        # 원본 이미지에 마스크를 적용하여 차선 부분만 남김
        masked = cv2.bitwise_and(frame, frame, mask=mask)

        # 컬러 이미지를 흑백으로 변환 (엣지 검출 전처리)
        gray = cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY)

        # Canny 엣지 검출 알고리즘 (차선의 윤곽선 검출)
        # 첫 번째, 두 번째 인자는 임계값 (작은 값과 큰 값)
        edges = cv2.Canny(gray, 100, 200)

        # 허프 변환 (Hough Line Transform)을 이용해 직선 검출
        # rho=1, theta=1도 단위, threshold=50 (검출 기준 강도)
        # minLineLength=50: 최소 길이, maxLineGap=20: 선 사이 허용 간격
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, minLineLength=50, maxLineGap=20)

        # 검출된 직선이 있으면 이미지에 그리기
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]  # 한 직선의 양 끝 좌표
                # 녹색 선(두께 2픽셀)으로 화면에 표시
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # 최종 결과를 화면에 표시
        cv2.imshow("Lane Detection", frame)

        # 키 입력을 1ms 동안 대기 (화면 갱신용)
        cv2.waitKey(1)

# -------------------------------
# 메인 함수
# -------------------------------
if __name__ == "__main__":
    try:
        LaneNode()  # 노드 실행
    except rospy.ROSInterruptException:
        # Ctrl+C 등으로 노드 종료 시 예외 처리
        pass
