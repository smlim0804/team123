#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def mission1_zigzag():
    # 노드 초기화: 'mission1_zigzag'라는 이름으로 ROS 노드를 시작한다.
    rospy.init_node('mission1_zigzag', anonymous=True)

    # /cmd_vel 토픽 퍼블리셔 생성
    # 로봇의 선속도(linear)와 각속도(angular)를 퍼블리시하는 용도
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # 반복 주기 설정: 10Hz → 초당 10번 명령을 보냄
    rate = rospy.Rate(10)

    # Twist 메시지 생성 (속도 명령 구조체)
    cmd = Twist()

    # 이동 속도 및 시간 파라미터 설정
    linear_speed = 0.2      # 직진 속도 (m/s)
    angular_speed = 0.5     # 회전 속도 (rad/s)
    forward_time = 2.0      # 직진 유지 시간 (초)
    turn_time = 1.5         # 회전 유지 시간 (초)

    rospy.loginfo("🚗 Mission1 Zigzag Start!")

    # ROS가 종료될 때까지 무한 반복
    while not rospy.is_shutdown():

        # -------------------------------
        # ① 직진 구간
        # -------------------------------
        cmd.linear.x = linear_speed   # 앞쪽으로 0.2 m/s
        cmd.angular.z = 0.0           # 회전 없음(직진)
        t0 = rospy.Time.now().to_sec()  # 현재 시작 시간 기록

        # forward_time(2초) 동안 계속 직진 명령을 반복해서 보냄
        while rospy.Time.now().to_sec() - t0 < forward_time:
            pub.publish(cmd)
            rate.sleep()

        # -------------------------------
        # ② 좌회전 구간
        # -------------------------------
        cmd.linear.x = 0.0            # 직진 정지
        cmd.angular.z = angular_speed # 반시계(+) 방향 회전
        t0 = rospy.Time.now().to_sec()

        # turn_time(1.5초) 동안 제자리에서 좌회전
        while rospy.Time.now().to_sec() - t0 < turn_time:
            pub.publish(cmd)
            rate.sleep()

        # -------------------------------
        # ③ 다시 직진 구간
        # -------------------------------
        cmd.linear.x = linear_speed   # 직진
        cmd.angular.z = 0.0
        t0 = rospy.Time.now().to_sec()

        while rospy.Time.now().to_sec() - t0 < forward_time:
            pub.publish(cmd)
            rate.sleep()

        # -------------------------------
        # ④ 우회전 구간
        # -------------------------------
        cmd.linear.x = 0.0             # 직진 정지
        cmd.angular.z = -angular_speed # 시계(-) 방향 회전
        t0 = rospy.Time.now().to_sec()

        # turn_time(1.5초) 동안 우회전
        while rospy.Time.now().to_sec() - t0 < turn_time:
            pub.publish(cmd)
            rate.sleep()

# 프로그램 시작점
if __name__ == '__main__':
    try:
        mission1_zigzag()
    except rospy.ROSInterruptException:
        # Ctrl+C 등으로 종료될 때 예외 방지용
        pass
