#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def mission1_zigzag():
    rospy.init_node('mission1_zigzag', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    cmd = Twist()
    linear_speed = 0.2
    angular_speed = 0.5
    forward_time = 2.0
    turn_time = 1.5

    rospy.loginfo("🚗 Mission1 Zigzag Start!")

    while not rospy.is_shutdown():
        # 직진
        cmd.linear.x = linear_speed
        cmd.angular.z = 0.0
        t0 = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t0 < forward_time:
            pub.publish(cmd)
            rate.sleep()

        # 좌회전
        cmd.linear.x = 0.0
        cmd.angular.z = angular_speed
        t0 = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t0 < turn_time:
            pub.publish(cmd)
            rate.sleep()

        # 직진
        cmd.linear.x = linear_speed
        cmd.angular.z = 0.0
        t0 = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t0 < forward_time:
            pub.publish(cmd)
            rate.sleep()

        # 우회전
        cmd.linear.x = 0.0
        cmd.angular.z = -angular_speed
        t0 = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t0 < turn_time:
            pub.publish(cmd)
            rate.sleep()

if __name__ == '__main__':
    try:
        mission1_zigzag()
    except rospy.ROSInterruptException:
        pass