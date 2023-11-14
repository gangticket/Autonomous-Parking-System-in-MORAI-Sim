#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from std_msgs.msg import String
import math

current_mode = "HIGHWAY MODE"
release_flag = False
def odom_callback(data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    
    parking_x = 8.46
    parking_y = 1066.64

    distance = math.sqrt((x - parking_x)**2 + (y - parking_y)**2)

    global current_mode
    if release_flag == True and current_mode == "WAITING MODE" :
        current_mode = "PARKING MODE"
    elif distance < 3.0 and current_mode == "HIGHWAY MODE":  # 일정 범위 내에 들어오고 현재 모드가 'PARKING MODE'가 아닌 경우
        current_mode = "WAITING MODE"

    if current_mode == "WAITING MODE" :
        print("[대기모드] 주차장 정보를 받아, 주차경로를 생성 중입니다.")
    elif current_mode == "PARKING MODE" :
        print("[주차모드] 생성된 주차경로를 따라 주차합니다.")
    else :
        print("[주행모드] 고속도로를 따라 주행합니다.")

    mode_msg = String()
    mode_msg.data = current_mode
    mode_publisher.publish(mode_msg)

def release_callback(data):
    global release_flag
    if data.data == "release" :
        print()
        print("[알림] 경로를 생성을 완료하였습니다. 주차모드로 전환합니다.")
        print() 
        release_flag = True

if __name__ == '__main__':
    rospy.init_node('parking_mode_detector')

    rospy.Subscriber('odom', Odometry, odom_callback)
    rospy.Subscriber('release_waiting_flag', String, release_callback)

    mode_publisher = rospy.Publisher('driving_mode', String, queue_size=10)

    rospy.spin()