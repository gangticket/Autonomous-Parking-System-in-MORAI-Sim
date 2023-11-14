#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from math import cos,sin,pi,sqrt,pow,atan2
from morai_msgs.msg  import EgoVehicleStatus,ObjectStatusList
from std_msgs.msg import Bool


class ForwardObjectDetector:
    def __init__(self):
        rospy.init_node('object_detector', anonymous=True)

        # (1) subscriber, publisher 선언
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        rospy.Subscriber("/Object_topic", ObjectStatusList, self.object_callback)
        
        self.object_detect_pub = rospy.Publisher('/object_detect', Bool, queue_size=1)

        self.is_status = False
        self.is_obj = False
        self.object_check = False
        self.cnt = 0

        rate = rospy.Rate(5) # 30hz
        while not rospy.is_shutdown():
            if self.is_status and self.is_obj:
                if self.checkObject(self.object_data):
                    obj_msg = Bool()
                    obj_msg.data = True
                    print("충돌 위험 감지!!")
                    self.object_detect_pub.publish(obj_msg)
            else:
                print("check path and ego and object topic")
            rate.sleep()

    def checkObject(self, object_data):
        self.object_check = False
        heading_angle = self.status_msg.heading
        for obstacle in object_data.npc_list:
            x_dis = self.status_msg.position.x - obstacle.position.x
            y_dis = self.status_msg.position.y - obstacle.position.y
            
            if -140 < heading_angle and heading_angle < -70: # 고속도로 주행
                print("HIGHWAY DRIVING")
                dis = sqrt(pow(self.status_msg.position.x - obstacle.position.x, 2) + pow(self.status_msg.position.y - obstacle.position.y, 2))
                if dis < 15.0:
                    # if (-7.0 < x_dis and x_dis < 7.0 and -2.0 < y_dis) or (-0.5 < x_dis and x_dis < 0.5 and y_dis < 2.0):
                    if ((-7.0 < x_dis and x_dis < 7.0 and -5.0 < y_dis) or  (-0.5 < x_dis and x_dis < 0.5 and y_dis < 2.0) ) :
                        print("OBJECT DETECT_HIGHWAY : ", self.cnt)
                        print("x_dis , y_dis = ", x_dis , " , ", y_dis)
                        self.object_check = True
                        self.cnt += 1
            
            elif 70 < heading_angle and heading_angle < 130: # 회전교차로 주행
                print("ROUNDABOUT DRIVING")
                dis = sqrt(pow(self.status_msg.position.x - obstacle.position.x, 2) + pow(self.status_msg.position.y - obstacle.position.y, 2))
                if dis < 30.0: 
                    # if x_dis > 15.0 and y_dis > -5.0:
                    print("OBJECT DETECT_ROUNDABOUT", self.cnt)
                    self.object_check = True
                    self.cnt += 1
            return self.object_check
        
    def status_callback(self,msg): ## Vehicl Status Subscriber 
        self.is_status = True
        self.status_msg = msg

    def object_callback(self,msg):
        self.is_obj = True
        self.object_data = msg

if __name__ == '__main__':
    try:
        ForwardObjectDetector()
    except rospy.ROSInterruptException:
        pass