#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
import time
import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import CtrlCmd,EgoVehicleStatus
from std_msgs.msg import String
from std_msgs.msg import Bool
import numpy as np
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from cartesian_to_frenet import *
from collections import deque
# import coordinate_convert as coordinate


# advanced_purepursuit 은 차량의 차량의 종 횡 방향 제어 예제입니다.
# Purpusuit 알고리즘의 Look Ahead Distance 값을 속도에 비례하여 가변 값으로 만들어 횡 방향 주행 성능을 올립니다.
# 횡방향 제어 입력은 주행할 Local Path (지역경로) 와 차량의 상태 정보 Odometry 를 받아 차량을 제어 합니다.
# 종방향 제어 입력은 목표 속도를 지정 한뒤 목표 속도에 도달하기 위한 Throttle control 을 합니다.
# 종방향 제어 입력은 longlCmdType 1(Throttle control) 이용합니다.

# 노드 실행 순서 
# 1. subscriber, publisher 선언
# 2. 속도 비례 Look Ahead Distance 값 설정
# 3. 좌표 변환 행렬 생성
# 4. Steering 각도 계산
# 5. PID 제어 생성
# 6. 도로의 곡률 계산
# 7. 곡률 기반 속도 계획
# 8. 제어입력 메세지 Publish


class pure_pursuit :
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        #TODO: (1) subscriber, publisher 선언
        rospy.Subscriber("/global_path", Path, self.global_path_callback)
        rospy.Subscriber("/local_path", Path, self.path_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/Ego_topic",EgoVehicleStatus, self.status_callback) 
        rospy.Subscriber("driving_mode", String, self.mode_callback)
        rospy.Subscriber("/object_detect", Bool, self.crash_callback)
        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd',CtrlCmd, queue_size=1)

        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1

        self.is_path = False
        self.is_odom = False 
        self.is_status = False
        self.is_global_path = False

        self.is_look_forward_point = False

        self.is_mode = False
        self.curr_mode = ""

        self.is_crash = False

        self.forward_point = Point()
        self.current_postion = Point()

        self.vehicle_length = 3.16
        self.lfd = 5
        if self.vehicle_length is None or self.lfd is None:
            print("you need to change values at line 57~58 ,  self.vegicle_length , lfd")
            exit()
        self.min_lfd = 2.5 # 5
        self.max_lfd = 30 # 30
        self.lfd_gain = 1.0 #0.78
        self.target_velocity = 50

        # control val - I term add
        self.i_gain = 0.05 #best = 0.05, 0.2 is worst
        self.i_control = 0
        prev_time = 0
        self.controlTime = 0.02
        self.i_control_history = deque(maxlen = 10)

        self.pid = pidControl()
        self.vel_planning = velocityPlanning(self.target_velocity/3.6, 0.15)
        while True:
                if self.is_global_path == True:
                    self.velocity_list = self.vel_planning.curvedBaseVelocity(self.global_path, 50)
                    break
                else:
                    print('Waiting global path data')

        rate = rospy.Rate(30) # 20hz
        while not rospy.is_shutdown():
            os.system('clear')
            if self.is_path == True and self.is_odom == True and self.is_status == True:

                # prev_time = time.time()
                cur_time = time.time()
                self.controlTime = cur_time - prev_time
                prev_time = cur_time

                self.current_waypoint = self.get_current_waypoint(self.status_msg,self.global_path)
                self.target_velocity = self.velocity_list[self.current_waypoint]*3.6
                if self.is_mode == True and self.curr_mode == "PARKING MODE":
                    self.target_velocity = 5
                if self.is_crash == True and self.curr_mode == "HIGHWAY MODE":
                    self.target_velocity = self.target_velocity - 20
                steering = self.calc_pure_pursuit()
                if self.is_look_forward_point :
                    self.ctrl_cmd_msg.steering = steering
                else : 
                    print("no found forward point")
                    self.ctrl_cmd_msg.steering = 0.0
                output = self.pid.pid(self.target_velocity,self.status_msg.velocity.x*3.6)

                if self.curr_mode == "WAITING MODE" :
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = 1.0
                else : 
                    if output > 0.0:
                        self.ctrl_cmd_msg.accel = output / 2
                        self.ctrl_cmd_msg.brake = 0.0
                    elif output > -5 and not self.target_velocity < 10:
                        self.ctrl_cmd_msg.accel = 0.0
                        self.ctrl_cmd_msg.brake = 0.0
                    else:
                        self.ctrl_cmd_msg.accel = 0.0 # 0.0
                        self.ctrl_cmd_msg.brake = -output# -output

                #TODO: (8) 제어입력 메세지 Publish
                # print(steering)
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
            else:
                print(f"self.is_path (/local_path) : {self.is_path}")
                print(f"self.is_status (/Ego_topic)  : {self.is_status}")
                print(f"self.is_odom (/odom)         : {self.is_odom}")

            self.is_crash = False
            rate.sleep()

    def mode_callback(self, data):
        self.is_mode = True
        self.curr_mode = data.data

    def crash_callback(self, data):
        self.is_crash = data.data

    def path_callback(self,msg):
        self.is_path=True
        self.path=msg  

    def odom_callback(self,msg):
        self.is_odom=True
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)
        self.current_postion.x=msg.pose.pose.position.x
        self.current_postion.y=msg.pose.pose.position.y

    def status_callback(self,msg): ## Vehicl Status Subscriber 
        self.is_status=True
        self.status_msg=msg    
        
    def global_path_callback(self,msg):
        self.global_path = msg
        self.is_global_path = True
    
    def get_current_waypoint(self,ego_status,global_path):
        min_dist = float('inf')        
        currnet_waypoint = -1
        for i,pose in enumerate(global_path.poses):
            dx = ego_status.position.x - pose.pose.position.x
            dy = ego_status.position.y - pose.pose.position.y

            dist = sqrt(pow(dx,2)+pow(dy,2))
            if min_dist > dist :
                min_dist = dist
                currnet_waypoint = i
        return currnet_waypoint

    def calc_pure_pursuit(self,):

        if self.i_control > 0.6 or self.i_control < -0.6: 
           self.i_control = 0
           print("*******************i reset!!************************")


        #TODO: (2) 속도 비례 Look Ahead Distance 값 설정
        self.lfd = (self.status_msg.velocity.x) * self.lfd_gain        
        
        if self.lfd < self.min_lfd : 
            self.lfd=self.min_lfd
        elif self.lfd > self.max_lfd :
            self.lfd=self.max_lfd
        rospy.loginfo(self.lfd)
        # print("lfd: ", self.lfd)
        
        vehicle_position=self.current_postion
        self.is_look_forward_point= False

        translation = [vehicle_position.x, vehicle_position.y]

        #TODO: (3) 좌표 변환 행렬 생성
        trans_matrix = np.array([
                [cos(self.vehicle_yaw), -sin(self.vehicle_yaw),translation[0]],
                [sin(self.vehicle_yaw),cos(self.vehicle_yaw),translation[1]],
                [0                    ,0                    ,1            ]])

        det_trans_matrix = np.linalg.inv(trans_matrix)

        for num,i in enumerate(self.path.poses) :
            path_point=i.pose.position

            global_path_point = [path_point.x,path_point.y,1]
            local_path_point = det_trans_matrix.dot(global_path_point)    

            if local_path_point[0]>0 :
                dis = sqrt(pow(local_path_point[0],2)+pow(local_path_point[1],2))
                if dis >= self.lfd :
                    self.forward_point = path_point
                    self.is_look_forward_point = True
                    break
        
        #TODO: (4) Steering 각도 계산
        if self.curr_mode == "WAITING MODE" :
            theta = atan2(2,3)
        else :
            theta = atan2(local_path_point[1],local_path_point[0])
            
        self.max_steering = 0.61 # (rad) 0.61 (deg) 35

        mapx = []
        mapy = []
        for i,pose in enumerate(self.global_path.poses):
            mapx.append(pose.pose.position.x)
            mapy.append(pose.pose.position.y)

        x = self.current_postion.x
        y = self.current_postion.y

        # convert vehicle state coordinate (global -> frenet)
        s, d = get_frenet(x, y, mapx, mapy)

        error = -d

        self.i_control = self.i_gain * error * self.controlTime
        self.i_control_history.append(self.i_control)
        sum_i_control = sum(self.i_control_history)

        steering = atan2((2 * self.vehicle_length * sin(theta)), (self.lfd)) + sum_i_control
        steering = max(-self.max_steering, min(self.max_steering, steering))

        if steering is None:
            print("[ERROR] you need to change pure_pursuit at line 179 : calcu_steering !")
            exit()

        return steering

class pidControl:
    def __init__(self):
        self.p_gain = 0.3
        self.i_gain = 0.00
        self.d_gain = 0.03
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.02

    def pid(self, target_vel, current_vel):
        error = target_vel - current_vel

        #TODO: (5) PID 제어 생성
        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error-self.prev_error) / self.controlTime

        output = p_control + self.i_control + d_control
        self.prev_error = error

        return output       

class velocityPlanning:
    def __init__ (self,car_max_speed, road_friciton):
        self.car_max_speed = car_max_speed
        self.road_friction = road_friciton

    def curvedBaseVelocity(self, gloabl_path, point_num):
        out_vel_plan = []

        for i in range(0,point_num):
            out_vel_plan.append(self.car_max_speed)

        for i in range(point_num, len(gloabl_path.poses) - point_num):
            x_list = []
            y_list = []
            for box in range(-point_num, point_num):
                x = gloabl_path.poses[i+box].pose.position.x
                y = gloabl_path.poses[i+box].pose.position.y
                x_list.append([-2*x, -2*y ,1])
                y_list.append((-x*x) - (y*y))

            #TODO: (6) 도로의 곡률 계산
            x_matrix = np.array(x_list)
            y_matrix = np.array(y_list)
            x_trans = x_matrix.T

            a_matrix = np.linalg.inv(x_trans.dot(x_matrix)).dot(x_trans).dot(y_matrix)
            a = a_matrix[0]
            b = a_matrix[1]
            c = a_matrix[2]
            r = sqrt(a*a+b*b-c)

            #TODO: (7) 곡률 기반 속도 계획
            v_max = sqrt(r*9.8*self.road_friction)

            if v_max > self.car_max_speed:
                v_max = self.car_max_speed
            out_vel_plan.append(v_max)

        for i in range(len(gloabl_path.poses) - point_num, len(gloabl_path.poses) - 10):
            out_vel_plan.append(3)

        for i in range(len(gloabl_path.poses) - 10, len(gloabl_path.poses)):
            out_vel_plan.append(0)

        return out_vel_plan

if __name__ == '__main__':
    try:
        test_track=pure_pursuit()
    except rospy.ROSInterruptException:
        pass
