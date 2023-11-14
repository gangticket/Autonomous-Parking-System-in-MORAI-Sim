#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import math

import random

import rospy
import sys
import os
import copy
from morai_msgs.msg import EgoVehicleStatus, ObjectStatusList, EventInfo, CtrlCmd
from morai_msgs.srv import MoraiEventCmdSrv
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from Obstacle import RectangleObstacle, CircleObstacle
from hybrid_a_star_ros import *
import matplotlib.pyplot as plt
from enum import Enum
from hybrid_a_star_ros_pub import hybrid_a_star_path_pub
from reed_sheep_curve_pub import reed_sheep_curve_pub
from rrt_star_dubins_pub import RRTStarDubinsPathPub


class ParkingMode(Enum):
    PARKING_ROUGH_MODE = 1
    PARKING_ACCURATE_MODE = 2


class parking:
    def __init__(self):
        rospy.init_node("parking", anonymous=True)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/Object_topic", ObjectStatusList, self.object_status_callback)  # add
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.ego_callback)
        rospy.Subscriber("driving_mode", String, self.mode_callback)
        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd', CtrlCmd, queue_size=1)
        self.ctrl_cmd_msg = CtrlCmd()
        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = '/map'
        self.g_x = 10.3133
        self.g_y = 1034.85
        self.g_yaw = 0

        self.CURRENT_MODE = ParkingMode.PARKING_ROUGH_MODE
        self.path_pub_count = 0
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_yaw = 0
        self.obstacle_list = []
        self.is_obstacle = False
        self.is_odom = False
        self.is_mode = False
        self.curr_mode = ""

        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            if self.is_mode == True and self.curr_mode == "PARKING MODE":
                if self.is_obstacle == True and self.is_odom == True:
                    self.global_path_pub.publish(self.global_path_msg)
                    rate.sleep()
                    # Get Rough Path
                    if self.CURRENT_MODE == ParkingMode.PARKING_ROUGH_MODE and self.path_pub_count == 0:
                        self.hybrid_a_star_publisher = hybrid_a_star_path_pub(self.g_x, self.g_y, self.g_yaw,
                                                                              self.curr_x, self.curr_y, self.curr_yaw,
                                                                              self.obstacle_list)
                        self.global_path_msg = self.hybrid_a_star_publisher.calc_hybrid_a_star_path_node(
                            epsilon_pos=0.1, epsilon_yaw=0.1)
                        self.path_pub_count += 1
                        

                    # Follow Rough Path
                    elif self.CURRENT_MODE == ParkingMode.PARKING_ROUGH_MODE and self.path_pub_count == 1:
                        print("[ ", self.CURRENT_MODE, "] : Way To Rough Goal Point")
                        # Goal Check
                        if self.isNearbyGoal(epsilon_position=0.1):
                            print("Reach Rough Goal Point!!")
                            # Stop
                            self.send_ctrl_cmd(0,0)
                            self.CURRENT_MODE = ParkingMode.PARKING_ACCURATE_MODE
                            break
                    self.global_path_pub.publish(self.global_path_msg)
                    rate.sleep()
                else :
                    print("Now is Highway mode.")
        '''     
                            """
                            while( abs(self.ego_status.velocity.x) > 0.1):
                                self.stop()
                                #Change Mode
                                if (self.ego_status.velocity.x + self.ego_status.velocity.y) / 2 < 0.5 :
                                    self.CURRENT_MODE = ParkingMode.PARKING_ACCURATE_MODE
                            """
                else:
                    print("[ ", self.CURRENT_MODE, "] : Waiting for Loading Obstacles and Odom")
                # global path pub
                self.global_path_pub.publish(self.global_path_msg)
                rate.sleep()
        else:
            print("Not Parking MODE")

            # Get Accurate Path
        print("[ ", self.CURRENT_MODE, "] : Waiting for RRT")
        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = '/map'
        self.rrt_star_dubins_path_publisher = RRTStarDubinsPathPub(self.g_x, self.g_y, self.g_yaw, self.curr_x,
                                                                   self.curr_y, self.obstacle_list)
        self.global_path_msg = self.rrt_star_dubins_path_publisher.calc_rrtstar_dubins_path()
        if len(self.global_path_msg.poses) == 0:
            print("Fail to finding RRT Dubins path, Retrying with Hybrid A*")
            self.hybrid_a_star_publisher_2 = hybrid_a_star_path_pub(self.g_x, self.g_y, self.g_yaw, self.curr_x,
                                                                    self.curr_y, self.curr_yaw, self.obstacle_list)
            self.global_path_msg = self.hybrid_a_star_publisher_2.calc_hybrid_a_star_path_node(epsilon_pos=1.5,
                                                                                               epsilon_yaw=np.pi)

        # Follow Accurate Path
        while not rospy.is_shutdown():
            print("[ ", self.CURRENT_MODE, "] : Way To Accurate Goal Point")
            # Goal Check
            if self.isNearbyGoal(epsilon_position=0.3):
                print("Parking Complete!!")
                # Stop
                self.stop()
            self.global_path_pub.publish(self.global_path_msg)
            rate.sleep()
        '''
    def mode_callback(self, data):
        self.is_mode = True
        self.curr_mode = data.data

    def ego_callback(self, data):
        self.ego_status = data

    def odom_callback(self, msg):
        self.is_odom = True
        odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                           msg.pose.pose.orientation.w)
        _, _, self.curr_yaw = euler_from_quaternion(odom_quaternion)
        self.curr_x = msg.pose.pose.position.x
        self.curr_y = msg.pose.pose.position.y
        self.g_yaw = 1.634 - self.curr_yaw
    def object_status_callback(self, msg):

        # data.obstacle_list를 for loop통해 object.position.x, object.position.y를 self.obstacle_list의,x,y.z로 넣.
        # generate Rectangle Obstacle
        if msg.num_of_obstacle == 21 and self.is_obstacle == False:
            for object in msg.obstacle_list:
                if object.name == "OBJ_Kia_K7":
                    element = RectangleObstacle(center_x=object.position.x, center_y=object.position.y, width=4.96,
                                                height=2.06, angle=object.heading * (np.pi / 180))
                else:
                    element = CircleObstacle(x=object.position.x, y=object.position.y, r=1.5)
                # element.plot()
                self.obstacle_list.append(element)
            print("Success loading obstacles!!!")
            self.is_obstacle = True
            # for visualize path
            for obstacle in self.obstacle_list:  # obstacle visualize
                obstacle.plot()
            plt.savefig('/home/ubuntu/cmaker_ws/src/parkingplanner/data/CollisionImage.png')

    def isNearbyGoal(self, epsilon_position=0.1):
        distance = np.sqrt((self.g_x - self.curr_x) ** 2 + (self.g_y - self.curr_y) ** 2)

        if distance < epsilon_position:
            return True
        else:
            return False
        
    def send_ctrl_cmd(self, steering, velocity):
        cmd = CtrlCmd()
        if (velocity > 0):
            cmd.longlCmdType = 2
            cmd.velocity = velocity
            cmd.steering = steering
        else:
            cmd.longlCmdType = 1
            cmd.brake = 1
            cmd.steering = 0
        self.ctrl_cmd_pub(cmd)

        '''
    def stop(self):
        self.ctrl_cmd_msg.longlCmdType = 1
        self.ctrl_cmd_msg.brake = 1
        self.ctrl_cmd_msg.steering = 0
        for _ in range(10):
            self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
        '''

if __name__ == '__main__':
    parking = parking()