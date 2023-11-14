#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import math

import random

import rospy
import sys
import os
import copy
from morai_msgs.msg import EgoVehicleStatus, ObjectStatusList
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from Obstacle import RectangleObstacle, CircleObstacle
from hybrid_a_star_ros import *
import matplotlib.pyplot as plt

class hybrid_a_star_path_pub:
    def __init__(self, gx_input, gy_input, gyaw_input , sx_input, sy_input, syaw_input, obstacle_list_input):

        self.current_position = PoseStamped().pose.position
        self.space = [-17.29, 40.3, 1013.86, 1075.85]
        self.obstacle_list = obstacle_list_input

        # Start state definition.
        self.start_state = [sx_input, sy_input, syaw_input]

        # Goal state definition.
        self.goal_state = [gx_input, gy_input, gyaw_input]

    def calc_hybrid_a_star_path_node(self, epsilon_pos = 3.0, epsilon_yaw = np.pi * 2):
        opt_path = a_star(self.start_state, self.goal_state, self.space, self.obstacle_list, epsilon_pos, epsilon_yaw, R=5.0, Vx=2.0, delta_time_step=0.5, weight=0.1)
        path_x, path_y, path_yaw = opt_path

        #visualize
        plt.plot(path_x, path_y, 'b.-', label='A* Path', markersize=5) #path visualize
        plt.plot(self.start_state[0], self.start_state[1], 'go', label='Start') # start point visualize
        plt.plot(self.goal_state[0], self.goal_state[1], 'ro', label='Goal') # goal point visualize
        plt.legend()
        plt.savefig('/home/ubuntu/cmaker_ws/src/parking/data/image.png')
        print("save image!!!!!!!!")

        out_path = Path()
        out_path.header.frame_id = '/map'

        for x, y, yaw in zip(path_x, path_y, path_yaw):
            read_pose = PoseStamped()
            read_pose.pose.position.x = x
            read_pose.pose.position.y = y
            quaternion = quaternion_from_euler(0., 0., yaw)

            read_pose.pose.orientation.x = quaternion[0]
            read_pose.pose.orientation.y = quaternion[1]
            read_pose.pose.orientation.z = quaternion[2]
            read_pose.pose.orientation.w = quaternion[3]
            print(read_pose)
            out_path.poses.append(read_pose)
        return out_path
