#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import os
from morai_msgs.msg import EgoVehicleStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler


import numpy as np

# https://cpb-us-e2.wpmucdn.com/faculty.sites.uci.edu/dist/e/700/files/2014/04/Dubins_Set_Robotics_2001.pdf
# https://github.com/hbanzhaf/steering_functions/blob/master/src/dubins_state_space/dubins_state_space.cpp

class dubins_path_pub:
    def __init__(self):
        rospy.init_node('dubins_path_pub', anonymous=True)

        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)

        self.kappa_ = 1. / 10 # max steering
        self.dubins = Dubins()
        self.current_position = PoseStamped().pose.position
        self.vehicle_yaw = 0
        # Start state definition.
        self.start_state = [0., 0., 0.]
        self.flag=0
        # Goal state definition.
        gx, gy = -2.57, 1023.95 
        gtheta = -2.092

        self.goal_state = [gx, gy, gtheta]
        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = '/map'

        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            self.global_path_pub.publish(self.global_path_msg)
            rate.sleep()
    
    def odom_callback(self, msg):
        odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                           msg.pose.pose.orientation.w)
        _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y
        
        sx, sy = self.current_position.x, self.current_position.y
    
        stheta = self.vehicle_yaw
        
        self.start_state[0], self.start_state[1], self.start_state[2] = sx, sy , stheta

        print("when stheta  =", stheta)
        # generate only one path when the car get in parking area. 
        if self.flag == 0 and stheta != 0.0 : 
            self.global_path_msg = self.calc_dubins_path_node()
            print("success to generate!!!!!!")
            self.flag =1
        
    def status_callback(self, msg):  ## Vehicle Status Subscriber
        self.status_msg=msg 
        
    def calc_dubins_path_node(self):

        cartesian_path, controls, dubins_path = self.dubins.plan(self.start_state, self.goal_state, self.kappa_)
        path_x, path_y, path_yaw = cartesian_path

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


def twopify(alpha):
    return alpha - np.pi * 2 * np.floor(alpha / (np.pi * 2))

def pify(alpha):
    v = np.fmod(alpha, 2*np.pi)
    if (v < - np.pi):
        v += 2 * np.pi
    else:
        v -= 2 * np.pi
    return v

class DubinsPath(object):
    def __init__(self, t=0, p=1e10, q=0, type=None):
        self.t = t
        self.p = p
        self.q = q
        self.length_ = [t, p, q]
        self.type = type
        self.controls = None

    def length(self):
        return self.t + self.p + self.q

class DubinsControl(object):
    def __init__(self):
        self.delta_s = 0.0
        self.kappa = 0.0


class Dubins(object):
    def __init__(self):
        self.constant = {
            "dubins_zero": -1e-9,
            "dubins_eps": 1e-6
        }

    def dubinsLSL(self, d, alpha, beta):
        ca, sa = np.cos(alpha), np.sin(alpha)
        cb, sb = np.cos(beta), np.sin(beta)
        tmp = 2 + d * d - 2 * (ca * cb + sa * sb - d * (sa - sb))
        if (tmp >= self.constant["dubins_zero"]):
            theta = np.arctan2(cb - ca, d + sa - sb)
            t = twopify(-alpha + theta)
            p = np.sqrt(np.amax([tmp, 0]))
            q = twopify(beta - theta)

            return DubinsPath(t, p, q, ["L", "S", "L"])

        else:
            return None

    def dubinsRSR(self, d, alpha, beta):
        ca, sa = np.cos(alpha), np.sin(alpha)
        cb, sb = np.cos(beta), np.sin(beta)
        tmp = 2. + d * d - 2. * (ca * cb + sa * sb - d * (sb - sa))
        if (tmp >= self.constant["dubins_zero"]):
            theta = np.arctan2(ca - cb, d - sa + sb)
            t = twopify(alpha - theta)
            p = np.sqrt(np.amax([tmp, 0]))
            q = twopify(-beta + theta)
            return DubinsPath(t, p, q, ["R", "S", "R"])
        else:
            return None

    def dubinsRSL(self, d, alpha, beta):
        ca, sa = np.cos(alpha), np.sin(alpha)
        cb, sb = np.cos(beta), np.sin(beta)
        tmp = d * d - 2. + 2. * (ca * cb + sa * sb - d * (sa + sb))
        if (tmp >= self.constant["dubins_zero"]):
            p = np.sqrt(np.amax([tmp, 0]))
            theta = np.arctan2(ca + cb, d - sa - sb) - np.arctan2(2., p)
            t = twopify(alpha - theta)
            q = twopify(beta - theta)
            return DubinsPath(t, p, q, ["R", "S", "L"])
        else:
            return None

    def dubinsLSR(self, d, alpha, beta):
        ca, sa = np.cos(alpha), np.sin(alpha)
        cb, sb = np.cos(beta), np.sin(beta)
        tmp = -2. + d * d + 2. * (ca * cb + sa * sb + d * (sa + sb))
        if (tmp >= self.constant["dubins_zero"]):
            p = np.sqrt(np.amax([tmp, 0]))
            theta = np.arctan2(-ca - cb, d + sa + sb) - np.arctan2(-2., p)
            t = twopify(-alpha + theta)
            q = twopify(-beta + theta)
            return DubinsPath(t, p, q, ["L", "S", "R"])
        else:
            return None

    def dubinsRLR(self, d, alpha, beta):
        ca, sa = np.cos(alpha), np.sin(alpha)
        cb, sb = np.cos(beta), np.sin(beta)
        tmp = .125 * (6. - d * d + 2. * (ca * cb + sa * sb + d * (sa - sb)))
        if (np.abs(tmp) < 1.):
            p = 2 * np.pi - np.arccos(tmp)
            theta = np.arctan2(ca - cb, d - sa + sb)
            t = twopify(alpha - theta + .5 * p)
            q = twopify(alpha - beta - t + p)
            return DubinsPath(t, p, q, ["R", "L", "R"])
        else:
            return None

    def dubinsLRL(self, d, alpha, beta):
        ca, sa = np.cos(alpha), np.sin(alpha)
        cb, sb = np.cos(beta), np.sin(beta)
        tmp = .125 * (6. - d * d + 2. * (ca * cb + sa * sb - d * (sa - sb)))
        if (np.abs(tmp) < 1.):
            p = 2 * np.pi - np.arccos(tmp)
            theta = np.arctan2(-ca + cb, d + sa - sb)
            t = twopify(-alpha + theta + .5 * p)
            q = twopify(beta - alpha - t + p)
            return DubinsPath(t, p, q, ["L", "R", "L"])
        else:
            return None

    def get_best_dubins_path(self, d, alpha, beta):
        dubins_functions = [
            self.dubinsLSL, self.dubinsRSR, self.dubinsRSL,
            self.dubinsLSR, self.dubinsRLR, self.dubinsLRL
        ]

        min_length = 1e10
        path = None
        for dubins_function in dubins_functions:
            tmp_path = dubins_function(d, alpha, beta)
            if tmp_path is not None:
                if (tmp_path.length() < min_length):
                    min_length = tmp_path.length()
                    path = tmp_path
        return path

    def plan(self, state1, state2, kappa):
        dx = state2[0] - state1[0]
        dy = state2[1] - state1[1]
        th = np.arctan2(dy, dx)

        d = np.hypot(dx, dy) * kappa
        alpha = twopify(state1[2] - th)
        beta = twopify(state2[2] - th)

        dubins_path = self.get_best_dubins_path(d, alpha, beta)
        controls = self.dubins_path_to_controls(dubins_path, kappa)
        cartesian_path = self.controls_to_cartesian_path(controls, state1)

        return cartesian_path, controls, dubins_path

    def dubins_path_to_controls(self, dubins_path, kappa):
        controls = []
        kappa_inv = 1.0/kappa

        if dubins_path is not None:
            for i in range(3):
                control = DubinsControl()
                type = dubins_path.type[i]
                length = dubins_path.length_[i]
                delta_s = kappa_inv * length

                control.delta_s = delta_s
                if (type == "L"):
                    control.kappa = kappa

                if (type == "S"):
                    control.kappa = 0

                if (type == "R"):
                    control.kappa = -kappa

                controls.append(control)
            return controls

        else:
            return None

    def controls_to_cartesian_path(self, controls, state1, discretization=0.1):
        if controls is None:
            return None

        x, y, yaw = state1
        xs, ys, yaws = [], [], []

        for control in controls:
            delta_s = control.delta_s
            abs_delta_s = np.abs(delta_s)
            kappa = control.kappa

            s_seg = 0
            integration_step = 0.0
            for j in range(int(np.ceil(abs_delta_s / discretization))):
                s_seg += discretization
                if (s_seg > abs_delta_s):
                    integration_step = discretization - (s_seg - abs_delta_s)
                    s_seg = abs_delta_s
                else:
                    integration_step = discretization

                if np.abs(kappa) > 0.0001:
                    x += 1/kappa * (-np.sin(yaw) + np.sin(yaw + integration_step * kappa))
                    y += 1/kappa * (np.cos(yaw) - np.cos(yaw + integration_step * kappa))
                    yaw = pify(yaw + integration_step * kappa)
                else:
                    x += integration_step * np.cos(yaw)
                    y += integration_step * np.sin(yaw)

                xs.append(x)
                ys.append(y)
                yaws.append(yaw)

        return xs, ys, yaws

