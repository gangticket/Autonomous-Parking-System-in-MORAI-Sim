#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import sys
import os
from morai_msgs.msg import EgoVehicleStatus, EventInfo, CtrlCmd
from morai_msgs.srv import MoraiEventCmdSrv
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from Obstacle import RectangleObstacle, CircleObstacle

from utils import *
import math
from enum import Enum
from dataclasses import dataclass, replace
import numpy as np

class reed_shepp_path_pub:
    def __init__(self):
        rospy.init_node('reed_shepp_path_pub', anonymous=True)

        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        self.cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)  # 추가된 부분

        rospy.wait_for_service('/Service_MoraiEventCmd')
        self.event_cmd_srv = rospy.ServiceProxy('Service_MoraiEventCmd', MoraiEventCmdSrv)


        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        self.vehicle_length = 3.16
        self.current_position = PoseStamped().pose.position
        self.vehicle_yaw = 0
        self.status_msg = None
        # Start state definition.
        self.start_state = [0., 0., 0.]
        self.flag = 0
 
        # Goal state definition.
        gx, gy = 14.815, 1031.320
        gtheta = 1.619

        self.goal_state = [gx, gy, gtheta]
        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = '/map'

        self.rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            self.global_path_pub.publish(self.global_path_msg)
            self.rate.sleep()

    def send_gear_cmd(self, gear_mode):
        # 기어 변경이 제대로 되기 위해서는 차량 속도가 약 0 이어야함
        while (abs(self.status_msg.velocity.x) > 0.1):
            self.send_ctrl_cmd(0, 0)
            self.rate.sleep()

        gear_cmd = EventInfo()
        gear_cmd.option = 3
        gear_cmd.ctrl_mode = 3
        gear_cmd.gear = gear_mode
        gear_cmd_resp = self.event_cmd_srv(gear_cmd)
        rospy.loginfo(gear_cmd)

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
        self.cmd_pub.publish(cmd)

        self.send_gear_cmd(Gear.D.value)
    def odom_callback(self, msg):
        odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                           msg.pose.pose.orientation.w)
        _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y

        sx, sy = self.current_position.x, self.current_position.y
        stheta = self.vehicle_yaw

        self.start_state[0], self.start_state[1], self.start_state[2] = sx, sy, stheta
        # generate only one path when the car get in parking area.
        if self.flag == 0 and stheta != 0.0:
            self.global_path_msg = self.calc_reed_shepp_path_node()
            print("success to generate!!!!!!")
            self.flag = 1

    def status_callback(self, msg):  ## Vehicle Status Subscriber
        self.status_msg = msg

    def calc_reed_shepp_path_node(self):
        opt_path = get_optimal_path(self.start_state, self.goal_state)
        out_path = Path()
        out_path.header.frame_id = '/map'

        x, y, yaw = self.current_position.x, self.current_position.y, self.vehicle_yaw
        for path_element in opt_path:
            if path_element.gear == Gear.D:
                gear = 1.
                self.send_gear_cmd(Gear.D.value)
            else:
                gear = -1.
                self.send_gear_cmd(Gear.R.value)
            if path_element.steering == Steering.LEFT:
                steering_angle = -math.pi / 4  # 왼쪽으로 45도 회전
            elif path_element.steering == Steering.RIGHT:
                steering_angle = math.pi / 4   # 오른쪽으로 45도 회전
            elif path_element.steering == Steering.STRAIGHT:
                steering_angle = 0  
            # Update vehicle state
            x += path_element.param * math.cos(yaw)
            y += path_element.param * math.sin(yaw)
            yaw += gear * path_element.param / self.vehicle_length * math.tan(steering_angle)

            # Convert state to ROS message
            read_pose = PoseStamped()
            read_pose.pose.position.x = x
            read_pose.pose.position.y = y
            quaternion = quaternion_from_euler(0., 0., yaw)

            read_pose.pose.orientation.x = quaternion[0]
            read_pose.pose.orientation.y = quaternion[1]
            read_pose.pose.orientation.z = quaternion[2]
            read_pose.pose.orientation.w = quaternion[3]

            out_path.poses.append(read_pose)
            print(out_path)
        return out_path

class Steering(Enum):
    LEFT = -1
    RIGHT = 1
    STRAIGHT = 0

class Gear(Enum):
    P = 1
    R = 2
    N = 3
    D = 4

@dataclass(eq=True)
class PathElement:
    param: float
    steering: Steering
    gear: Gear

    @classmethod
    def create(cls, param: float, steering: Steering, gear: Gear):
        if param >= 0:
            return cls(param, steering, gear)
        else:
            return cls(-param, steering, gear).reverse_gear()

    def __repr__(self):
        s = "{ Steering: " + self.steering.name + "\tGear: " + self.gear.name \
            + "\tdistance: " + str(round(self.param, 2)) + " }"
        return s

    def reverse_steering(self):
        steering = Steering(-self.steering.value)
        return replace(self, steering=steering)

    def reverse_gear(self):
        if self.gear == Gear.D:
            new_gear = Gear.R
        elif self.gear == Gear.R:
            new_gear = Gear.D
        else: # can add P or N gear
            new_gear = Gear.D

        return replace(self, gear=new_gear)


def path_length(path):
    """
    this one's obvious
    """
    return sum([e.param for e in path])


def get_optimal_path(start, end):
    """
    Return the shortest path from start to end among those that exist
    """
    paths = get_all_paths(start, end)
    return min(paths, key=path_length)


def get_all_paths(start, end):
    """
    Return a list of all the paths from start to end generated by the
    12 functions and their variants
    """
    path_fns = [path1, path2, path3, path4, path5, path6,
                path7, path8, path9, path10, path11, path12]
    paths = []

    # get coordinates of end in the set of axis where start is (0,0,0)
    x, y, theta = change_of_basis(start, end)

    for get_path in path_fns:
        # get the four variants for each path type, cf article
        paths.append(get_path(x, y, theta))
        paths.append(timeflip(get_path(-x, y, -theta)))
        paths.append(reflect(get_path(x, -y, -theta)))
        paths.append(reflect(timeflip(get_path(-x, -y, theta))))

    # remove path elements that have parameter 0
    for i in range(len(paths)):
        paths[i] = list(filter(lambda e: e.param != 0, paths[i]))

    # remove empty paths
    paths = list(filter(None, paths))

    return paths


def timeflip(path):
    """
    timeflip transform described around the end of the article
    """
    new_path = [e.reverse_gear() for e in path]
    return new_path


def reflect(path):
    """
    reflect transform described around the end of the article
    """
    new_path = [e.reverse_steering() for e in path]
    return new_path

def path1(x, y, phi):
    """
    Formula 8.1: CSC (same turns)
    """
    phi = deg2rad(phi)
    path = []

    u, t = R(x - math.sin(phi), y - 1 + math.cos(phi))
    v = M(phi - t)

    path.append(PathElement.create(t, Steering.LEFT, Gear.D))
    path.append(PathElement.create(u, Steering.STRAIGHT, Gear.D))
    path.append(PathElement.create(v, Steering.LEFT, Gear.D))

    return path


def path2(x, y, phi):
    """
    Formula 8.2: CSC (opposite turns)
    """
    phi = M(deg2rad(phi))
    path = []

    rho, t1 = R(x + math.sin(phi), y - 1 - math.cos(phi))

    if rho * rho >= 4:
        u = math.sqrt(rho * rho - 4)
        t = M(t1 + math.atan2(2, u))
        v = M(t - phi)

        path.append(PathElement.create(t, Steering.LEFT, Gear.D))
        path.append(PathElement.create(u, Steering.STRAIGHT, Gear.D))
        path.append(PathElement.create(v, Steering.RIGHT, Gear.D))

    return path


def path3(x, y, phi):
    """
    Formula 8.3: C|C|C
    """
    phi = deg2rad(phi)
    path = []

    xi = x - math.sin(phi)
    eta = y - 1 + math.cos(phi)
    rho, theta = R(xi, eta)

    if rho <= 4:
        A = math.acos(rho / 4)
        t = M(theta + math.pi/2 + A)
        u = M(math.pi - 2*A)
        v = M(phi - t - u)

        path.append(PathElement.create(t, Steering.LEFT, Gear.D))
        path.append(PathElement.create(u, Steering.RIGHT, Gear.R))
        path.append(PathElement.create(v, Steering.LEFT, Gear.D))

    return path


def path4(x, y, phi):
    """
    Formula 8.4 (1): C|CC
    """
    phi = deg2rad(phi)
    path = []

    xi = x - math.sin(phi)
    eta = y - 1 + math.cos(phi)
    rho, theta = R(xi, eta)

    if rho <= 4:
        A = math.acos(rho / 4)
        t = M(theta + math.pi/2 + A)
        u = M(math.pi - 2*A)
        v = M(t + u - phi)

        path.append(PathElement.create(t, Steering.LEFT, Gear.D))
        path.append(PathElement.create(u, Steering.RIGHT, Gear.R))
        path.append(PathElement.create(v, Steering.LEFT, Gear.R))

    return path


def path5(x, y, phi):
    """
    Formula 8.4 (2): CC|C
    """
    phi = deg2rad(phi)
    path = []

    xi = x - math.sin(phi)
    eta = y - 1 + math.cos(phi)
    rho, theta = R(xi, eta)

    if rho <= 4:
        u = math.acos(1 - rho*rho/8)
        A = math.asin(2 * math.sin(u) / rho)
        t = M(theta + math.pi/2 - A)
        v = M(t - u - phi)

        path.append(PathElement.create(t, Steering.LEFT, Gear.D))
        path.append(PathElement.create(u, Steering.RIGHT, Gear.D))
        path.append(PathElement.create(v, Steering.LEFT, Gear.R))

    return path


def path6(x, y, phi):
    """
    Formula 8.7: CCu|CuC
    """
    phi = deg2rad(phi)
    path = []

    xi = x + math.sin(phi)
    eta = y - 1 - math.cos(phi)
    rho, theta = R(xi, eta)

    if rho <= 4:
        if rho <= 2:
            A = math.acos((rho + 2) / 4)
            t = M(theta + math.pi/2 + A)
            u = M(A)
            v = M(phi - t + 2*u)
        else:
            A = math.acos((rho - 2) / 4)
            t = M(theta + math.pi/2 - A)
            u = M(math.pi - A)
            v = M(phi - t + 2*u)

        path.append(PathElement.create(t, Steering.LEFT, Gear.D))
        path.append(PathElement.create(u, Steering.RIGHT, Gear.D))
        path.append(PathElement.create(u, Steering.LEFT, Gear.R))
        path.append(PathElement.create(v, Steering.RIGHT, Gear.R))

    return path


def path7(x, y, phi):
    """
    Formula 8.8: C|CuCu|C
    """
    phi = deg2rad(phi)
    path = []

    xi = x + math.sin(phi)
    eta = y - 1 - math.cos(phi)
    rho, theta = R(xi, eta)
    u1 = (20 - rho*rho) / 16

    if rho <= 6 and 0 <= u1 <= 1:
        u = math.acos(u1)
        A = math.asin(2 * math.sin(u) / rho)
        t = M(theta + math.pi/2 + A)
        v = M(t - phi)

        path.append(PathElement.create(t, Steering.LEFT, Gear.D))
        path.append(PathElement.create(u, Steering.RIGHT, Gear.R))
        path.append(PathElement.create(u, Steering.LEFT, Gear.R))
        path.append(PathElement.create(v, Steering.RIGHT, Gear.D))

    return path


def path8(x, y, phi):
    """
    Formula 8.9 (1): C|C[pi/2]SC
    """
    phi = deg2rad(phi)
    path = []

    xi = x - math.sin(phi)
    eta = y - 1 + math.cos(phi)
    rho, theta = R(xi, eta)

    if rho >= 2:
        u = math.sqrt(rho*rho - 4) - 2
        A = math.atan2(2, u+2)
        t = M(theta + math.pi/2 + A)
        v = M(t - phi + math.pi/2)

        path.append(PathElement.create(t, Steering.LEFT, Gear.D))
        path.append(PathElement.create(math.pi/2, Steering.RIGHT, Gear.R))
        path.append(PathElement.create(u, Steering.STRAIGHT, Gear.R))
        path.append(PathElement.create(v, Steering.LEFT, Gear.R))

    return path


def path9(x, y, phi):
    """
    Formula 8.9 (2): CSC[pi/2]|C
    """
    phi = deg2rad(phi)
    path = []

    xi = x - math.sin(phi)
    eta = y - 1 + math.cos(phi)
    rho, theta = R(xi, eta)

    if rho >= 2:
        u = math.sqrt(rho*rho - 4) - 2
        A = math.atan2(u+2, 2)
        t = M(theta + math.pi/2 - A)
        v = M(t - phi - math.pi/2)

        path.append(PathElement.create(t, Steering.LEFT, Gear.D))
        path.append(PathElement.create(u, Steering.STRAIGHT, Gear.D))
        path.append(PathElement.create(math.pi/2, Steering.RIGHT, Gear.D))
        path.append(PathElement.create(v, Steering.LEFT, Gear.R))

    return path


def path10(x, y, phi):
    """
    Formula 8.10 (1): C|C[pi/2]SC
    """
    phi = deg2rad(phi)
    path = []

    xi = x + math.sin(phi)
    eta = y - 1 - math.cos(phi)
    rho, theta = R(xi, eta)

    if rho >= 2:
        t = M(theta + math.pi/2)
        u = rho - 2
        v = M(phi - t - math.pi/2)

        path.append(PathElement.create(t, Steering.LEFT, Gear.D))
        path.append(PathElement.create(math.pi/2, Steering.RIGHT, Gear.R))
        path.append(PathElement.create(u, Steering.STRAIGHT, Gear.R))
        path.append(PathElement.create(v, Steering.RIGHT, Gear.R))

    return path


def path11(x, y, phi):
    """
    Formula 8.10 (2): CSC[pi/2]|C
    """
    phi = deg2rad(phi)
    path = []

    xi = x + math.sin(phi)
    eta = y - 1 - math.cos(phi)
    rho, theta = R(xi, eta)

    if rho >= 2:
        t = M(theta)
        u = rho - 2
        v = M(phi - t - math.pi/2)

        path.append(PathElement.create(t, Steering.LEFT, Gear.D))
        path.append(PathElement.create(u, Steering.STRAIGHT, Gear.D))
        path.append(PathElement.create(math.pi/2, Steering.LEFT, Gear.D))
        path.append(PathElement.create(v, Steering.RIGHT, Gear.R))

    return path


def path12(x, y, phi):
    """
    Formula 8.11: C|C[pi/2]SC[pi/2]|C
    """
    phi = deg2rad(phi)
    path = []

    xi = x + math.sin(phi)
    eta = y - 1 - math.cos(phi)
    rho, theta = R(xi, eta)

    if rho >= 4:
        u = math.sqrt(rho*rho - 4) - 4
        A = math.atan2(2, u+4)
        t = M(theta + math.pi/2 + A)
        v = M(t - phi)

        path.append(PathElement.create(t, Steering.LEFT, Gear.D))
        path.append(PathElement.create(math.pi/2, Steering.RIGHT, Gear.R))
        path.append(PathElement.create(u, Steering.STRAIGHT, Gear.R))
        path.append(PathElement.create(math.pi/2, Steering.LEFT, Gear.R))
        path.append(PathElement.create(v, Steering.RIGHT, Gear.D))

    return path
if __name__ == "__main__":
    reed_shepp_path_pub = reed_shepp_path_pub()