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

import matplotlib.pyplot as plt


show_animation = True

class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.heading = 0.0  # -> 연속 공간에서 영향?
        self.f = 0
        self.g = 0
        self.h = 0


# Check if position of node is same( if distance < threshold, regard as same node)
def isSamePosition(node_1, node_2, epsilon_position=0.3):
    distance = np.sqrt((node_1.position[0] - node_2.position[0]) ** 2 + (node_1.position[1] - node_2.position[1]) ** 2)

    if distance < epsilon_position:
        return True
    else:
        return False


def isSameYaw(node_1, node_2, epsilon_yaw=0.2):
    angle_difference = np.abs(node_1.position[2] - node_2.position[2])
    if angle_difference > np.pi:
        angle_difference = np.pi * 2 - angle_difference

    if angle_difference <= epsilon_yaw:
        return True
    else:
        return False


# Action set, Moving only forward direction
def get_action(R, Vx, delta_time_step):
    yaw_rate = Vx / R
    distance_travel = Vx * delta_time_step
    # yaw_rate, delta_time_step, cost
    action_set = [[yaw_rate, delta_time_step, distance_travel],
                  [-yaw_rate, delta_time_step, distance_travel],
                  [yaw_rate / 2, delta_time_step, distance_travel],
                  [-yaw_rate / 2, delta_time_step, distance_travel],
                  [0.0, delta_time_step, distance_travel]]
    return action_set


def vehicle_move(position_parent, yaw_rate, delta_time, Vx):
    x_parent = position_parent[0]
    y_parent = position_parent[1]
    yaw_parent = position_parent[2]

    if yaw_rate != 0:
        # left or right turn
        r = Vx / yaw_rate
        yaw_circle_pos = np.pi / 2 + yaw_parent
        x_circle = r * np.cos(yaw_circle_pos) + x_parent
        y_circle = r * np.sin(yaw_circle_pos) + y_parent

        yaw_child = yaw_rate * delta_time + yaw_parent  # heading angle
        x_child = r * np.cos(yaw_child - np.pi / 2) + x_circle  # heading angle -> 극 좌표계의 각도로 변환 : -pi/2
        y_child = r * np.sin(yaw_child - np.pi / 2) + y_circle

    else:
        # move straight
        r = Vx * delta_time

        yaw_child = yaw_rate * delta_time + yaw_parent
        x_child = r * np.cos(yaw_child) + x_parent
        y_child = r * np.sin(yaw_child) + y_parent

    # yaw processing
    if yaw_child > 2 * np.pi:
        yaw_child = yaw_child - 2 * np.pi
    if yaw_child < 0:
        yaw_child = yaw_child + 2 * np.pi
    # return position : [x, y, yaw]

    return [x_child, y_child, yaw_child]


# Collision check : path overlaps with any of obstacle
def collision_check(position_parent, yaw_rate, delta_time_step, obstacle_list, Vx):
    check_path = []

    for delta_time in np.linspace(0.0, delta_time_step, 5):  # -> 임의의 샘플 개수로 경로 생성
        check_path.append(vehicle_move(position_parent, yaw_rate, delta_time, Vx))

    # 경로 중 collision의 위치에 매우 근접할 때 True 반환
    for pos in check_path:
        for obstacle in obstacle_list:  # RectangleObstacle : (x, y, widht, height, angle)
            if obstacle.is_inside(pos[0], pos[1]) :
                return True
    x_list = [arr[0] for arr in check_path]
    y_list = [arr[1] for arr in check_path]
    #plt.plot(x_list, y_list, color='silver', linestyle='-')

    return False


# Check if the node is in the searching space
def isNotInSearchingSpace(position_child, space):
    # space: [min_x, max_x, min_y, max_y]
    if space[0] > position_child[0] or space[1] < position_child[0]:
        return True
    if space[2] > position_child[1] or space[3] < position_child[1]:
        return True

    return False


def isSeenNode(new_node, node_list):
    for i, node in enumerate(node_list):
        if isSamePosition(new_node, node):
            if isSameYaw(new_node, node, epsilon_yaw=0.2):
                return i
    return 0


def heuristic(cur_node, goal_node):
    dist = np.sqrt(
        (cur_node.position[0] - goal_node.position[0]) ** 2 + (cur_node.position[1] - goal_node.position[1]) ** 2)
    return dist


def a_star(start, goal, space, obstacle_list, epsilon_pos, epsilon_yaw, R, Vx, delta_time_step, weight):
    start_node = Node(None, start)
    goal_node = Node(None, goal)

    start_node.h = heuristic(start_node, goal_node)
    start_node.f = start_node.g + weight * start_node.h

    open_list = []
    closed_list = []

    open_list.append(start_node)
    while open_list is not None:
        cur_node = open_list[0]
        cur_index = 0

        # Find node with lowest cost
        for index, node in enumerate(open_list):
            if node.h < cur_node.h:
                cur_node = node
                cur_index = index

        # If goal, return optimal path
        if (isSamePosition(cur_node, goal_node, epsilon_position = epsilon_pos) and isSameYaw(cur_node, goal_node, epsilon_yaw)):
            opt_cost = 0.0
            opt_path = []
            opt_path_x = []
            opt_path_y = []
            opt_path_yaw = []
            node = cur_node

            while node is not None:
                opt_path.append(node.position)
                opt_path_x.append(node.position[0])
                opt_path_y.append(node.position[1])
                opt_path_yaw.append(node.position[2])
                opt_cost = opt_cost + node.g
                node = node.parent

            print(f"searching space : {len(closed_list)}")
            print(f"optimal cost : {opt_cost}")

            return list(reversed(opt_path_x)), list(reversed(opt_path_y)), list(reversed(opt_path_yaw))

        # If not goal, move from open list to closed list
        open_list.pop(cur_index)
        closed_list.append(cur_node)

        # Generate child candidate
        action_set = get_action(R, Vx, delta_time_step)
        for action in action_set:  # yaw_rate, delta_time_step, cost
            child_candidate_position = vehicle_move(cur_node.position, action[0], action[1], Vx)

            # If not in searching space, do nothing
            if isNotInSearchingSpace(child_candidate_position, space):
                continue
            # If collision expected, do nothing
            if collision_check(cur_node.position, action[0], action[1], obstacle_list, Vx):
                continue
            # If not collision, create child node
            child = Node(cur_node, child_candidate_position)
            # If already in closed list, do nothing
            if isSeenNode(child, closed_list):
                continue

            # If not in closed list, update open list
            child.g = cur_node.g + action[2]
            child.h = heuristic(child, goal_node)
            child.f = child.g + weight * child.h

            node_index = isSeenNode(child, open_list)
            if node_index:
                if child.f < open_list[node_index].f:
                    open_list[node_index].position = child.position
                    open_list[node_index].parent = child.parent
                    open_list[node_index].f = child.f
                    open_list[node_index].g = child.g
                    open_list[node_index].h = child.h
                    # plt.plot(open_list[node_index].position[0], open_list[node_index].position[1], color='black', marker='o', alpha=0.5)
                    # plt.plot([open_list[node_index].position[0], open_list[node_index].position[0]+0.2*np.cos(open_list[node_index].position[2])],[open_list[node_index].position[1], open_list[node_index].position[1]+0.2*np.sin(open_list[node_index].position[2])],color='black', linestyle='-')
                else:
                    continue
            else:
                open_list.append(child)
            
            plt.plot(child.position[0], child.position[1], color='silver', marker='o', alpha=0.1)
            plt.plot([child.position[0], child.position[0] + 0.2 * np.cos(child.position[2])],
                     [child.position[1], child.position[1] + 0.2 * np.sin(child.position[2])], color='silver',
                     linestyle='-', alpha=0.1)

        # show graph
        """
        if show_animation:
            plt.plot(cur_node.position[0], cur_node.position[1], 'ro', alpha=0.1)
            plt.plot([cur_node.position[0], cur_node.position[0] + 0.2 * np.cos(cur_node.position[2])],
                     [cur_node.position[1], cur_node.position[1] + 0.2 * np.sin(cur_node.position[2])], 'r-', alpha=0.1)

            if len(closed_list) % 100 == 0:
                plt.pause(0.1)
        """
def main():
    
    
    start, goal, obstacle_list, space = map()
    
    opt_path = a_star(start, goal, space, obstacle_list, R=5.0, Vx=2.0, delta_time_step=0.5, weight=0.1)
    print("Optimal path found!")
    opt_path = np.array(opt_path)
    print(opt_path)
    
    if show_animation == True:
        plt.plot(opt_path[:, 0], opt_path[:, 1], "m.-")
        # plt.show()
    
