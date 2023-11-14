#!/usr/bin/env python
# -*- coding: utf-8 -*-

from rrt_star_dubins import Obstacle, RRTStar
from dubins_path import Dubins, pify
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from morai_msgs.msg import EgoVehicleStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import numpy as np
import matplotlib.pyplot as plt
import networkx as nx

class RRTStarDubinsPathPub :
    def __init__(self, gx_input, gy_input, sx_input, sy_input, syaw_input, obstacle_list_input):

        self.start_state = [sx_input, sy_input, syaw_input]
        self.obstacles = obstacle_list_input

        # Goal state definition.
        gtheta = -0.865
        self.goal_state = [gx_input, gy_input, gtheta]

    def calc_rrtstar_dubins_path(self):
        min_x, max_x = -17.29, 40.3
        min_y, max_y = 1013.86, 1075.85

        space = [min_x, max_x, min_y, max_y]
        start = self.start_state
        goal = self.goal_state

        config = {
            "eta": 10.0,
            "gamma_rrt_star": 10.0,
            "goal_sample_rate": 0.05,
        }

        rrt_star = RRTStar(start, goal, config)

        is_first_node = True
        goal_node_id = None
        out_path = None

        dubins = Dubins()
        kappa = 1/6.0

        for i in range(500):
            rand_node_state = rrt_star.sample_free(self.obstacles, space)
            # plt.plot(rand_node[0], rand_node[1], '.')

            nearest_node_id = rrt_star.get_nearest(rand_node_state)
            nearest_node_state = rrt_star.get_node(nearest_node_id)
            new_node_state = rrt_star.steer(nearest_node_state, rand_node_state)
            if new_node_state is None:
                continue
            # plt.plot(new_node[0], new_node[1], 's')

            if rrt_star.is_collision_free(nearest_node_state, new_node_state, self.obstacles):
                near_node_ids = rrt_star.get_near_node_ids(new_node_state, draw=True)
                path, _, dubins_path = dubins.plan(nearest_node_state, new_node_state, kappa)
                if path is not None:
                    rrt_star.add_node(i, x=new_node_state[0], y=new_node_state[1], yaw=new_node_state[2])
                    if is_first_node:
                        rrt_star.add_edge(-1, i, path)
                        is_first_node = False
                    plt.plot(new_node_state[0], new_node_state[1], 's')

                    min_node_id = nearest_node_id
                    # min_cost = rrt_star.get_node_cost(nearest_node_id) + rrt_star.get_distance(i, nearest_node_id)
                    min_cost = rrt_star.get_node_cost(nearest_node_id) + dubins_path.length()
                    min_path = path

                    # Connect along a minimum-cost path
                    for near_node_id in near_node_ids:
                        near_node_state = rrt_star.get_node(near_node_id)
                        if rrt_star.is_collision_free(near_node_state, new_node_state, self.obstacles):
                            path, _, dubins_path = dubins.plan(near_node_state, new_node_state, kappa)
                            if path is not None:
                                cost = rrt_star.get_node_cost(near_node_id) + dubins_path.length()
                                if cost < min_cost:
                                    min_node_id = near_node_id
                                    min_cost = cost
                                    min_path = path

                    if min_path is not None:
                        rrt_star.set_node_cost(i, min_cost)
                        rrt_star.add_edge(min_node_id, i, min_path)

                    # Rewire the tree
                    for near_node_id in near_node_ids:
                        near_node_state = rrt_star.get_node(near_node_id)
                        if rrt_star.is_collision_free(new_node_state, near_node_state, self.obstacles):
                            path, _, dubins_path = dubins.plan(new_node_state, near_node_state, kappa)
                            if path is not None:
                                cost = rrt_star.get_node_cost(i) + dubins_path.length()
                                if cost < rrt_star.get_node_cost(near_node_id):
                                    parent_node_id = rrt_star.get_parent(near_node_id)
                                    if parent_node_id is not None:
                                        rrt_star.remove_edge(parent_node_id, near_node_id)
                                        rrt_star.add_edge(i, near_node_id, path)

                    if rrt_star.check_goal_by_id(i):
                        goal_node_id = i
                        break

        # put path into dictionary datatype
        path_on_edge = {}
        for (u, v, path) in rrt_star.G.edges.data('path'):
            plt.plot(path[0], path[1], 'b-')
            path_on_edge[(u, v)] = path

        out_path = Path()
        out_path.header.frame_id = '/map'

        # select shortest path
        if goal_node_id is not None:
            path = nx.shortest_path(rrt_star.G, source=-1, target=goal_node_id)
            merged_xs = []
            merged_ys = []

            print(path)

            for node_idx in range(len(path)-1):
                node_id = path[node_idx+1]
                prev_node_id = path[node_idx]
                node = rrt_star.G.nodes[node_id]
                edge = path_on_edge[(prev_node_id, node_id)]
                merged_xs.extend(edge[0])
                merged_ys.extend(edge[1])


            # put shortest path in global_path
            for i in range(len(merged_xs)-1):
                pose = PoseStamped()
                pose.pose.position.x = merged_xs[i]
                pose.pose.position.y = merged_ys[i]
                pose.pose.orientation.x = 0
                pose.pose.orientation.y = 0
                pose.pose.orientation.z = 0
                pose.pose.orientation.w = 0
                out_path.poses.append(pose)
            return out_path
        else :
            print("no path!!!!")
            return out_path
