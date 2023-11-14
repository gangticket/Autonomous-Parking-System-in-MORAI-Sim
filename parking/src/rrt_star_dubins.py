#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

from dubins_path import Dubins, pify

import networkx as nx
import numpy as np
import matplotlib.pyplot as plt

np.random.seed(5)

class Obstacle(object):
    def __init__(self, x, y, r):
        self.x = x
        self.y = y
        self.r = r

    def plot(self, color='k'):
        theta = np.linspace(0, np.pi*2, num=30)
        x = self.x + self.r * np.cos(theta)
        y = self.y + self.r * np.sin(theta)

        plt.plot(x, y, color=color)

    def is_inside(self, x, y):
        dist = np.hypot(x - self.x, y - self.y)
        if dist <= self.r:
            return True
        else:
            return False


class RRTStar(object):
    def __init__(self, start, goal, config):
        self.G = nx.DiGraph()

        node_attrb = {
            'cost': 0,
            'x': start[0],
            'y': start[1],
            'yaw': start[2]
        }

        self.G.add_nodes_from([
            (-1, node_attrb)
        ])

        self.start = start
        self.goal = goal

        self.config = config

    def sample_free(self, obstacles, space):
        min_x, max_x, min_y, max_y = space
        if np.random.rand() > self.config["goal_sample_rate"]:
            rand_x = np.random.uniform(min_x, max_x)
            rand_y = np.random.uniform(min_y, max_y)
            rand_yaw = np.random.uniform(0, 2*np.pi)
            return np.array([rand_x, rand_y, rand_yaw])

        else:
            return self.goal

    def get_nearest(self, rand_node):
        # node: np.array with 2 elements
        min_dist = 1e10
        nearest_node_id = None
        for v in self.G.nodes:
            node = self.G.nodes[v]
            dist = np.hypot(rand_node[0] - node['x'], rand_node[1] - node['y'])
            if dist < min_dist:
                nearest_node_id = v
                min_dist = dist

        return nearest_node_id

    def steer(self, node_from, node_to, u=None):
        dubins = Dubins()
        curvature = 1.0/2.0
        path, _, dubins_path = dubins.plan([node_from[0], node_from[1], node_from[2]],
                                           [node_to[0], node_to[1], node_to[2]], curvature)
        if path is None:
            return None

        path_length = dubins_path.length()
        path_x, path_y, path_yaw = path

        # node_attribute = {
        #     'x': node_to[0],
        #     'y': node_to[1],
        #     'yaw': node_to[2],
        #     'path_x': path_x,
        #     'path_y': path_y,
        #     'path_yaw': path_yaw,
        #     'path_length': path_length,
        #     'cost': 0.0
        # }

        return [node_to[0], node_to[1], node_to[2]]

    def get_node(self, node_id):
        node_state = np.array([self.G.nodes[node_id]['x'], self.G.nodes[node_id]['y'], self.G.nodes[node_id]['yaw']])
        return node_state

    def is_collision_free(self, node_from, node_to, obstacles):
        dubins = Dubins() 
        curvature = 1.0/2.0
        path, _, dubins_path = dubins.plan([node_from[0], node_from[1], node_from[2]],
                                           [node_to[0], node_to[1], node_to[2]], curvature)

        path_x, path_y, path_yaw = path

        for x, y in zip(path_x, path_y):
            node_to_check = np.array([x, y])

            for i, obs in enumerate(obstacles):
                col = obs.is_inside(node_to_check[0], node_to_check[1])
                if col:
                    return False

        return True

    def get_near_node_ids(self, new_node, draw=False):
        card_v = len(list(self.G.nodes))
        radius = np.amin([
            self.config["gamma_rrt_star"] * np.sqrt(np.log(card_v) / card_v),
            self.config["eta"]
        ])

        if draw:
            theta = np.linspace(0, np.pi*2, num=30)
            x = new_node[0] + radius * np.cos(theta)
            y = new_node[1] + radius * np.sin(theta)

            plt.plot(x, y, 'g--', alpha=0.3)

        near_node_ids = []
        for v in self.G.nodes:
            node = self.G.nodes[v]
            dist = np.hypot(new_node[0] - node['x'], new_node[1] - node['y'])
            if dist < radius:
                near_node_ids.append(v)

        return near_node_ids

    def add_node(self, node_id, x, y, yaw):
        self.G.add_node(node_id, x=x, y=y, yaw=yaw)

    def get_node_cost(self, node_id):
        return self.G.nodes[node_id]['cost']

    def get_distance(self, node_from_id, node_to_id):
        node_from = self.G.nodes[node_from_id]
        node_to = self.G.nodes[node_to_id]

        dx = node_to['x'] - node_from['x']
        dy = node_to['y'] - node_from['y']
        return np.hypot(dx, dy)

    def add_edge(self, node_from_id, node_to_id, path):
        # print(" node_from_id: %d, node_to_id: %d " % (node_from_id, node_to_id))
        self.G.add_edge(node_from_id, node_to_id, path=path)

    def set_node_cost(self, node_id, cost):
        self.G.nodes[node_id]['cost'] = cost

    def get_parent(self, node_id):
        parents = list(self.G.predecessors(node_id))
        if len(parents) > 0:
            return parents[0]
        else:
            return None

    def remove_edge(self, node_from_id, node_to_id):
        self.G.remove_edge(node_from_id, node_to_id)

    def check_goal_by_id(self, node_id):
        node = self.G.nodes[node_id]

        dx = node['x'] - self.goal[0]
        dy = node['y'] - self.goal[1]
        dist = np.hypot(dx, dy)

        if dist < 1:
            return True
        else:
            return False