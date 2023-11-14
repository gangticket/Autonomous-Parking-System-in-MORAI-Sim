#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
import sys
import os
import copy
import numpy as np
import json
import pickle

from math import cos,sin,sqrt,pow,atan2,pi
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry,Path
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)

from lib.mgeo.class_defs import *

# mgeo_dijkstra_path_1 은 Mgeo 데이터를 이용하여 시작 Node 와 목적지 Node 를 지정하여 Dijkstra 알고리즘을 적용하는 예제 입니다.
# 사용자가 직접 지정한 시작 Node 와 목적지 Node 사이 최단 경로 계산하여 global Path(전역경로) 를 생성 합니다.

# 노드 실행 순서 
# 1. Mgeo data 읽어온 후 데이터 확인
# 2. 시작 Node 와 종료 Node 정의
# 3. weight 값 계산
# 4. Dijkstra Path 초기화 로직
# 5. Dijkstra 핵심 코드
# 6. node path 생성
# 7. link path 생성
# 8. Result 판별
# 9. point path 생성
# 10. dijkstra 경로 데이터를 ROS Path 메세지 형식에 맞춰 정의
# 11. dijkstra 이용해 만든 Global Path 정보 Publish


class dijkstra_path_pub :
    def __init__(self):
        rospy.init_node('dijkstra_path_pub', anonymous=True)
        rospy.Subscriber("driving_mode", String, self.mode_callback)
        self.global_path_pub = rospy.Publisher('/global_path',Path, queue_size = 1)

        #TODO: (1) Mgeo data 읽어온 후 데이터 확인
        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/kcity'))
        mgeo_planner_map = MGeoPlannerMap.create_instance_from_json(load_path)

        node_set = mgeo_planner_map.node_set
        link_set = mgeo_planner_map.link_set

        self.is_mode = False
        self.curr_mode = ""
        self.nodes=node_set.nodes
        self.links=link_set.lines

        self.global_planner=Dijkstra(self.nodes,self.links)

        #TODO: (2) 시작 Node 와 종료 Node 정의
        self.node_list = ['A119BS010209', 'A119BS010314', 'A119BS010324', 'A119BS010184'] #start, tollgate, end
        # self.start_node ='A119BS010209' # original : 'A119BS010184' 
        # self.end_node = 'A119BS010184' # original : 'A119BS010148'

        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = '/map'

        self.global_path_msg = self.calc_dijkstra_path_node(self.node_list)
        self.enter_parking_lot_path()

        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            if self.is_mode == True and self.curr_mode == "HIGHWAY MODE":
                #TODO: (11) dijkstra 이용해 만든 Global Path 정보 Publish
                self.global_path_pub.publish(self.global_path_msg)
            print(self.curr_mode)
            rate.sleep()

    def mode_callback(self, data):
        self.is_mode = True
        self.curr_mode = data.data
    
    def enter_parking_lot_path(self):
        with open("/home/ubuntu/cmaker_ws/src/path_planning/data/data_1011_v2.pickle", "rb") as f:
            out_path = pickle.load(f)
            path_x = pickle.load(f)
            path_y = pickle.load(f)
            path_yaw = pickle.load(f)

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
            self.global_path_msg .poses.append(read_pose)

    def calc_dijkstra_path_node(self, node_list):
        #TODO: (10) dijkstra 경로 데이터를 ROS Path 메세지 형식에 맞춰 정의
        out_path = Path()
        out_path.header.frame_id = '/map'

        for i in range(len(node_list) - 1):
            print("load from : ", node_list[i],", to : ", node_list[i+1])
            result, path = self.global_planner.find_shortest_path(node_list[i], node_list[i + 1])

            for waypoint in path["point_path"] :
                path_x = waypoint[0]
                path_y = waypoint[1]
                read_pose = PoseStamped()
                read_pose.pose.position.x = path_x
                read_pose.pose.position.y = path_y
                read_pose.pose.orientation.w = 1
                out_path.poses.append(read_pose)

        return out_path

class Dijkstra:
    def __init__(self, nodes, links):
        self.nodes = nodes
        self.links = links
        self.weight = self.get_weight_matrix()
        self.lane_change_link_idx = []

    def get_weight_matrix(self):
        #TODO: (3) weight 값 계산
        # 초기 설정
        weight = dict() 
        for from_node_id, from_node in self.nodes.items():
            # 현재 노드에서 다른 노드로 진행하는 모든 weight
            weight_from_this_node = dict()
            for to_node_id, to_node in self.nodes.items():
                weight_from_this_node[to_node_id] = float('inf')
            # 전체 weight matrix에 추가
            weight[from_node_id] = weight_from_this_node

        for from_node_id, from_node in self.nodes.items():
            # 현재 노드에서 현재 노드로는 cost = 0
            weight[from_node_id][from_node_id] = 0

            for to_node in from_node.get_to_nodes():
                # 현재 노드에서 to_node로 연결되어 있는 링크를 찾고, 그 중에서 가장 빠른 링크를 찾아준다
                shortest_link, min_cost = self.find_shortest_link_leading_to_node(from_node,to_node)
                weight[from_node_id][to_node.idx] = min_cost           

        return weight
    
    def find_shortest_link_leading_to_node(self, from_node, to_node):
        """현재 노드에서 to_node로 연결되어 있는 링크를 찾고, 그 중에서 가장 빠른 링크를 찾아준다"""
        #TODO: (3) weight 값 계산
        # NOTE: 
        to_links = []
        for link in from_node.get_to_links():
            if link.to_node is to_node:
                to_links.append(link)

        if len(to_links) == 0:
            raise BaseException('[ERROR] Error @ Dijkstra.find_shortest_path : Internal data error. There is no link from node (id={}) to node (id={})'.format(self.idx, to_node.idx))

        shortest_link = None
        min_cost = float('inf')
        for link in to_links:
            if link.cost < min_cost:
                min_cost = link.cost
                shortest_link = link

        return shortest_link, min_cost
        
    def find_nearest_node_idx(self, distance, s):        
        idx_list = list(self.nodes.keys())
        min_value = float('inf')
        min_idx = idx_list[-1]

        for idx in idx_list:
            if distance[idx] < min_value and s[idx] == False :
                min_value = distance[idx]
                min_idx = idx
        return min_idx

    def find_shortest_path(self, start_node_idx, end_node_idx): 
        #TODO: (4) Dijkstra Path 초기화 로직
        # s 초기화         >> s = [False] * len(self.nodes)
        # from_node 초기화 >> from_node = [start_node_idx] * len(self.nodes)
        s = dict()
        from_node = dict() 
        for node_id in self.nodes.keys():
            s[node_id] = False
            from_node[node_id] = start_node_idx

        s[start_node_idx] = True
        distance =copy.deepcopy(self.weight[start_node_idx])

        #TODO: (5) Dijkstra 핵심 코드
        for i in range(len(self.nodes.keys()) - 1):
            selected_node_idx = self.find_nearest_node_idx(distance, s)
            s[selected_node_idx] = True            
            for j, to_node_idx in enumerate(self.nodes.keys()):
                if s[to_node_idx] == False:
                    distance_candidate = distance[selected_node_idx] + self.weight[selected_node_idx][to_node_idx]
                    if distance_candidate < distance[to_node_idx]:
                        distance[to_node_idx] = distance_candidate
                        from_node[to_node_idx] = selected_node_idx

        #TODO: (6) node path 생성
        tracking_idx = end_node_idx
        node_path = [end_node_idx]
        
        while start_node_idx != tracking_idx:
            tracking_idx = from_node[tracking_idx]
            node_path.append(tracking_idx)     

        node_path.reverse()

        #TODO: (7) link path 생성
        link_path = []
        for i in range(len(node_path) - 1):
            from_node_idx = node_path[i]
            to_node_idx = node_path[i + 1]

            from_node = self.nodes[from_node_idx]
            to_node = self.nodes[to_node_idx]

            shortest_link, min_cost = self.find_shortest_link_leading_to_node(from_node, to_node)
            link_path.append(shortest_link.idx)

        #TODO: (8) Result 판별
        if len(link_path) == 0:
            return False, {'node_path': node_path, 'link_path':link_path, 'point_path':[]}

        #TODO: (9) point path 생성
        point_path = []

        # [USER OPTION] 차선 변경하기 위한 경로를 생성하여 point_path에 반영한다
        user_option_draw_lane_change = True
        
        for link_id in link_path:
            link = self.links[link_id]
            if link.is_it_for_lane_change() and user_option_draw_lane_change:  
                # 차선 변경 링크이고, 그 차선 변경이 여러개 차선을 건너는 것일때,
                # 이를 한번에 차선 변경하는 것으로 간주하여 point를 계산한다
                lane_ch_pair_list = link.get_lane_change_pair_list()
                lane_ch_first = lane_ch_pair_list[0]
                lane_ch_last = lane_ch_pair_list[-1]
                lane_ch_distance = 20 * link.get_number_of_lane_change()

                output_path = self.draw_lange_change(
                    lane_ch_first['from'], lane_ch_last['to'], lane_ch_distance, 1)
        
                output_path = np.array(output_path)
                x = output_path[:,0]
                y = output_path[:,1]
                for i in range(len(x)):
                    point_path.append([x[i], y[i], 0])

            else:
                # 차선 변경 링크가 아닌 경우
                for point in link.points:
                    point_path.append([point[0], point[1], 0])

        return True, {'node_path': node_path, 'link_path':link_path, 'point_path':point_path}
    

    def draw_lange_change(self, start_link, end_link, lane_change_distance, step_size):
        output_path = []


        translation = [start_link.points[0][0], start_link.points[0][1]]
        theta = atan2(start_link.points[1][1] - start_link.points[0][1], start_link.points[1][0] - start_link.points[0][0])

        t = np.array([
                    [cos(theta), -sin(theta),translation[0]],
                    [sin(theta),  cos(theta),translation[1]],
                    [0         ,  0         ,1            ]])

        det_t = np.array([
                        [t[0][0], t[1][0], -(t[0][0]*translation[0] + t[1][0]*translation[1])],
                        [t[0][1], t[1][1], -(t[0][1]*translation[0] + t[1][1]*translation[1])],
                        [0      , 0      , 1                                                 ]])

        world_end_link_list = []
        for point in end_link.points :
            world_end_link_list.append([point[0], point[1], 1])

        world_end_link_metrix = np.array(world_end_link_list).T
        local_end_link_metrix = det_t.dot(world_end_link_metrix).T

        min_dis=float('inf')
        local_end_point_list = []

        
        for point in local_end_link_metrix:
            if point[0] > 0:
                dis = abs(sqrt(point[0]*point[0] + point[1]*point[1]) - lane_change_distance)
                if dis < min_dis:
                    min_dis = dis
                    local_end_point_list = [[point[0]], [point[1]] ,[1]]
           
     
        local_end_point_matrix = np.array(local_end_point_list)
   
        
        x=[]
        y=[]
        x_interval=step_size
        xs=0
        xf=local_end_point_matrix[0][0]

        ps=0.0
        pf=local_end_point_matrix[1][0]
        
        x_num = xf / x_interval
        for i in range(xs, int(x_num)): 
            x.append(i * x_interval)

        a = [0.0, 0.0, 0.0, 0.0]
        a[0] = ps
        a[1] = 0
        a[2] = 3.0 * (pf - ps) / (xf**2)
        a[3] = -2.0 * (pf - ps) / (xf**3)
        for i in x:
            result=a[3]*i**3 + a[2]*i**2 + a[1]*i + a[0]
            y.append(result)

        for i in range(0, len(y)) :
            local_change_path = np.array([[x[i]],[y[i]],[1]])
            global_change_path = t.dot(local_change_path)
            # print([global_change_path[0][0],global_change_path[1][0],0])
            output_path.append([global_change_path[0][0], global_change_path[1][0],0])


        end_point_index = 0
        for (i,end_point) in enumerate(local_end_link_metrix.tolist()):
            if end_point[0] == local_end_point_matrix[0][0] and end_point[1] == local_end_point_matrix[1][0] :
                end_point_index = i
                break
        

        for end_point in end_link.points[end_point_index:]:
            # print([end_point[0],end_point[1],0])
            output_path.append([end_point[0],end_point[1],0])

        return output_path

if __name__ == '__main__':
    
    dijkstra_path_pub = dijkstra_path_pub()
