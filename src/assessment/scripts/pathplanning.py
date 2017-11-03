#!/usr/bin/env python2
import rospy
import rospkg
import tf
import numpy as np
import math
import pickle
import heapq
from itertools import chain, permutations
from geometry_msgs.msg import Point, Twist
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Float32MultiArray


class Node:  # helper classes for both graph and nodes to make the representation easier
    def __init__(self, name, x_coord, y_coord):
        self.name = name
        self.x_coord = x_coord
        self.y_coord = y_coord
        self.neighbours = {}

    def add_neighbour(self, neighbour, cost):
        self.neighbours[neighbour] = cost

    def get_weight(self, neighbour):
        return self.neighbours[neighbour]

    def get_neighbours(self):
        return self.neighbours.keys()

    def get_name(self):
        return self.name


class Graph:
    def __init__(self):
        self.nodes = {}
        self.num_nodes = 0

    def add_node(self, vertex, x_coord, y_coord):
        new_node = Node(vertex, x_coord, y_coord)
        self.num_nodes += 1
        self.nodes[vertex] = new_node

    def get_node(self, vertex):
        return self.nodes[vertex]

    def add_edge(self, s, s_x, s_y, t, t_x, t_y, cost):
        if s not in self.nodes:
            self.add_node(s, s_x, s_y)
        if t not in self.nodes:
            self.add_node(t, t_x, t_y)
        self.nodes[s].add_neighbour(self.nodes[t], cost)

    def get_nodes(self):
        return self.nodes.keys()


def heuristic_estimate(start, end):
    s_x = start.x_coord
    s_y = start.y_coord
    f_x = end.x_coord
    f_y = end.y_coord

    return math.sqrt(abs(s_x - f_x)**2 + abs(s_y - f_y)**2)


class myRobot():
    def __init__(self):

        self.cells_array = pickle.load(
            open(rospkg.RosPack().get_path('assessment') + "/resources/cells.pickle", "rb"))

        self.neighbours = pickle.load(
            open(rospkg.RosPack().get_path('assessment') + "/resources/neighbours.pickle", "rb"))

        self.path = []

        self.best_path = []

        self.distance = 0

        self.order = []

        self.all_markers = []

        self.graph = Graph()

        self.init_graph()

        self.path_pub = rospy.Publisher(
            "/calculated_path", MarkerArray, queue_size=10)

        self.goals_pub = rospy.Publisher(
            "/goals_models", MarkerArray, queue_size=10)

        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.get_map)

        self.tf_listener = tf.TransformListener()

        self.robot_pose = Odometry()

    def init_graph(self):

        for i in range(0, len(self.cells_array)):
            self.graph.add_node(
                i, self.cells_array[i][0], self.cells_array[i][1])

        # Go through all identified cells.
        for i in range(0, len(self.cells_array)):
            # And for every cell, go through its neighbours.
            for j in range(0, len(self.neighbours[i])):
                dist_x = abs(
                    self.cells_array[i][0] - self.cells_array[list(self.neighbours[i])[j]][0])
                dist_y = abs(
                    self.cells_array[i][1] - self.cells_array[list(self.neighbours[i])[j]][1])
                s_x = self.cells_array[i][0]
                s_y = self.cells_array[i][1]
                t_x = self.cells_array[list(self.neighbours[i])[j]][0]
                t_y = self.cells_array[list(self.neighbours[i])[j]][1]

                self.graph.add_edge(
                    i, s_x, s_y, list(self.neighbours[i])[j], t_x, t_y, math.sqrt(dist_x**2 + dist_y**2))

    def find_shortest_path(self, s, t):

        start = self.graph.get_node(s)
        finish = self.graph.get_node(t)
        closedset = []
        came_from = {}
        g_score = {}
        g_score[start] = 0
        f_score = {}
        f_score[start] = g_score[start] + heuristic_estimate(start, finish)
        openset = [[f_score[start], start]]
        heapq.heapify(openset)

        while openset:
            current = heapq.heappop(openset)[1]
            if current == finish:
                break

            closedset.append(current)

            for neighbour in current.get_neighbours():

                if neighbour in closedset:
                    continue

                tentative_g_score = g_score[current] + \
                    current.get_weight(neighbour)

                if neighbour not in openset or tentative_g_score < g_score[neighbour]:
                    came_from[neighbour] = current
                    g_score[neighbour] = tentative_g_score
                    f_score[neighbour] = g_score[neighbour] + \
                        heuristic_estimate(neighbour, finish)

                    if neighbour not in list(chain.from_iterable(openset)):
                        heapq.heappush(
                            openset, [f_score[neighbour], neighbour])
                    else:
                        index = [i for i, x in enumerate(
                            openset) if x[1] == neighbour][0]
                        openset[index][0] = f_score[neighbour]

        current = finish
        total_path = [finish]
        distance = 0

        while current != start:
            try:
                current = came_from[current]
                total_path.append(current)
            except KeyError:
                self.distance = 999999999999
                self.path = []
                return

        for i in range(0, len(total_path) - 2):
            try:
                distance += total_path[i].get_weight(total_path[i + 1])
            except KeyError:
                distance += total_path[i + 1].get_weight(total_path[i])

        total_path = total_path[1:-1]
        self.distance += distance
        self.path = total_path

    def get_map(self, data):
        self.mapgrid = data
        self.start_x = data.info.origin.position.x
        self.start_y = data.info.origin.position.y
        self.resolution = data.info.resolution

    def get_path_permutation(self, start_x, start_y):
        goal_list = []
        goal_list_permutations = []
        distances = []
        for i in range(5):
            goal_list.append(rospy.get_param("/goal" + str(i)))

        ite = 0
        for x in list(permutations(goal_list)):
            goal_list_permutations.append(list(x))
            goal_list_permutations[ite].insert(0, [start_x, start_y])
            ite += 1

        for x in goal_list_permutations:
            self.graph = Graph()
            self.init_graph()
            for y in range(len(x) - 1):
                self.draw_path(x[y][0], x[y][1], x[y + 1][0], x[y + 1][1], y)
            distances.append(self.distance)
            self.distance = 0

        self.best_path = goal_list_permutations[distances.index(
            min(distances))]

    def find_closest_cell(self, x, y):
        min_dist = 9999
        cell_index = -1
        ite = 0
        for i in self.cells_array:
            dist = math.sqrt(i[0]**2 + i[1]**2)
            if((dist < min_dist) and
               (x in range(i[0] - i[2] / 2, i[0] + i[2] / 2) and
                    (y in range(i[1] - i[3] / 2, i[1] + i[3] / 2)))):
                cell_index = ite
                min_dist = dist
            ite += 1
        return cell_index

    def draw_path(self, sx, sy, fx, fy, id_num):

        cell_y = 0

        if sy < 0:
            cell_y = abs(sy) + 4.8
        else:
            cell_y = abs(sy - 4.8)

        sx_coord = int(round((sx + 6.0) / 0.012))
        sy_coord = int(round((cell_y) / 0.012))

        if fy < 0:
            cell_y = abs(fy) + 4.8
        else:
            cell_y = abs(fy - 4.8)

        fx_coord = int(round((fx + 6.0) / 0.012))
        fy_coord = int(round((cell_y) / 0.012))

        self.find_shortest_path(self.find_closest_cell(
            sy_coord, sx_coord), self.find_closest_cell(fy_coord, fx_coord))

        ma = MarkerArray()
        mr = Marker()
        mr.header.frame_id = "/map"
        mr.ns = "path"
        mr.id = id_num
        mr.type = mr.LINE_STRIP
        mr.action = mr.ADD

        mr.points = []

        mr.points.append(Point(x=fx, y=fy))

        for node in self.path:
            y_coord = -self.start_y - \
                (self.cells_array[node.get_name()][0] * self.resolution)
            x_coord = self.start_x + \
                (self.cells_array[node.get_name()][1] * self.resolution)
            mr.points.append(Point(x=x_coord, y=y_coord))
        mr.points.append(Point(x=sx, y=sy))

        mr.scale.x = 0.02

        mr.color.r = 1
        mr.color.g = 0
        mr.color.b = 0
        mr.color.a = 1.0
        ma = MarkerArray()
        ma.markers.append(mr)

        self.all_markers.append(ma)
        self.path_pub.publish(ma)

    def draw_goals(self, sx, sy):

        cur_x = sx
        cur_y = sy

        ma = MarkerArray()

        ite = 0

        self.all_markers = []

        for i in self.best_path:
            robot.draw_path(cur_x, cur_y, i[0], i[1], ite)

            cur_x = i[0]
            cur_y = i[1]

            mr = Marker()
            mr.header.frame_id = "/map"
            mr.ns = "goals"
            mr.id = ite
            mr.type = mr.CUBE
            mr.action = mr.ADD
            mr.pose.position.x = i[0] - 0.05
            mr.pose.position.y = i[1]
            mr.pose.position.z = 0.05
            mr.scale.x = 0.1
            mr.scale.y = 0.1
            mr.scale.z = 0.1
            mr.color.r = 0
            mr.color.g = 1
            mr.color.b = 0
            mr.color.a = 1.0
            ma.markers.append(mr)
            ite += 1
        self.goals_pub.publish(ma)

    def publish_best_path(self):
        pub1 = rospy.Publisher(
            "/best_path_x", Float32MultiArray, queue_size=10, latch=True)
        pub2 = rospy.Publisher(
            "/best_path_y", Float32MultiArray, queue_size=10, latch=True)

        message_x = Float32MultiArray()
        message_y = Float32MultiArray()

        for i in self.all_markers:
            for marker in i.markers:
                for point in marker.points[::-1]:
                    message_x.data.append(point.x)
                    message_y.data.append(point.y)

        pub1.publish(message_x)
        pub2.publish(message_y)


if __name__ == '__main__':
    try:
        rospy.init_node('pathplannig_node', anonymous=True)
        robot = myRobot()

        rate = rospy.Rate(5)

        got_permutations = False
        while not got_permutations:
            try:
                robot.get_path_permutation(-4.8, -3.6)
                got_permutations = True
            except:
                pass

        while not rospy.is_shutdown():

            robot.draw_goals(-4.8, -3.6)
            robot.publish_best_path()

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
