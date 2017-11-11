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


# Heuristic estimation based on start and end position and their euclidian distance.
def heuristic_estimate(start, end):
    s_x = start.x_coord
    s_y = start.y_coord
    f_x = end.x_coord
    f_y = end.y_coord

    return math.sqrt(abs(s_x - f_x)**2 + abs(s_y - f_y)**2)


class myRobot():
    def __init__(self):

        # Load our pre-processed map data from pixeldraw.py script.
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

        # Publishers for our path and goal models.
        self.path_pub = rospy.Publisher(
            "/calculated_path", MarkerArray, queue_size=10)
        self.goals_pub = rospy.Publisher(
            "/goals_models", MarkerArray, queue_size=10)

        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.get_map)
        self.tf_listener = tf.TransformListener()
        self.robot_pose = Odometry()

    # Initialise our graph.
    def init_graph(self):

        # For every cell in the array, add it as a node in our graph.
        for i in range(0, len(self.cells_array)):
            self.graph.add_node(
                i, self.cells_array[i][0], self.cells_array[i][1])

        # Go through all identified cells.
        for i in range(0, len(self.cells_array)):
            # And for every cell, go through its neighbours.
            for j in range(0, len(self.neighbours[i])):
                # Obtain the euclidian distance between those two cells.
                dist_x = abs(
                    self.cells_array[i][0] - self.cells_array[list(self.neighbours[i])[j]][0])
                dist_y = abs(
                    self.cells_array[i][1] - self.cells_array[list(self.neighbours[i])[j]][1])
                s_x = self.cells_array[i][0]
                s_y = self.cells_array[i][1]
                t_x = self.cells_array[list(self.neighbours[i])[j]][0]
                t_y = self.cells_array[list(self.neighbours[i])[j]][1]

                # And append a new edge with those values.
                self.graph.add_edge(
                    i, s_x, s_y, list(self.neighbours[i])[j], t_x, t_y, math.sqrt(dist_x**2 + dist_y**2))

    # Find shortest path using A*
    def find_shortest_path(self, s, t):

        # Get our start and end node from our graph structure.
        start = self.graph.get_node(s)
        finish = self.graph.get_node(t)

        # Initialize all of our variables.
        closedset = []
        came_from = {}
        g_score = {}
        g_score[start] = 0
        f_score = {}
        f_score[start] = g_score[start] + heuristic_estimate(start, finish)
        openset = [[f_score[start], start]]
        # Use heap queue for maximalised efficiency.
        heapq.heapify(openset)

        # While there are some edges left in the openset.
        while openset:

            # Pop the edge from the heapq that is closest to our current edge.
            current = heapq.heappop(openset)[1]

            # If it is the finish, terminate the loop.
            if current == finish:
                break

            # Add considered node to the closed set.
            closedset.append(current)

            # And consider all its neighbours that are in openset.
            for neighbour in current.get_neighbours():

                if neighbour in closedset:
                    continue

                # Obtain possible score.
                tentative_g_score = g_score[current] + \
                    current.get_weight(neighbour)

                # If the neighbour doesn't have any value attached or possible score is better than current score.
                if neighbour not in openset or tentative_g_score < g_score[neighbour]:

                    # Update its score.
                    # So we can track the final path at the end.
                    came_from[neighbour] = current
                    g_score[neighbour] = tentative_g_score
                    f_score[neighbour] = g_score[neighbour] + \
                        heuristic_estimate(neighbour, finish)

                    # And append to the openset if it's not there.
                    if neighbour not in list(chain.from_iterable(openset)):
                        heapq.heappush(
                            openset, [f_score[neighbour], neighbour])

                    # Otherwise find out the entry and just update it.
                    else:
                        index = [i for i, x in enumerate(
                            openset) if x[1] == neighbour][0]
                        openset[index][0] = f_score[neighbour]

        # Now after finding the shortest path, go through the final path.
        current = finish
        total_path = [finish]
        distance = 0

        # While we haven't reached the start
        while current != start:
            # Go from current -> to start
            try:
                current = came_from[current]
                total_path.append(current)
            except KeyError:
                self.distance = 999999999999
                self.path = []
                return

        # For the purpose of TSP solution, get the total length of the path
        for i in range(0, len(total_path) - 2):
            try:
                distance += total_path[i].get_weight(total_path[i + 1])
            except KeyError:
                distance += total_path[i + 1].get_weight(total_path[i])

        # Remove start and end cell from the path and update member variables.
        total_path = total_path[1:-1]
        self.distance += distance
        self.path = total_path

    # Get map data from the map server.
    def get_map(self, data):
        self.mapgrid = data
        self.start_x = data.info.origin.position.x
        self.start_y = data.info.origin.position.y
        self.resolution = data.info.resolution

    # Obtain al possible permutations of our path to get shortestt one overall.
    def get_path_permutation(self, start_x, start_y):
        goal_list = []
        goal_list_permutations = []
        distances = []

        # Get goals from params.
        for i in range(5):
            goal_list.append(rospy.get_param("/goal" + str(i)))

        ite = 0

        # And for every permutation of our goal list.
        for x in list(permutations(goal_list)):
            goal_list_permutations.append(list(x))
            goal_list_permutations[ite].insert(0, [start_x, start_y])
            ite += 1

        # Obtain shortest path between those points using our A* method.
        for x in goal_list_permutations:
            self.graph = Graph()
            self.init_graph()
            for y in range(len(x) - 1):
                self.draw_path(x[y][0], x[y][1], x[y + 1][0], x[y + 1][1], y)
            distances.append(self.distance)
            self.distance = 0

        # Get the shortest path overall from all of our permutations.
        self.best_path = goal_list_permutations[distances.index(
            min(distances))]

    # Based on X Y coordinate find out which node (cell) is the closest)
    def find_closest_cell(self, x, y):
        min_dist = 9999
        cell_index = -1
        ite = 0

        # Go through all possible cells.
        for i in self.cells_array:
            dist = math.sqrt(i[0]**2 + i[1]**2)
            if((dist < min_dist) and
               (x in range(i[0] - i[2] / 2, i[0] + i[2] / 2) and
                    (y in range(i[1] - i[3] / 2, i[1] + i[3] / 2)))):
                cell_index = ite
                min_dist = dist
            ite += 1

        # And return the index of the requested cell.
        return cell_index

    # Draw path between X Y and place it on rviz
    def draw_path(self, sx, sy, fx, fy, id_num):

        cell_y = 0

        # Convert from the frame based variables to grid based coordinattes
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

        # And then get the shortest path between those to points.
        self.find_shortest_path(self.find_closest_cell(
            sy_coord, sx_coord), self.find_closest_cell(fy_coord, fx_coord))

        # And create a path in rviz. between those two points.
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

    # Simply grab all goals from the param server and place them on the map.
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

    # Send our best path to our driving node.
    def publish_best_path(self):
        pub1 = rospy.Publisher(
            "/best_path_x", Float32MultiArray, queue_size=10, latch=True)
        pub2 = rospy.Publisher(
            "/best_path_y", Float32MultiArray, queue_size=10, latch=True)

        message_x = Float32MultiArray()
        message_y = Float32MultiArray()

        # Skip theta, since it's not needed.
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

        # Wait till we obtain our permutations.
        got_permutations = False
        while not got_permutations:
            try:
                robot.get_path_permutation(-4.8, -3.6)
                got_permutations = True
            except:
                pass

        while not rospy.is_shutdown():
            # Keep drawing the goals.
            robot.draw_goals(-4.8, -3.6)
            robot.publish_best_path()

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
