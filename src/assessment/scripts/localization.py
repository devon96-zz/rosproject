#!/usr/bin/env python2
import rospy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import math
import pickle
import heapq
from itertools import chain
from geometry_msgs.msg import Point, Twist
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from time import sleep
import scipy.stats
import random
from multiprocessing.dummy import Pool as ThreadPool


class Localisation():
    def __init__(self):
        rospy.init_node('localization_node', anonymous=True)
        laser_sub = rospy.Subscriber(
            '/noisy_base_scan', LaserScan, self.get_scan)
        map_sub = rospy.Subscriber('/map', OccupancyGrid, self.get_map)
        rospy.wait_for_message('/map', OccupancyGrid)
        pose_sub = rospy.Subscriber(
            '/base_pose_ground_truth', Odometry, self.get_pose)
        # odom_sub = rospy.Subscriber(
        #     '/odom', Odometry, self.update_particles_positions)
        self.rviz_pub = rospy.Publisher(
            '/laserscanner', MarkerArray, queue_size=10)
        self.particle_pub = rospy.Publisher(
            '/particles', MarkerArray, queue_size=1000)
        self.tf = tf.TransformListener()
        self.laser_readings = LaserScan()

        self.particles = []
        self.probabilities = [0] * 1000
        self.init_particles()

    def init_particles(self):
        for i in range(1000):
            self.particles.append([random.randint(0, 800), random.randint(
                0, 1000), random.uniform(-math.pi, math.pi)])

    def gaussian_p(self, expected, reading):
        upper_bound = reading + 0.02
        lower_bound = reading - 0.02
        norm = scipy.stats.norm(expected, 0.1)

        return abs(norm.cdf(upper_bound) - norm.cdf(lower_bound))

    def draw_particles(self):
        ma = MarkerArray()
        for i in range(1000):
            mr = Marker()
            mr.header.frame_id = "/map"
            mr.ns = "particle"
            mr.id = i
            mr.type = mr.CUBE
            mr.action = mr.ADD
            mr.pose.position.x, mr.pose.position.y = self.grid2frame(
                self.particles[i][0], self.particles[i][1])
            mr.pose.position.z = 0.05
            mr.pose.orientation.x = quaternion_from_euler(
                0, 0, self.particles[i][2])[0]
            mr.pose.orientation.y = quaternion_from_euler(
                0, 0, self.particles[i][2])[1]
            mr.pose.orientation.z = quaternion_from_euler(
                0, 0, self.particles[i][2])[2]
            mr.pose.orientation.w = quaternion_from_euler(
                0, 0, self.particles[i][2])[3]
            mr.scale.x = 0.1
            mr.scale.y = 0.1
            mr.scale.z = 0.1
            mr.color.r = 0
            mr.color.g = 1
            mr.color.b = 0
            mr.color.a = 1.0
            ma.markers.append(mr)
        self.particle_pub.publish(ma)

    def update_particles_positions(self):
        for i in range(1000):
            error = random.choice([-0.002, 0, 0.002])
            x, y = self.grid2frame(self.particles[i][0], self.particles[i][1])

            updated_y = math.sin(self.particles[i][2]) * (0.05 + error)
            updated_x = math.cos(self.particles[i][2]) * (0.05 + error)
            self.particles[i][0], self.particles[i][1] = self.frame2grid(
                x + updated_x, y + updated_y)

    def get_pose(self, pose):
        self.pose = pose.pose

    def get_map(self, grid):
        self.mapgrid = grid
        self.start_x = grid.info.origin.position.x
        self.start_y = grid.info.origin.position.y
        self.resolution = grid.info.resolution

        new_array = np.reshape(self.mapgrid.data, (-1, 1000))
        new_array[:, 0] = 100
        new_array[0, :] = 100
        new_array = np.flipud(new_array)
        self.grid_array = new_array

    def get_scan(self, scan):
        self.laser_readings = scan

    def frame2grid(self, x, y):
        cell_y = 0
        if y < 0:
            cell_y = abs(y) + 4.8
        else:
            cell_y = abs(y - 4.8)
        x_coord = int(round((x + 6.0) / 0.012))
        y_coord = int(round((cell_y) / 0.012))
        return [y_coord, x_coord]

    def grid2frame(self, x, y):
        y_coord = -self.start_y - (x * self.resolution)
        x_coord = self.start_x + (y * self.resolution)
        return [x_coord, y_coord]

    def get_closest_expected_obstacle(self, x, y, yaw):
        updated_x = 0
        updated_y = 0
        ite = self.resolution
        while ite <= 3.0:
            updated_y = math.sin(yaw) * ite
            updated_x = math.cos(yaw) * ite
            # print math.cos(yaw)
            map_x, map_y = self.frame2grid(x + updated_x, y + updated_y)
            try:
                if(self.grid_array[map_x][map_y] == 100):
                    # print "OBSTACLE FOUND at", self.grid2frame(map_x, map_y)
                    return ite
                    break
            except IndexError:
                return ite
                break
            # print map_x, map_y
            ite += self.resolution / 2
        return ite

    def get_position_probability(self, index):
        total_prob = 0

        robot_x, robot_y = self.grid2frame(
            self.particles[index][0], self.particles[index][1])
        # robot_y = self.particles[index][1]
        radians = self.particles[index][2]

        ite = 1
        laser_angle = self.laser_readings.angle_min
        angle_inc = self.laser_readings.angle_increment
        for reading in self.laser_readings.ranges[::3]:
            obst_dist = self.get_closest_expected_obstacle(
                robot_x, robot_y, radians + laser_angle)
            total_prob += self.gaussian_p(obst_dist, reading)

            laser_angle += angle_inc * 3
            ite += 1
        self.probabilities[index] = total_prob

    def update_probabilities(self):
        from multiprocessing import Process

        # processes = [Process(target=self.get_position_probability, args=(i,))
        #              for i in range(1000)]

        # # Start the threads
        # for x in processes:
        #     x.start()

        # # Stop the threads
        # for x in processes:
        #     x.join()
        for i in range(1000):
            self.get_position_probability(i)

        print self.probabilities


if __name__ == '__main__':
    try:
        localisation = Localisation()
        rate = rospy.Rate(3)
        twist_sub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)

        tw = Twist()

        while not rospy.is_shutdown():

            try:
                yaw = euler_from_quaternion(localisation.tf.lookupTransform(
                    '/map', '/base_footprint', rospy.Time(0))[1])[2]
                pose = localisation.tf.lookupTransform(
                    '/map', '/base_footprint', rospy.Time(0))[0]

                # localisation.display_all_lasers(pose[0], pose[1], yaw)

                localisation.draw_particles()
                # localisation.update_particles_positions()
                localisation.update_probabilities()

            except Exception as e:
                print e

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
