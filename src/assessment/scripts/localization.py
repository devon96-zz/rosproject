#!/usr/bin/env python2

"""Node responsible for localisation"""

import rospy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import math
import pickle
import heapq
from itertools import chain
from geometry_msgs.msg import Point, Twist, Quaternion, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from time import sleep
import scipy.stats
import random
from numpy import cumsum, sort, sum, searchsorted
from numpy.random import rand
from scipy import ndimage
import threading
from collections import Counter


class Localisation():
    def __init__(self):
        rospy.init_node('localization_node', anonymous=True)

        self.laser_readings = LaserScan()

        self.particles = []
        self.probabilities = [0] * 800  # 800 particles

        # Initialize our gaussian curve to calculate probability.
        self.init_particles()

        self.norm = scipy.stats.norm(0, 0.1)
        self.current_x = 0
        self.current_y = 0
        self.current_rad = 0
        self.odom = Odometry()
        self.base_pose = Odometry()
        self.tf = tf.TransformListener()

        # Subscribe to the noisy scanner, map, odometry and base pose
        laser_sub = rospy.Subscriber(
            '/noisy_base_scan', LaserScan, self.get_scan)
        map_sub = rospy.Subscriber('/map', OccupancyGrid, self.get_map)
        odom_sub = rospy.Subscriber(
            '/odom', Odometry, self.get_odom)
        base_pose_sub = rospy.Subscriber(
            '/base_pose_ground_truth', Odometry, self.get_base_pose)

        # Publish twist and particles positions.
        self.particle_pub = rospy.Publisher(
            '/particles', MarkerArray, queue_size=1000)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)

    # Get odometry readings.
    def get_odom(self, data):
        self.odom = data
        self.quaternion = (
            self.odom.pose.pose.orientation.x,
            self.odom.pose.pose.orientation.y,
            self.odom.pose.pose.orientation.z,
            self.odom.pose.pose.orientation.w
        )

    # Get ground pose.
    def get_base_pose(self, data):
        self.base_pose = data.pose.pose.position

    def init_particles(self):
        # Start by assigning random X, Y and Theta to every particle
        for i in range(800):
            self.particles.append([random.randint(0, 799), random.randint(
                0, 999), random.uniform(-math.pi, math.pi)])

    # Use scipy to calculate PDF of our laser reading.
    def gaussian_p(self, expected, reading):
        return self.norm.pdf(expected - reading)

    #Render partricles on the map#
    def draw_particles(self):
        ma = MarkerArray()
        # For every particle grab its pose and attach marker based on it.
        for i in range(800):
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

        # Publish markers with all particles
        self.particle_pub.publish(ma)

    # Update positions of all particles based on movement.
    def update_particles_positions(self):
        # Obtain pose from previous and current iteration.
        old_x = self.current_x
        old_y = self.current_y
        old_t = self.current_rad
        self.current_x = self.odom.pose.pose.position.x
        self.current_y = self.odom.pose.pose.position.y
        quaternion = (
            self.odom.pose.pose.orientation.x,
            self.odom.pose.pose.orientation.y,
            self.odom.pose.pose.orientation.z,
            self.odom.pose.pose.orientation.w
        )
        self.current_rad = euler_from_quaternion(quaternion)[2]

        delta_t = self.current_rad - old_t

        diff_x = (self.current_x - old_x)
        diff_y = (self.current_y - old_y)

        # r = how far robot has moved forward
        r = math.cos(self.current_rad) * diff_x + \
            math.sin(self.current_rad) * diff_y

        # Update only if the robot has moved (either forward or turned)
        if r != 0 or delta_t != 0:
            # For every particle
            for i in range(800):

                # Convert its X,Y from grid readings to frame reading.
                x, y = self.grid2frame(
                    self.particles[i][0], self.particles[i][1])

                # Update particle's theta postions plus possible error.
                self.particles[i][2] += delta_t + random.uniform(-0.005, 0.005)

                # Update X,Y position plus possible error.
                updated_y = math.sin(
                    self.particles[i][2]) * (r + random.uniform(-0.005, 0.005))
                updated_x = math.cos(
                    self.particles[i][2]) * (r + random.uniform(-0.005, 0.005))

                # Conver the readings back to grid readings
                self.particles[i][0], self.particles[i][1] = self.frame2grid(
                    x + updated_x, y + updated_y)

    # Simply obtain our grid from map_server.
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

    # Obtain our noisy laser readings.
    def get_scan(self, scan):
        self.laser_readings = scan

    # Convert between frame (e.g x=-3.2, y=1.4) to grid (eg. grid[100][200])
    def frame2grid(self, x, y):
        cell_y = 0
        if y < 0:
            cell_y = abs(y) + 4.8
        else:
            cell_y = abs(y - 4.8)
        x_coord = int(round((x + 6.0) / 0.012))
        y_coord = int(round((cell_y) / 0.012))
        return [y_coord, x_coord]

    # Same as above but in reverse.
    def grid2frame(self, x, y):
        y_coord = -self.start_y - (x * self.resolution)
        x_coord = self.start_x + (y * self.resolution)
        return [x_coord, y_coord]

    # Based on beam laser model obtain the closest obstacle that should be encountered based on laser reading.
    def get_closest_expected_obstacle(self, x, y, yaww):
        updated_x = 0
        updated_y = 0
        ite = self.resolution

        # Repeat until we hit 3.0 (scanners maximum range)
        while ite <= 3.0:
            # Update X,Y based on possible obstacle location
            updated_y = math.sin(yaww) * ite
            updated_x = math.cos(yaww) * ite

            # Convert between frame and grid so we can find out what's in there in the Grid.
            map_x, map_y = self.frame2grid(x + updated_x, y + updated_y)

            # If the result is out of map boundaries, we hit an edge and return current reading.
            if (not (0 < map_x < 800)) or (not (0 < map_y < 1000)):
                break
            # If laser reading points to a grid field with value 100, it means we've hit an obstacle.
            try:
                if(self.grid_array[map_x][map_y] == 100):
                    return ite
            except Exception as e:
                return ite

            # Increase iterator based on the resolution.
            ite += self.resolution * 3
        return ite

    # Update particles probaility based on the laser reading
    def get_position_probability(self, index):
        total_prob = 0

        # Obtain particles x,y and theta.
        robot_x, robot_y = self.grid2frame(
            self.particles[index][0], self.particles[index][1])

        radians = self.particles[index][2]

        # Get our noisy scanner data.
        laser_angle = self.laser_readings.angle_min
        angle_inc = self.laser_readings.angle_increment

        # For every 6th laser scanner.
        for reading in self.laser_readings.ranges[::6]:
            # Find out the closest obstacle.
            obst_dist = self.get_closest_expected_obstacle(
                robot_x, robot_y, radians + laser_angle)

            # And based on the gaussian probability density function get the probability.
            total_prob += self.gaussian_p(obst_dist, reading)

            # Increament the angle of the reading.
            laser_angle += angle_inc * 6

        # Update the probability of given particle.
        self.probabilities[index] = total_prob

    # Sample articles for the next iteration.
    def update_probabilities(self):

        # Helper method to select (with replacement) from set based on the weight)
        def weighted_pick(weights, n_picks):

            t = cumsum(weights)
            s = sum(weights)
            return searchsorted(t, rand(n_picks) * s)

        # Run contiunously since it's going to be threaded.
        while True:

            # For every particle.
            for i in range(800):
                # Update its probability
                self.get_position_probability(i)

            # Sample 500 particles based on the probability.
            indices = weighted_pick(self.probabilities, 500)
            updated_prob = []

            total_x = 0
            total_y = 0

            # Find out the most probable entry
            for i in indices:
                updated_prob.append(self.particles[i][:])
                total_x += self.particles[i][0]
                total_y += self.particles[i][1]
            common_indx = max(set(indices), key=list(indices).count)

            # Output where we think we are based on the MCL and where we actually are.
            print "I think I am at:", self.grid2frame(self.particles[common_indx][0], self.particles[common_indx][1])
            print "Where I actuall am:", self.base_pose.x, self.base_pose.y

            # Append 300 random particles to allow recovery from getting lost.
            for i in range(300):
                updated_prob.append([random.randint(0, 799), random.randint(
                    0, 999), random.uniform(-math.pi, math.pi)])

            # Update our particles.
            self.particles = list(updated_prob)

    # Turn our robot based on radian distance (function borrowed from driving module).
    def turn(self, rad, cw):

        tw = Twist()

        t0 = rospy.Time.now().to_sec()
        t1 = rospy.Time.now().to_sec()

        if cw:
            tw.angular.z = -0.20
        else:
            tw.angular.z = 0.20
        self.vel_pub.publish(tw)

        rate = rospy.Rate(10)
        # While our turn distance so far is smaller than the requested distance.
        while (abs(tw.angular.z) * (t1 - t0) / 2) < rad / 2:

            # Keep sending twist and update our time.
            t1 = rospy.Time.now().to_sec()
            self.vel_pub.publish(tw)
            rate.sleep()

        tw.angular.z = 0
        tw.linear.x = 0
        self.vel_pub.publish(tw)

    # Simple wander method making the robot drive randomly around the map.
    def wander(self):

        tw = Twist()
        rate = rospy.Rate(5)
        while True:

            # If we are close to the obstacle, turn the robot.
            try:
                if min(self.laser_readings.ranges[8:20]) < 0.2:
                    tw.linear.x = 0
                    self.vel_pub.publish(tw)
                    self.turn(math.pi / 2, False)
            except Exception as e:
                pass

            # If not, keep moving forward.
            tw.linear.x = 0.2
            self.vel_pub.publish(tw)
            rate.sleep()

        tw.linear.x = 0
        self.vel_pub.publish(tw)


if __name__ == '__main__':
    try:
        localisation = Localisation()
        rate = rospy.Rate(4)
        # Wait till we obtain info from map_server.
        while not hasattr(localisation, "mapgrid"):
            rate.sleep()

        # Start wandering in a new thread.
        wander_thread = threading.Thread(target=localisation.wander)

        # Start updating our probabilities in a new thread.
        particles_thread = threading.Thread(
            target=localisation.update_probabilities)
        wander_thread.daemon = True
        particles_thread.daemon = True

        # Start both threads.
        wander_thread.start()
        particles_thread.start()

        while not rospy.is_shutdown():

            try:
                # Keep updating positions of particles and drawing them on a map.
                localisation.draw_particles()
                localisation.update_particles_positions()

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

            rate.sleep()

    except rospy.ROSInterruptException:
        wander_thread.exit()
