#!/usr/bin/env python2
import rospy
import tf
from tf.transformations import euler_from_quaternion
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


class Localisation():
    def __init__(self):
        rospy.init_node('localization', anonymous=True)
        laser_sub = rospy.Subscriber(
            '/noisy_base_scan', LaserScan, self.get_scan)
        map_sub = rospy.Subscriber('/map', OccupancyGrid, self.get_map)
        rospy.wait_for_message('/map', OccupancyGrid)
        pose_sub = rospy.Subscriber(
            '/base_pose_ground_truth', Odometry, self.get_pose)
        self.rviz_pub = rospy.Publisher(
            '/laserscanner', MarkerArray, queue_size=10)
        self.tf = tf.TransformListener()
        self.laser_readings = LaserScan()

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
            if(self.grid_array[map_x][map_y] == 100):
                print "OBSTACLE FOUND at", self.grid2frame(map_x, map_y)
                break
            # print map_x, map_y
            ite += self.resolution

        ma = MarkerArray()
        mr = Marker()
        mr.header.frame_id = "/map"
        mr.ns = "laser"
        mr.id = 1
        mr.type = mr.CUBE
        mr.action = mr.ADD
        mr.pose.position.x = x + updated_x - 0.05
        mr.pose.position.y = y + updated_y
        mr.pose.position.z = 0.05
        mr.scale.x = 0.1
        mr.scale.y = 0.1
        mr.scale.z = 0.1
        mr.color.r = 0
        mr.color.g = 1
        mr.color.b = 0
        mr.color.a = 1.0
        ma.markers.append(mr)
        self.rviz_pub.publish(ma)


if __name__ == '__main__':
    try:
        localisation = Localisation()
        rate = rospy.Rate(5)
        twist_sub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)

        tw = Twist()

        while not rospy.is_shutdown():
            tw.angular.z = 0.2
            tw.linear.x = 0.05
            twist_sub.publish(tw)
            try:
                yaw = euler_from_quaternion(localisation.tf.lookupTransform(
                    '/map', '/base_footprint', rospy.Time(0))[1])[2]
                pose = localisation.tf.lookupTransform(
                    '/map', '/base_footprint', rospy.Time(0))[0]
                localisation.get_closest_expected_obstacle(
                    pose[0], pose[1], yaw)

                print pose[0], pose[1], math.degrees(yaw)

                # print math.degrees(yaw)

                # print math.sin(yaw) * 2
                # print math.cos(yaw) * 2

                # print euler_from_quaternion(localisation.pose.pose.orientation)
                # print localisation.pose.covariance
                # print localisation.laser_readings.ranges[14]
                # print scipy.stats.norm(0, 1).cdf(0.5)
            except Exception as e:
                print e

            # print s
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
