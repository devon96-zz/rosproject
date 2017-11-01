#!/usr/bin/env python2
import rospy
import rospkg
import tf
import numpy as np
import math
import pickle
import heapq
from itertools import chain, permutations
from geometry_msgs.msg import Point, Twist, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid, Odometry
from multiprocessing import Process
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from time import sleep
import os
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class Drive():
    def __init__(self):
        rospy.init_node('drive_node', anonymous=True)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.get_odom)
        self.laser_sub = rospy.Subscriber(
            "/noisy_base_scan", LaserScan, self.get_scan)

        self.x_points_sub = rospy.Subscriber(
            "/best_path_x", Float32MultiArray, self.get_x_points)
        self.y_points_sub = rospy.Subscriber(
            "/best_path_x", Float32MultiArray, self.get_y_points)

        self.odom = Odometry()
        self.scan = LaserScan()
        self.get_odom(self.odom)
        self.theta = 0

        self.x_points = []
        self.y_points = []

    def get_x_points(self, data):
        self.x_points = list(data.data)[3:]

    def get_y_points(self, data):
        self.y_points = list(data.data)[3:]

    def get_odom(self, odom):
        self.odom = odom
        self.quaternion = (
            self.odom.pose.pose.orientation.x,
            self.odom.pose.pose.orientation.y,
            self.odom.pose.pose.orientation.z,
            self.odom.pose.pose.orientation.w
        )
        self.radians = euler_from_quaternion(self.quaternion)[2]

    def get_scan(self, scan):
        self.scan = scan

    def angle_difference(self, t1, t2):
        if abs(t1 - t2) > math.pi / 2:
            return math.pi - max(t1, t2)
        else:
            return abs(t1 - t2)

    def turn(self, rad, cw):

        if euler_from_quaternion(self.quaternion)[2] == 0:
            sleep(0.3)

        tw = Twist()

        t0 = rospy.Time.now().to_sec()
        t1 = rospy.Time.now().to_sec()

        if cw:
            tw.angular.z = -0.30
        else:
            tw.angular.z = 0.30
        self.vel_pub.publish(tw)
        while (abs(tw.angular.z) * (t1 - t0) / 2) < rad / 2:

            t1 = rospy.Time.now().to_sec()
            self.vel_pub.publish(tw)

        tw.angular.z = 0
        self.vel_pub.publish(tw)

    def wander(self):

        tw = Twist()
        while True:
            try:
                if min(self.scan.ranges[12:16]) < 0.15:
                    tw.linear.x = 0
                    self.vel_pub.publish(tw)
                    x = self.scan.ranges.index(min(self.scan.ranges[12:16]))
                    self.turn(self.scan.angle_min + x *
                              self.scan.angle_increment + math.pi / 2, False)
            except ValueError:
                pass

            tw.linear.x = 0.3
            self.vel_pub.publish(tw)

        tw.linear.x = 0
        self.vel_pub.publish(tw)

    def drive_to_goal(self, sx, sy, fx, fy):
        distance = math.sqrt((sx - fx)**2 + (sy - fy)**2)
        angle = math.asin(abs(sy - fy) / distance) + self.theta
        if fy > sy:
            self.turn(angle, False)
            self.theta += angle
        else:
            self.turn(angle, True)
            self.theta -= angle

        t0 = rospy.Time.now().to_sec()
        t1 = rospy.Time.now().to_sec()
        tw = Twist()
        tw.linear.x = 0.3
        while (tw.linear.x * (t1 - t0)) < distance:
            self.vel_pub.publish(tw)
            t1 = rospy.Time.now().to_sec()
        tw.linear.x = 0
        self.vel_pub.publish(tw)

    def drive_to_all_goals(self):
        if len(self.x_points) == 0 or len(self.y_points) == 0:
            sleep(1)

        x = -4.8
        y = -3.6

        for i in range(1, len(self.x_points) - 1):
            print self.x_points[i], self.y_points[i]
            self.drive_to_goal(x, y, self.x_points[i], self.y_points[i])
            x = self.x_points[i]
            y = self.y_points[i]


if __name__ == '__main__':
    try:
        drive = Drive()
        drive.drive_to_all_goals()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
