#!/usr/bin/env python2

"""Module responsible for driving to all goals and controlling robots motion"""
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
    """Main class containing all the functions"""

    def __init__(self):
        rospy.init_node('drive_node', anonymous=True)
        self.vel_pub = rospy.Publisher(
            "/cmd_vel", Twist, queue_size=10)  # To send twists
        # To know how far we travelled
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.get_odom)

        self.x_points_sub = rospy.Subscriber(
            "/best_path_x", Float32MultiArray, self.get_x_points)
        self.y_points_sub = rospy.Subscriber(
            "/best_path_y", Float32MultiArray, self.get_y_points)  # Grab best path from pathplanning node.

        self.pose_sub = rospy.Subscriber(
            "/base_pose_ground_truth", Odometry, self.get_pose)  # To know how far we travelled (fake localisation)

        self.odom = Odometry()
        self.pose = Odometry()
        self.scan = LaserScan()
        self.get_odom(self.odom)
        self.theta = 0

        self.x_points = []
        self.y_points = []

    def get_pose(self, data):
        self.pose = data.pose.pose

    # Two methods to obtain our path from pathplanning node
    def get_x_points(self, data):
        self.x_points = list(data.data)[3:]

    def get_y_points(self, data):
        self.y_points = list(data.data)[3:]

    # Obtain odometry readings and save them in variables.
    def get_odom(self, odom):
        self.odom = odom
        self.quaternion = (
            self.odom.pose.pose.orientation.x,
            self.odom.pose.pose.orientation.y,
            self.odom.pose.pose.orientation.z,
            self.odom.pose.pose.orientation.w
        )
        self.radians = euler_from_quaternion(self.quaternion)[2]

    # Calculate angle difference between two values exprressed in radians.
    def angle_difference(self, t1, t2):
        if abs(t1 - t2) > math.pi / 2:
            return math.pi - max(t1, t2)
        else:
            return abs(t1 - t2)

    # Command robot to turn by given degree
    def turn(self, rad, cw):

        # If odom info isn't ready, wait
        if euler_from_quaternion(self.quaternion)[2] == 0:
            sleep(0.3)

        tw = Twist()

        t0 = rospy.Time.now().to_sec()
        t1 = rospy.Time.now().to_sec()

        # To minise turn time, we need to establish whether we want to turn counter- or clockwise.
        if cw:
            tw.angular.z = -0.20
        else:
            tw.angular.z = 0.20
        self.vel_pub.publish(tw)

        # While our turn distance so far is smaller than the requested distance.
        while (abs(tw.angular.z) * (t1 - t0) / 2) < rad / 2:

            # Keep sending twist and update the time for further inspection.
            t1 = rospy.Time.now().to_sec()
            self.vel_pub.publish(tw)

        # Stop the robot after completing the manevour.
        tw.angular.z = 0
        tw.linear.x = 0
        self.vel_pub.publish(tw)

    # To establish whether we should turn counterclockwise or not.
    def turn_direction(self, x, y):

        # Since I will have to compare floats, I need to define a helper function that will help me with comparison regardless float error.
        def isclose(a, b, rel_tol=1e-09, abs_tol=0.0):
            return abs(a - b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)
        x1 = math.degrees(x)
        x2 = math.degrees(y)
        if x < 0:
            x1 = 360 + x1
        if y < 0:
            x2 = 360 + x2

        dist = (x1 - x2 + 360) % 360
        if (dist > 180):
            dist = 360 - dist
        direction = False

        if isclose(x1 + dist, x2) or isclose(x1 + dist - 360, x2):
            direction = True

        return math.radians(dist), direction

    # Based on start X,Y and finish X,Y drive the robot.
    def drive_to_goal(self, sx, sy, fx, fy):
        # Obtain distance and angle differences.
        distance = math.sqrt((sx - fx)**2 + (sy - fy)**2)
        angle = math.atan2(fy - sy, fx - sx)

        quaternion = (
            self.pose.orientation.x,
            self.pose.orientation.y,
            self.pose.orientation.z,
            self.pose.orientation.w
        )
        self.theta = euler_from_quaternion(quaternion)[2]

        angle, direction = self.turn_direction(angle, self.theta)

        # First turn the robot so he will be facing the goals.
        self.theta = math.atan2(fy - sy, fx - sx)
        self.turn(abs(angle), direction)

        t0 = rospy.Time.now().to_sec()
        t1 = rospy.Time.now().to_sec()
        tw = Twist()
        tw.linear.x = 0.3

        # And then keep moving the robot until he reaches requested distance.
        while (tw.linear.x * (t1 - t0)) < distance:
            self.vel_pub.publish(tw)
            t1 = rospy.Time.now().to_sec()

        # Stop the robot after finishing all the movement.
        tw.linear.x = 0
        self.vel_pub.publish(tw)

    # Name self-explanatory.
    def drive_to_all_goals(self):

        # If we haven't obtained our path, just keep waiting.
        if len(self.x_points) == 0 or len(self.y_points) == 0:
            sleep(1)

        x = rospy.get_param("robot_start")[0]
        y = rospy.get_param("robot_start")[1]

        # For every pair of points, keep driving the robot towards the next pair.
        for i in range(0, len(self.x_points) - 1):
            self.drive_to_goal(x, y, self.x_points[i], self.y_points[i])

            # Update current position.
            x = self.pose.position.x
            y = self.pose.position.y


# Main entry to the program.
if __name__ == '__main__':
    try:
        drive = Drive()
        drive.drive_to_all_goals()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
