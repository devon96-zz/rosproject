#!/usr/bin/env python2
import rospy
import rospkg
import tf
import numpy as np
import math
import pickle
import heapq
from itertools import chain, permutations
from geometry_msgs.msg import Point, Twist, Quaternion, PointStamped, QuaternionStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid, Odometry
from multiprocessing import Process
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, Header
from time import sleep
import os
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class RvizMarkers():
    def __init__(self):
        rospy.init_node('rviz_node', anonymous=True)

        pose_sub = rospy.Subscriber(
            "/base_pose_ground_truth", Odometry, self.draw_self)
        self.rviz_pub = rospy.Publisher(
            "/robot_pose", MarkerArray, queue_size=10)
        self.odom_pub = rospy.Publisher(
            "/odom_pose", Odometry, queue_size=10)
        self.listener = tf.TransformListener()

    def draw_self(self, odom):

        ma = MarkerArray()

        mr = Marker()
        mr.header.frame_id = "/map"
        mr.ns = "real_pose"
        mr.id = 1
        mr.type = mr.CUBE
        mr.action = mr.ADD
        mr.pose.position.x = odom.pose.pose.position.x - 0.05
        mr.pose.position.y = odom.pose.pose.position.y
        mr.pose.position.z = 0.05
        mr.pose.orientation = odom.pose.pose.orientation
        mr.scale.x = 0.1
        mr.scale.y = 0.1
        mr.scale.z = 0.1
        mr.color.r = 0
        mr.color.g = 0
        mr.color.b = 1
        mr.color.a = 1.0
        ma.markers.append(mr)

        try:
            tf = self.listener.lookupTransform(
                '/map', '/odom', rospy.Time(0))

            mr = Marker()
            mr.header.frame_id = "/map"
            mr.ns = "odom_pose"
            mr.id = 1
            mr.type = mr.CUBE
            mr.action = mr.ADD
            mr.pose.position.x = tf[0][0] - 0.05
            mr.pose.position.y = tf[0][1]
            mr.pose.position.z = 0.05
            mr.pose.orientation.x = tf[1][0]
            mr.pose.orientation.y = tf[1][1]
            mr.pose.orientation.z = tf[1][2]
            mr.pose.orientation.w = tf[1][3]
            mr.scale.x = 0.1
            mr.scale.y = 0.1
            mr.scale.z = 0.1
            mr.color.r = 1
            mr.color.g = 0
            mr.color.b = 0
            mr.color.a = 1.0
            ma.markers.append(mr)

            odom = Odometry()
            odom.pose.pose.position.x = tf[0][0]
            odom.pose.pose.position.y = tf[0][1]
            odom.pose.pose.orientation.x = tf[1][0]
            odom.pose.pose.orientation.y = tf[1][1]
            odom.pose.pose.orientation.z = tf[1][2]
            odom.pose.pose.orientation.w = tf[1][3]
            self.odom_pub.publish(odom)
        except:
            pass

        self.rviz_pub.publish(ma)


if __name__ == '__main__':
    try:

        rviz = RvizMarkers()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
