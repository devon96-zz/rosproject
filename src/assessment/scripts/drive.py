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
            "/best_path_y", Float32MultiArray, self.get_y_points)

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
                if min(self.scan.ranges[10:18]) < 0.15:
                    tw.linear.x = 0
                    self.vel_pub.publish(tw)
                    x = self.scan.ranges.index(min(self.scan.ranges[10:18]))
                    self.turn(self.scan.angle_min + x *
                              self.scan.angle_increment + math.pi / 2, False)
            except ValueError:
                pass

            tw.linear.x = 0.3
            self.vel_pub.publish(tw)

        tw.linear.x = 0
        self.vel_pub.publish(tw)

    def turn_direction(self, x, y):

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
        print x1, x2
        if isclose(x1 + dist, x2) or isclose(x1 + dist - 360, x2):
            direction = True

        return math.radians(dist), direction

    def drive_to_goal(self, sx, sy, fx, fy):
        distance = math.sqrt((sx - fx)**2 + (sy - fy)**2)
        angle = math.atan2(fy - sy, fx - sx)

        angle, direction = self.turn_direction(angle, self.theta)
        print math.degrees(angle), direction

        self.theta = math.atan2(fy - sy, fx - sx)
        self.turn(abs(angle), direction)

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
        # self.x_points = [-4.883999824523926, -4.127999782562256, -3.75600004196167, -2.628000020980835, -2.628000020980835, -2.3519999980926514, -2.3519999980926514, -2.3519999980926514, -2.3519999980926514, -2.3519999980926514, -2.0, -2.0, -1.8839999437332153, -1.1279999017715454, -0.38399994373321533, -0.19199994206428528, -0.19199994206428528, -0.19199994206428528, -0.19199994206428528, -0.19199994206428528, 0.3720000684261322, 0.8280000686645508, 0.924000084400177, 1.2120000123977661, 1.4040000438690186, 2.5, 2.5, 4.5, 4.5, 5.24399995803833, 4.404000282287598, 4.404000282287598, 4.404000282287598, 4.404000282287598, 4.211999893188477, 4.019999980926514, 3.9240000247955322, 3.9240000247955322, 3.9240000247955322, 3.828000068664551, 3.371999979019165, 3.828000068664551, 3.828000068664551, 3.828000068664551, 3.828000068664551,
        #                  4.019999980926514, 4.079999923706055, 4.380000114440918, 4.559999942779541, 4.740000247955322, 4.956000328063965, 5.052000045776367, 5.052000045776367, 5.150000095367432, 5.150000095367432, 5.052000045776367, 5.052000045776367, 4.956000328063965, 4.559999942779541, 4.380000114440918, 4.079999923706055, 4.019999980926514, 3.828000068664551, 3.6480000019073486, 3.4560000896453857, 3.180000066757202, 2.9040000438690186, 2.9040000438690186, 2.9040000438690186, 2.9040000438690186, 2.9040000438690186, 2.9040000438690186, 2.9040000438690186, 2.808000087738037, 2.4240000247955322, 2.052000045776367, 1.6800000667572021, 1.4040000438690186, 0.7440000772476196, -0.7559999823570251, -1.5959999561309814, -1.5959999561309814, -1.7879999876022339, -1.691999912261963, -2.0759999752044678, -2.447999954223633, -2.5439999103546143, -2.549999952316284]

        # self.y_points = [-3.299999952316284, -3.299999952316284, -4.200000286102295, -3.9000000953674316, -3.299999952316284, -2.9160001277923584, -2.7720000743865967, -2.616000175476074, -2.4720001220703125, -2.315999984741211, -2.0, -2.0, -1.5, -1.5, -1.5, -1.944000005722046, -2.24399995803833, -2.5440001487731934, -2.8440001010894775, -3.1440000534057617, -3.299999952316284, -3.5160000324249268, -3.74399995803833, -3.815999984741211, -3.815999984741211, -4.150000095367432, -4.150000095367432, -3.5, -3.5, -1.8000000715255737, -1.4160000085830688, -1.2720000743865967, -1.1160000562667847, -0.9720000624656677, -0.9720000624656677, -0.9720000624656677, -0.7440000772476196, -0.4440000355243683, -0.14400003850460052, 0.08399996161460876, 0.2999999523162842, 0.5279999375343323, 0.6839999556541443, 0.8279999494552612,
        #                  0.9839999675750732, 0.9839999675750732, 1.0080000162124634, 1.0439999103546143, 1.0439999103546143, 1.0439999103546143, 1.128000020980835, 1.3559999465942383, 1.656000018119812, 2.0, 2.0, 1.656000018119812, 1.3559999465942383, 1.128000020980835, 1.0439999103546143, 1.0439999103546143, 1.0080000162124634, 0.9839999675750732, 0.9839999675750732, 0.9839999675750732, 0.9839999675750732, 1.055999994277954, 1.128000020980835, 1.2839999198913574, 1.4279999732971191, 1.5839999914169312, 1.7279999256134033, 1.8839999437332153, 2.0279998779296875, 2.25600004196167, 2.25600004196167, 2.25600004196167, 2.25600004196167, 2.328000068664551, 3.0, 3.0, 3.0839998722076416, 3.2279999256134033, 3.2279999256134033, 3.4560000896453857, 3.4560000896453857, 3.4560000896453857, 3.2279999256134033, 3.130000114440918]

        x = -4.8
        y = -3.6

        for i in range(0, len(self.x_points) - 1):
            self.drive_to_goal(x, y, self.x_points[i], self.y_points[i])
            x = self.x_points[i]
            y = self.y_points[i]


if __name__ == '__main__':
    try:
        drive = Drive()
        # drive.drive_to_all_goals()
        drive.wander()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
