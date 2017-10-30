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
from multiprocessing import Process
from time import sleep
import os


class Test():
    def __init__(self):
        self.x = range(1000)
        self.array = [0] * 10000

    def print_x(self):
        for i in self.x:
            # # print os.getpid(), i
            # self.array[i] = i
            pass

    def procs(self):
        procs = []

        for i in range(1000):
            proc = Process(target=self.print_x)
            procs.append(proc)
            proc.start()
        for i in procs:
            i.join()
        print self.array


if __name__ == '__main__':
    try:
        rospy.init_node('drive_node', anonymous=True)
        vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rate = rospy.Rate(5)

        while not rospy.is_shutdown():

            tw = Twist()
            tw.linear.x = 0.05
            #tw.angular.z = 0.005
            vel_pub.publish(tw)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
