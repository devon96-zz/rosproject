#!/usr/bin/env python2
import rospy
import tf
import numpy as np
import math
import pickle
import heapq
from itertools import chain
from geometry_msgs.msg import Point, Twist
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan


class Localisation():
    def __init__(self):
        rospy.init_node('localization', anonymous=True)
        laser_sub = rospy.Subscriber(
            '/noisy_base_scan', LaserScan, self.get_scan)
        self.laser_readings = LaserScan()

    def get_scan(self, scan):
        self.laser_readings = scan


if __name__ == '__main__':
    try:
        localisation = Localisation()
        rate = rospy.Rate(1)

        s = np.random.uniform(0, 10, 10)
        while not rospy.is_shutdown():
            print s
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
