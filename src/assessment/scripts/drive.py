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

if __name__ == '__main__':
    try:
        rospy.init_node('drive_node', anonymous=True)
        vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rate = rospy.Rate(5)

        while not rospy.is_shutdown():

            tw = Twist()
            tw.linear.x = 0.01
            vel_pub.publish(tw)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
