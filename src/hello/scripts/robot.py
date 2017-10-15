#!/usr/bin/env python2
import rospy
import tf
import roslib
import math
import numpy as np
from std_msgs.msg import String, Header
from geometry_msgs.msg import PointStamped, Point, Twist, Pose
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid, Odometry


class myRobot():
    def __init__(self):

        self.cells_array = []

        self.rviz_pub = rospy.Publisher(
            "/robot_model", MarkerArray, queue_size=10)
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.getMap)

        self.rviz_map_pub = rospy.Publisher(
            "/real_robot_pose", OccupancyGrid, queue_size=10)

        self.cells_pub = rospy.Publisher(
            "/cells_boxes", MarkerArray, queue_size=10)

        self.ground_pose_sub = rospy.Subscriber(
            "/base_pose_ground_truth", Odometry, self.getPose)

        self.tf_listener = tf.TransformListener()

    def getMap(self, data):
        self.mapgrid = data
        self.start_x = data.info.origin.position.x
        self.start_y = data.info.origin.position.y
        self.resolution = data.info.resolution

        occupancy = np.array(data.data, dtype=np.int16)
        occupancy = np.where(occupancy == 100, 1, occupancy)

        occupancy = np.reshape(occupancy, (-1, data.info.width))
        occupancy = np.flipud(occupancy)

        occupancy[:, 0] = 0
        occupancy[0, :] = 0

        def draw_lines(arr, depth):
            if 1 not in arr:
                arr[arr.shape[0] / 2, arr.shape[1] / 2] = arr.shape[0]
            elif depth == 0:
                return
            else:
                draw_lines(arr[:arr.shape[0] / 2, :arr.shape[1] / 2],
                           depth - 1)  # Top left

                draw_lines(arr[:arr.shape[0] / 2, arr.shape[1] / 2:],
                           depth - 1)  # Top right

                draw_lines(arr[arr.shape[0] / 2:, :arr.shape[1] / 2],
                           depth - 1)  # Bottom left

                draw_lines(arr[arr.shape[0] / 2:, arr.shape[1] / 2:],
                           depth - 1)  # Bottom right

        draw_lines(occupancy, 7)

        for i in range(0, occupancy.shape[0]):
            for j in range(0, occupancy.shape[1]):
                if occupancy[i][j] > 1:
                    self.cells_array.append((i, j, occupancy[i][j], int(
                        round((occupancy[i][j] * occupancy.shape[1]) / occupancy.shape[0]))))

    def draw_boxes(self):
        ma = MarkerArray()

        ite = 2

        for mytuple in self.cells_array:
            mr = Marker()
            mr.header.frame_id = "/map"
            mr.ns = "basic"
            mr.id = ite
            mr.type = mr.CUBE
            mr.action = mr.ADD
            mr.pose.position.y = -self.start_y - (mytuple[0] * self.resolution)
            mr.pose.position.x = self.start_x + (mytuple[1] * self.resolution)
            mr.pose.position.z = 0.05
            mr.scale.x = 0.05
            mr.scale.y = 0.05
            mr.scale.z = 0.05
            mr.color.r = 0
            mr.color.g = 0
            mr.color.b = 1
            mr.color.a = 1.0
            ma.markers.append(mr)
            ite += 1

        self.cells_pub.publish(ma)

    def getPose(self, robot_pose):
        newgrid = self.mapgrid
        newgrid.info.origin.position.x = self.start_x
        newgrid.info.origin.position.y = self.start_y

        mr = Marker()
        mr.header.frame_id = "/map"
        mr.ns = "basic"
        mr.id = 2
        mr.type = mr.CUBE
        mr.action = mr.ADD
        mr.pose.position.x = robot_pose.pose.pose.position.x - 0.05
        mr.pose.position.y = robot_pose.pose.pose.position.y
        mr.pose.position.z = 0.05
        mr.pose.orientation = robot_pose.pose.pose.orientation
        mr.scale.x = 0.1
        mr.scale.y = 0.1
        mr.scale.z = 0.1
        mr.color.r = 0
        mr.color.g = 0
        mr.color.b = 1
        mr.color.a = 1.0
        ma = MarkerArray()
        ma.markers.append(mr)
        self.rviz_pub.publish(ma)

        self.rviz_map_pub.publish(newgrid)

    def drawRobot(self):
        pass


if __name__ == '__main__':
    try:
        rospy.init_node('my_robot_node', anonymous=True)
        vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        robot = myRobot()

        rate = rospy.Rate(5)

        while not rospy.is_shutdown():

            tw = Twist()
            # tw.linear.x = 0.1
            vel_pub.publish(tw)

            robot.drawRobot()
            try:
                robot.draw_boxes()
            except:
                pass
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
