#!/usr/bin/env python2
# license removed for brevity
import rospy
import tf
import roslib
import math
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, Point, Twist
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid, Odometry


class myRobot():
    def __init__(self):
        self.rviz_pub = rospy.Publisher(
            "/robot_model", MarkerArray, queue_size=10)
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.getMap)

        self.rviz_map_pub = rospy.Publisher(
            "/real_robot_pose", OccupancyGrid, queue_size=10)

        self.ground_pose_sub = rospy.Subscriber(
            "/base_pose_ground_truth", Odometry, self.getPose)

        self.tf_listener = tf.TransformListener()

    def getMap(self, data):
        self.mapgrid = data
        self.start_x = data.info.origin.position.x
        self.start_y = data.info.origin.position.y

    def getPose(self, robot_pose):
        newgrid = self.mapgrid
        newgrid.info.origin.position.x = self.start_x
        newgrid.info.origin.position.y = self.start_y
        #newgrid.info.origin.position.x = self.start_x
        #newgrid.info.origin.position.y = self.start_y

        ps = PointStamped(header=Header(stamp=rospy.Time.now(), frame_id="/base_link"),
                          point=Point(robot_pose.pose.pose.position.x, robot_pose.pose.pose.position.y, 0))

        # print robot_pose.pose.pose.position.x, robot_pose.pose.pose.position.y

        # try:
        #     transform = self.tf_listener.transformPoint("/map", ps)
        #     print transform
        # except Exception, e:
        #     print e
        #     pass

        # print robot_pose.pose.pose.position.x, robot_pose.pose.pose.position.y
        # print robot_pose.header.frame_id

        mr = Marker()
        mr.header.frame_id = "/map"
        mr.ns = "basic"
        mr.id = 2
        mr.type = mr.CUBE
        mr.action = mr.ADD
        mr.pose.position.x = robot_pose.pose.pose.position.x - 0.05
        mr.pose.position.y = robot_pose.pose.pose.position.y
        mr.pose.orientation.w = 1
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
            tw.linear.x = 0.1
            vel_pub.publish(tw)

            robot.drawRobot()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
