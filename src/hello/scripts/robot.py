#!/usr/bin/env python
# license removed for brevity
import rospy
import tf
import roslib
import math
from std_msgs.msg import String, Header
from geometry_msgs.msg import PointStamped, Point, Twist, Pose
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid, Odometry


def add(x, y, r, g, b, frame):
    mr = Marker()
    mr.header.frame_id = frame
    mr.ns = "basic"
    mr.id = 10
    mr.type = mr.ARROW
    mr.action = mr.ADD

    mr.points.append(Point(x=x, y=y, z=0.05))
    mr.points.append(Point(x=x + 0.5, y=y, z=0.05))

    mr.scale.x = 0.05
    mr.scale.y = 0.1
    mr.scale.z = 0.05
    mr.color.r = r
    mr.color.g = g
    mr.color.b = b
    mr.color.a = 1.0

    ma = MarkerArray()
    ma.markers.append(mr)
    map_pub.publish(ma)


def add_robot_model():
    mr = Marker()
    mr.header.frame_id = "/base_link"
    mr.ns = "basic"
    mr.id = 2
    mr.type = mr.CUBE
    mr.action = mr.ADD

    mr.pose.position.x = 0
    mr.pose.position.y = 0
    mr.pose.position.z = 0

    mr.scale.x = 0.1
    mr.scale.y = 0.1
    mr.scale.z = 0.1
    mr.color.r = 0
    mr.color.g = 0
    mr.color.b = 1
    mr.color.a = 1.0

    ma = MarkerArray()
    ma.markers.append(mr)
    map_pub.publish(ma)


class MapTransform():
    def __init__(self):
        self.pub = rospy.Publisher(
            "/real_robot_pos", OccupancyGrid, queue_size=10)
        map_sub = rospy.Subscriber("/map", OccupancyGrid, self.get_old_map)
        get_odom_sub = rospy.Subscriber(
            "/base_pose_ground_truth", Odometry, self.get_odom)

        self.listener = tf.TransformListener()

    def get_old_map(self, data):
        self.old_map = data
        self.start_x = data.info.origin.position.x
        self.start_y = data.info.origin.position.y
        self.start_z = data.info.origin.position.z

    def get_odom(self, data):
        self.robot_pose = data.pose

    def transformmap(self):
        new_map = self.old_map

        try:

            transformed_pose = self.listener.lookupTransform(
                "/base_link", "/odom", rospy.Time(0))

            print
            print self.robot_pose.pose.position.x, self.start_x
            print self.robot_pose.pose.position.y, self.start_y
            print

            #new_map.info.origin.position.x = transformed_pose[0][0] + self.start_x
            #new_map.info.origin.position.y = transformed_pose[0][1] + self.start_y
            new_map.info.origin.position.x = self.robot_pose.pose.position.x
            new_map.info.origin.position.y = self.robot_pose.pose.position.y
            new_map.info.origin.position.z = transformed_pose[0][2]

            new_map.info.origin.orientation.x = transformed_pose[1][0]
            new_map.info.origin.orientation.y = transformed_pose[1][1]
            new_map.info.origin.orientation.z = transformed_pose[1][2]

            self.pub.publish(new_map)
        except Exception, e:
            print str(e)


if __name__ == '__main__':
    try:
        publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        map_pub = rospy.Publisher("/map_rviz", MarkerArray, queue_size=10)
        mapTransform = MapTransform()

        rospy.init_node('my_robot_node', anonymous=True)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            add_robot_model()
            mapTransform.transformmap()
            tw = Twist()
            tw.linear.x = 0.01
            publisher.publish(tw)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
