#!/usr/bin/env python
# license removed for brevity
import rospy
import tf
import roslib
import math
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, Point, Twist
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan


class MyRobot:
    def __init__(self):
        self.publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def accelerate(self):
        tw = Twist()
        tw.linear.x = 5
        self.publisher.publish(tw)
        rospy.Timer(rospy.Duration(2), self.deaccelerate, oneshot=True)

    def deaccelerate(self):
        tw = Twist()
        tw.angular.z = 0
        self.publisher.publish(tw)


def transformToBase(x, y):
    try:
        ps = PointStamped(header=Header(stamp=rospy.Time.now(),
                                        frame_id="/base_laser_link"),
                          point=Point(x, y, 0))

        # print "TRANSFORM"
        # print listener.transformPoint("/base_link", ps)
        # print
        # print "ORIGINAL"
        # print Point(x, y, 0)
    except:
        return


def mover():
    pub = rospy.Publisher('cmd_vel', Twist)
    rospy.init_node('robot_mover')

    twist = Twist()
    twist.linear.x = 1  # move forward at 0.1 m/s

    rospy.loginfo("Moving the robot forward.")
    pub.publish(twist)
    rospy.sleep(1)

    rospy.loginfo("Moving the robot backward.")
    twist.linear.x = 2  # move backward at 0.1 m/s
    pub.publish(twist)
    rospy.sleep(10)

    rospy.loginfo("Turning the robot left.")
    twist = Twist()
    twist.angular.z = 0.5
    pub.publish(twist)
    rospy.sleep(1)

    rospy.loginfo("Turning the robot right.")
    twist.angular.z = -0.5
    pub.publish(twist)
    rospy.sleep(1)

    rospy.loginfo("Stopping!")
    twist = Twist()
    pub.publish(twist)

    rospy.loginfo("Node exiting.")


def checkDistance(laserScan):
    curAngle = laserScan.angle_min
    inc = laserScan.angle_increment

    for range in laserScan.ranges:
        x = range * math.cos(curAngle)
        y = range * math.sin(curAngle)
        transformToBase(x, y)
        curAngle = curAngle + inc


if __name__ == '__main__':
    try:
        publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.init_node('my_robot_node', anonymous=True)
        #robot = MyRobot()
        # robot.accelerate()
        start = rospy.Time.now()
        duration = rospy.Duration(2)
        rate = rospy.Rate(5)
        while (rospy.Time.now() - start < duration):
            print "hello"
            tw = Twist()
            tw.linear.x = 5
            publisher.publish(tw)
            rate.sleep()

        listener = tf.TransformListener()
        laserSub = rospy.Subscriber('/base_scan', LaserScan, checkDistance)

        rospy.spin()
        mover()
    except rospy.ROSInterruptException:
        pass
