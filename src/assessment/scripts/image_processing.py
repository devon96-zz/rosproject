#!/usr/bin/env python2
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from matplotlib import pyplot as plt


class image_converter:

    def __init__(self):
        rospy.init_node('image_converter', anonymous=True)
        self.bridge = CvBridge()
        self.time = rospy.Time.now()
        self.image_sub1 = rospy.Subscriber("image_0", Image, self.callback1)
        self.image_sub2 = rospy.Subscriber("image_1", Image, self.callback2)

    def callback1(self, data):
        try:
            self.image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
    def callback2(self, data):
        try:
            self.image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
    
    def disparity(self):
        try:

            frame1 = cv2.cvtColor(self.image1, cv2.COLOR_BGR2GRAY)
            frame2 = cv2.cvtColor(self.image2, cv2.COLOR_BGR2GRAY)

            stereo = cv2.StereoBM_create(numDisparities=32, blockSize=15)
            disparity = stereo.compute(frame1, frame2)
            cv2.imshow("Image window1", disparity,)
            cv2.waitKey(3)
        except Exception as e:
            print e
       
        

if __name__ == '__main__':
    ic = image_converter()

    try:
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            ic.disparity()
            rate.sleep()
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
