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
import depth_image_proc


class image_converter:

    def __init__(self):
        rospy.init_node('image_converter', anonymous=True)
        self.bridge = CvBridge()
        self.time = rospy.Time.now()
        self.image_sub = rospy.Subscriber("image", Image, self.callback)
        self.cv_image = Image()

    def callback(self, data):
        if (rospy.Time.now() - self.time).to_sec() < 1:
            return
        self.time = rospy.Time.now()
        try:
            old_image = self.cv_image
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # self.cv_image = cv2.inRange(self.cv_image, (0, 0, 0), (0, 0, 0))

        sift = cv2.xfeatures2d.SIFT_create()

        kp1, des1 = sift.detectAndCompute(old_image, None)
        kp2, des2 = sift.detectAndCompute(self.cv_image, None)

        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)   # or pass empty dictionary

        flann = cv2.FlannBasedMatcher(index_params, search_params)

        matches = flann.knnMatch(des1, des2, k=2)

        # Need to draw only good matches, so create a mask
        matchesMask = [[0, 0] for i in xrange(len(matches))]

        # ratio test as per Lowe's paper
        for i, (m, n) in enumerate(matches):
            if m.distance < 0.7 * n.distance:
                matchesMask[i] = [1, 0]

        draw_params = dict(matchColor=(0, 255, 0),
                           singlePointColor=(255, 0, 0),
                           matchesMask=matchesMask,
                           flags=0)

        img3 = cv2.drawMatchesKnn(
            old_image, kp1, self.cv_image, kp2, matches, None, **draw_params)

        cv2.imshow("Image window", img3)
        cv2.waitKey(3)


def main(args):
    ic = image_converter()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
