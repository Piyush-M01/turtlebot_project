#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2 as cv
import numpy as np
import math
import os
import uuid
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ast import literal_eval

class image_converter:

    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)


    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        img = cv_image
        rows, columns, _ = img.shape

        img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        
        lower_green = np.array([20, 86, 6])
        upper_green = np.array([64, 255, 255])

        mask = cv.inRange(img_hsv, lower_green, upper_green)

        erode = cv.erode(mask, None, iterations = 2)
        dilate = cv.dilate(erode, None, iterations = 2)

        guassian_blur = cv.GaussianBlur(dilate,(3,3),0)
        median = cv.medianBlur(guassian_blur, 5)

        img_copy = img.copy()

        cnts, hierarchy = cv.findContours(image = median, mode = cv.RETR_TREE, method = cv.CHAIN_APPROX_NONE)

        center = None
        if cnts:
            c = max(cnts, key = cv.contourArea)
            ((x, y), radius) = cv.minEnclosingCircle(c)
            M = cv.moments(c)
            area = cv.contourArea(cnts[0])

            if radius < 100:
                cv.circle(img_copy, (int(x), int(y)), int(radius), (255, 0, 0), 2)
                
                cv.putText(img_copy, 'The center coordinates are : {center_x}, {center_y}'.format(center_x = int(x), center_y = int(y)), (10,30), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv.LINE_AA)
            print("radius is : ", radius)

        cv.imshow("contour", img_copy)

        cv.waitKey(3)


def main(args):

    ic = image_converter()

    rospy.init_node('image_converter_2', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
