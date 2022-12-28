#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
# import tensorflow as tf

class Nodo(object):
    def __init__(self):
        # Params
        self.image = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)
        # self.model=tf.keras.models.load_model('/home/piyush/lane_ws/src/lane_mapping/model/model.h5')
        # Publishers
        self.pub = rospy.Publisher('detected_lanes', Image,queue_size=10)

        # Subscribers
        rospy.Subscriber("/my_robot/camera/image_raw",Image,self.callback)

    def callback(self, msg):
        self.image = self.br.imgmsg_to_cv2(msg,"bgr8")
        self.height,self.width=self.image.shape[:2]
        frame=cv2.cvtColor(self.image,cv2.COLOR_BGR2GRAY)
        frame=frame[int(self.height//2):,:]
        edges = cv2.Canny(frame,100,200)
        ret,thresh=cv2.threshold(edges, 100, 250, cv2.THRESH_BINARY)

        lines = cv2.HoughLinesP(thresh, 1, np.pi / 180, 30,maxLineGap=100)
        
        mask=np.zeros(shape=(self.height,self.width))

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(self.image[int(self.height//2):,:], (x1, y1), (x2, y2), (255, 255, 255), 3)
                cv2.line(mask[int(self.height//2):,:], (x1, y1), (x2, y2), (255, 255, 255), 3)
        self.pub.publish(self.br.cv2_to_imgmsg(self.image))
        cv2.imshow("frame",mask)
        cv2.waitKey(1)


    def start(self):
        c=0
        while not rospy.is_shutdown():
            if self.image is not None:
                c=0
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node("lane_detect")
    my_node = Nodo()
    my_node.start()