#!/usr/bin/env python3
import roslib
import numpy as np, cv2
import math
from mpmath import mp
import rospy
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge, CvBridgeError

from dynamic_reconfigure.server import Server 
from lane_mapping.cfg import parametersConfig

class image_to_laser:

    def __init__(self):
        # Laser Scan To Subscribe to
        self.joy_sub = rospy.Subscriber("/detected_lanes",Image,self.img_to_laser_callback)
        # Publisher for Image
        self.pub = rospy.Publisher("/img_scan",LaserScan, queue_size = 100)
        # CvBridge Setup
        self.br = CvBridge()
        self.scan=LaserScan()
        self.scaling_factor=500000

        # disc factor = original length of object (metres) / width of the object in image
        self.disc_factor=0.0009525
        
        self.srv = Server(parametersConfig, self.callback)
        self.angle_min=-math.pi/2
        self.angle_max=math.pi/2
        self.samples=360
        # self.angle_increment=(abs(self.angle_max)+abs(self.angle_min))/self.samples
        self.angle_increment=1

    def callback(self,config, level):
        
        #find the appropriate scaling factor using dynamic reconfigure 
        # self.scaling_factor=config.scaling_factor

        # self.disc_factor=config.disc_factor
        self.angle_increment=config.angle_increment
        # self.range_size=int(180/math.degrees(self.angle_increment))
        
        # print(self.scaling_factor)
        return config

    def convert_pix_to_coord(self,pix_x,pix_y):

        pt_x=(pix_x/self.disc_factor)
        pt_y=(pix_y/self.disc_factor)

        if(pt_x != 0):
            theta=math.atan(pt_y/pt_x)
        else:
            theta=math.pi/2

        r=pt_x/mp.cos(theta)
        theta=int(math.degrees(theta))


        return r,theta

    def img_to_laser_callback(self,msg):

        self.image = self.br.imgmsg_to_cv2(msg)
        self.height,self.width=self.image.shape[:2]
        
        frame=cv2.cvtColor(self.image,cv2.COLOR_BGR2GRAY)
        frame=frame[int(self.height//3):,:]
        edges = cv2.Canny(frame,100,200)
        ret,thresh=cv2.threshold(edges, 100, 250, cv2.THRESH_BINARY)

        ranges = dict()
        intensities=dict()


        for i in range(-180,180,1):
            ranges[i]=0
            intensities[i]=0

        lines = cv2.HoughLinesP(thresh, 1, np.pi / 180, 30,maxLineGap=100)
        
        mask=np.zeros(shape=(self.height,self.width))

        # following specifications and lane detection code works 
        #
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                r,theta=self.convert_pix_to_coord(x1,y1)
                ranges[theta]=abs(r)/self.scaling_factor
                intensities[theta]=10
                r,theta=self.convert_pix_to_coord(x2,y2)
                ranges[theta]=abs(r)/self.scaling_factor
                intensities[theta]=100
            
        self.scan.angle_max=self.angle_max
        self.scan.angle_min=self.angle_min
        self.scan.angle_increment=0.017453


        # self.ranges=[]
        # for i in ranges.keys():
        #     self.ranges.append(ranges[i])

        # print(self.ranges)


        # this method makes code slower and the laser points are getting projected behind the bot
        # inv=cv2.flip(self.image,0)
        # coords=np.where(self.image == 255)
        # coordinates = zip(coords[0], coords[1])
        # j=1
        # for i in coordinates:
        #     # print("coords",i)
        #     # if j<50:
        #     r,theta=self.convert_pix_to_coord(i[0],i[1])
        #     ranges[theta]=abs(r)/self.scaling_factor
        #     intensities[theta]=10
        #         # j=j+1

        # self.scan.angle_max=3.14*4
        # self.scan.angle_min=-3.14*4
        # self.scan.angle_increment=-0.0174533

        # for i in ranges.values():
        #     print(i)
        # print(list(ranges.values()))



        self.scan.header.frame_id="laser"
        self.scan.header.stamp.secs=rospy.get_rostime().secs
        self.scan.header.stamp.nsecs=rospy.get_rostime().nsecs
        self.scan.time_increment=0.0
        self.scan.scan_time=0.0
        self.scan.range_min=0.04
        self.scan.range_max=10.0
        self.ranges=[]
        self.intensities=[]
        self.ranges=list(ranges.values())
        self.intensities=list(intensities.values())
        self.scan.ranges=self.ranges
        self.scan.intensities=self.intensities
        self.pub.publish(self.scan)


if __name__=='__main__':
	rospy.init_node('image_to_laser')
	laser_to_image = image_to_laser()
	rospy.spin()    
