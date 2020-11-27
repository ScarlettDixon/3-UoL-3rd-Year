#!/usr/bin/env python

import numpy

import cv2
import cv_bridge
import rospy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image,
                                          self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist,
                                           queue_size=1)
        self.twist = Twist()

    def image_callback(self, msg):
        #cv2.namedWindow("window", 1)
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #lower_yellow = numpy.array([10, 10, 10])
        #upper_yellow = numpy.array([255, 255, 250])
	#lower_yellow = numpy.array([50, 90, 30])
        #upper_yellow = numpy.array([70, 110, 50])
	lower_yellow = numpy.array([20, 100, 100]) #Half the value given by the converter
        upper_yellow = numpy.array([40, 255, 255]) #Half the value given by the converter
	lower_red = numpy.array([0, 100, 100])
        upper_red = numpy.array([20, 255, 250])
	lcoke_red = numpy.array([0, 95, 90])
	ucoke_red = numpy.array([10, 105, 100])
	low_bgr = numpy.array([100, 100, 100])
	high_bgr = numpy.array([104, 104, 104])
        #mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
	#mask = cv2.inRange(hsv, lcoke_red, ucoke_red)
	#mask = cv2.inRange(image, low_bgr, high_bgr)
	mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        h, w, d = image.shape
       # search_top = 3*h/4 # Top 3/4 ignored
       # search_bot = 3*h/4 + 20 #Bottom is 3/4 + 20 pixels
	search_top = h/2 # Top 3/4 ignored
        search_bot = h - 30
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            err = cx - w/2
            self.twist.linear.x = 0.2
            self.twist.angular.z = -float(err) / 100
            print self.twist.angular.z

            self.cmd_vel_pub.publish(self.twist)
        cv2.imshow("window_bgr", image)
	#cv2.imshow("window_hsv", hsv)
        cv2.waitKey(3)


cv2.startWindowThread()
rospy.init_node('follower')
follower = Follower()
rospy.spin()

cv2.destroyAllWindows() 
