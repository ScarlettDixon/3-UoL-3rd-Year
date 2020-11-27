#!/usr/bin/env python

import rospy
import cv2
import numpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String


class image_converter:

    def __init__(self):

        cv2.startWindowThread()
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",
                                          Image, self.callback)
        # self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",
        #                                   Image, self.callback)
	
    def callback(self, data):
        cv2.namedWindow("Image window", 1)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e

        while not rospy.is_shutdown():
            bgr_thresh = cv2.inRange(cv_image,
                                     numpy.array((0, 102, 102)),
                                     numpy.array((0, 102, 102)))
    
            hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            hsv_thresh = cv2.inRange(hsv_img,
                                     numpy.array((90, 150, 0)),
                                     numpy.array((180, 255, 255)))
            
            pub = rospy.Publisher("/Img_Output", String, queue_size = 1)
            #r = rospy.rate(10)
     #       while not rospy.is_shutdown():
      #          #rospy.loginfo("test")
       #         outputData = String()
        #        outputData = numpy.mean(hsv_img[:, :, 0])
         #       outputData = numpy.mean(hsv_img[:, :, 1])
          #      outputData = numpy.mean(hsv_img[:, :, 2])
            #    self.pub.publish(outputData)
                 #r.sleep()
            
            Out = str(numpy.mean(hsv_img[:, :, 0])) + " Blue ||";
            Out = Out + ' ' + str(numpy.mean(hsv_img[:, :, 1])) + " Green ||";
            Out = Out + ' ' + str(numpy.mean(hsv_img[:, :, 2])) + " Red";
            Out = Out + ' ' + "===="
            pub.publish(Out);
            
            #print numpy.mean(hsv_img[:, :, 0])
            #print numpy.mean(hsv_img[:, :, 1])
            #print numpy.mean(hsv_img[:, :, 2])
    
            _, bgr_contours, hierachy = cv2.findContours(
                bgr_thresh.copy(),
                cv2.RETR_TREE,
                cv2.CHAIN_APPROX_SIMPLE)
    
            _, hsv_contours, hierachy = cv2.findContours(
                hsv_thresh.copy(),
                cv2.RETR_TREE,
                cv2.CHAIN_APPROX_SIMPLE)
            for c in hsv_contours:
                a = cv2.contourArea(c)
                if a > 100.0:
                    cv2.drawContours(cv_image, c, -1, (255, 0, 0))
           # print '===='
            #cv2.imshow("Image window", cv_image)
	    cv2.imshow("Image window", bgr_thresh)
	    #cv2.imshow("Image window", hsv_img)
	    #cv2.imshow("Image window", hsv_thresh)	
            cv2.waitKey(10)

	#def StOut(self):
	#	r = rospy.rate(10)
	#	while not rospy.is_shutdown():
	#	    rospy.loginfo("test")
	#	    outputData = String()
	#	    #outputData = 
	#	    self.pub.publish(outputData)
	#	    r.sleep()
image_converter()
rospy.init_node('image_converter', anonymous=True)
#ic = image_converter()
#ic.StOut()
rospy.spin()
cv2.destroyAllWindows()
