#!/usr/bin/env python

import numpy

import cv2
import cv_bridge
import rospy
import actionlib

from sensor_msgs.msg import Image #Displays Image data
from sensor_msgs.msg import LaserScan #Used to calculate distance to objects
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal 
from rosgraph_msgs.msg import Log

laser_dist = 0.0

def laserData(ld):
    laserd = ld.ranges
    #lasersz = len(laserd) # 640 values
    laser_dist = 101.0
    #laserd[320]

def DebugData(Dd):
    print (Dd.data)
    #return Dd.data

if __name__ == '__main__':
    rospy.init_node('Dbug', anonymous=True)
    #Display = rospy.Subscriber('/Debug', String, DebugData)
    scanning = rospy.Subscriber('/scan', LaserScan, laserData)
    simp_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=3)
    while not rospy.is_shutdown():
        deb_pub = rospy.Publisher('Debug', String, queue_size=10)
        deb_pub.publish(str(laser_dist))
        
        posst = PoseStamped()   # header and pose  
        
        head = Header()
        head.frame_id = "map" #Has to be map to interect with it
        
        pos = Pose() # point and quarternion
        poi = Point() #x, y and z
        poi.x = 0
        poi.y = 0
        #po.z = 0.0
        qu = Quaternion() #x, y, z and w
        #qu.x = 0.0
        #qu.y = 0.0
        #qu.z = 0.0
        qu.w = 1.0
        pos.position = poi
        pos.orientation = qu
        posst.header = head
        posst.pose = pos
        simp_pub.publish(posst)
        rospy.sleep(1)
#rostopic pub  geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}
#, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 1.0}}}'
        
        #DebugData
    #
    #rospy.spin()
    #
    #image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image)
    
    #ros_pub.pub
    #print(scanning.callback)
