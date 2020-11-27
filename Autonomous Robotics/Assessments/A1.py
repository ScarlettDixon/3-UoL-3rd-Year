#!/usr/bin/env python

import numpy

import cv2
import cv_bridge
import rospy
import actionlib

from sensor_msgs.msg import Image #Displays Image data
from sensor_msgs.msg import LaserScan #Used to calculate distance to objects

from geometry_msgs.msg import Twist #Moving Turtlebot
from geometry_msgs.msg import PoseStamped #Used with Move_base
from geometry_msgs.msg import Pose #Part of PoseStamped
from geometry_msgs.msg import Point #Part of Pose
from geometry_msgs.msg import Quaternion #Part of Pose

from std_msgs.msg import Header

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal 

from nav_msgs.msg import OccupancyGrid

waypoints = [ 
    [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)], 
    [(0.5, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)]   
 ] 

class Pole_Finder:
    def __init__(self):
        
        #Declerations
        self.twist = Twist()
        
        #Subscriptions
        self.bridge = cv_bridge.CvBridge()
        self.scanning = rospy.Subscriber('/scan', LaserScan, self.scan_dist)
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback) #,callback_args=scan_dist)
        
        self.data_map = rospy.Subscriber('/map', OccupancyGrid, self.map_inp) 
        self.stop_move = False
        
        #Publications
        self.simp_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=3)
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.mv_to_goal()

    def image_callback(self, msg):#, scan_dist):
        #print(scan_dist.data)
        #print(Scan_dist.range)
        #cv2.namedWindow("window", 1)
        #self.mv_to_goal()
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #bgr_tester = numpy.uint8([[[102,0,0]]])
        #hsv_tester = cv2.cvtColor(bgr_tester ,cv2.COLOR_BGR2HSV)
        lower_blue = numpy.array([110, 100, 100])
        upper_blue = numpy.array([130, 255, 255])
        lower_green = numpy.array([50, 100, 100])
        upper_green = numpy.array([70, 255, 255])
        lower_red = numpy.array([0, 100, 100])
        upper_red = numpy.array([10, 255, 255])
        lower_yellow = numpy.array([20, 100, 100])
        upper_yellow = numpy.array([40, 255, 255])
                
        mask_b = cv2.inRange(hsv, lower_blue, upper_blue)
        mask_g = cv2.inRange(hsv, lower_green, upper_green)
        mask_r = cv2.inRange(hsv, lower_red, upper_red)
        mask_y = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask_list = [mask_b, mask_g, mask_r, mask_y]
        for i in range(0,len(mask_list)):
            mask = mask_list[i]
            h, w, d = image.shape
            search_top = h/2 # Top 1/2 ignored
            search_bot = h - 30
            mask[0:search_top, 0:w] = 0
            mask[search_bot:h, 0:w] = 0
            M = cv2.moments(mask)
            a = self.laser_dist
            if M['m00'] > 0 and a > 1.0:
               while M['m00'] > 0 and a > 1.0:
                    self.stop_move = True
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
                    err = cx - w/2
                    self.twist.linear.x = 0.2
                    self.twist.angular.z = -float(err) / 100
                    #print self.twist.angular.z
                    self.cmd_vel_pub.publish(self.twist)
            elif M['m00'] > 0 and a < 1.0:
                print("Found Object")
                #del mask_list[i]
                #self.mv_to_goal()
            cv2.imshow("window_bgr", image)
        
            print(a)
            #cv2.imshow("window_hsv", hsv)
            cv2.waitKey(3)

    #def CreatePoseSt(self, pose_data):
    def Create_Pose_St(self, x, y, w):
        posst = PoseStamped()   # header and pose  
        
        head = Header()
        head.frame_id = "map" #Has to be map to interect with it
        
        pos = Pose() # point and quarternion
        poi = Point() #x, y and z
        poi.x = x
        poi.y = y
        #po.z = 0.0
        qu = Quaternion() #x, y, z and w
        #qu.x = 0.0
        #qu.y = 0.0
        #qu.z = 0.0
        qu.w = w
        pos.position = poi
        pos.orientation = qu
        posst.header = head
        posst.pose = pos
        self.posstam = posst
        
       
    def scan_dist(self, dist_d):
        laserd = dist_d.ranges
        self.laser_dist = laserd[320]
    
    def map_inp(self, map_d):
        self.mapd = map_d.data
        #print(mapd)
    def goal_pose(self, pose): 
        goal_pose = MoveBaseGoal() 
        goal_pose.target_pose.header.frame_id = 'map' 
        goal_pose.target_pose.pose.position.x = pose[0][0] 
        goal_pose.target_pose.pose.position.y = pose[0][1] 
        goal_pose.target_pose.pose.position.z = pose[0][2] 
        #goal_pose.target_pose.pose.orientation.x = pose[1][0] 
        #goal_pose.target_pose.pose.orientation.y = pose[1][1] 
        #goal_pose.target_pose.pose.orientation.z = pose[1][2] 
        goal_pose.target_pose.pose.orientation.w = pose[1][3] 
        return goal_pose 
    
    def mv_to_goal(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server() 
        #while True: 
        for pose in waypoints: 
            #g_pose = MoveBaseGoal()
            #g_pose.target_pose = self.Create_Pose_St()
            goal = self.goal_pose(pose)
            
            self.client.send_goal(goal) 
            if self.stop_move == True:
                print("Hello Caleb")
                self.client.cancel_goal()
            #client.send_goal(g_pose) 
            self.client.wait_for_result() 
   

if __name__ == '__main__': 
    cv2.startWindowThread()    
    rospy.init_node('patrol') 
    ptrl = Pole_Finder()
    rospy.spin()
    cv2.destroyAllWindows()
    exit(0)
    
      
#print("Hello")
    
#cv2.startWindowThread()
#rospy.init_node('pl_find')
#pl_find = Pole_Finder()
#test2 = rospy.Subscriber('/move_base_simple/goal', PoseStamped, Displaypos)
#test1 = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)    
#i=0;
#while i < 8 :
#test1.publish(posst)
#i +=1
#print(test2.Pose.position)


#roslaunch uol_turtlebot_simulator object-search-training.launch
#roslaunch turtlebot_rviz_launchers view_robot.launch

#Open Rviz and add map, then change topic to /map
#interesting topics:
#/camera/depth/points - sensor_msgs/PointCloud2
#/initialpose - geometry_msgs/PoseWithCovarianceStamped
#/move_base_simple/goal - geometry_msgs/PoseStamped
#/move_base/goal - move_base_msgs/MoveBaseActionGoal
#/move_base/current_goal - geometry_msgs/PoseStamped
#rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'

#Research:
#Using HSV:
#https://stackoverflow.com/questions/45070661/pythonopencv-how-to-plot-hsv-range/45071147#45071147
#https://docs.opencv.org/3.2.0/df/d9d/tutorial_py_colorspaces.html

#What I need to do:
    #subsribing to get image data that picks up hsv data to find the four poles
    #Some way of publishing data to the robot to move it to a position
    #have the robot spin around to see if it sees a particular colour
    #Move towards that specific colour and mark it off as found





