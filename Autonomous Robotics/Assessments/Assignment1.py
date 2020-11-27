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
from std_msgs.msg import String

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback

from actionlib_msgs.msg import GoalStatusArray

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry

lower_blue = numpy.array([110, 100, 100])
upper_blue = numpy.array([130, 255, 255])
lower_green = numpy.array([50, 100, 100])
upper_green = numpy.array([70, 255, 255])
lower_red = numpy.array([0, 100, 100])
upper_red = numpy.array([4, 255, 255])
lower_yellow = numpy.array([20, 100, 100])
upper_yellow = numpy.array([40, 255, 255])


class Pole_Finder:
    ###Initialiser###
    def __init__(self):
        
        ###Definitions###
        #self.laser_dist
        self.bridge = cv_bridge.CvBridge()
        cv2.startWindowThread()
        self.twist = Twist()
        self.stop_move = False
        self.stat_list_te = ""
        self.found_Pole = [False, False, False, False]
        self.pole_nm = ["Blue","Green","Red","Yellow","None"]
        self.blue=0
        self.green=1
        self.red=2
        self.yellow=3
        self.mask_all = numpy.array([]) 
        self.curr_goal = 0
        self.locations = [ 
        [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)], 
        [(3.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)],  
        [(3.0, 2.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
        [(-1.0, 2.0,0.0), (0.0, 0.0, 0.0, 1.0)], 
        [(-1.0, 3.0,0.0), (0.0, 0.0, 0.0, 1.0)],
        [(-1.0, 4.0,0.0), (0.0, 0.0, 0.0, 1.0)], 
        [(1.0, 4.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
        [(3.0, 4.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
        [(1.0, 4.0, 0.0), (0.0, 0.0, 0.0, 1.0)], 
        [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
        [(2.0,-4.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
        [(-3.0,-4.0,0.0), (0.0, 0.0, 0.0, 1.0)],
        [(1.0, -4.0,0.0), (0.0, 0.0, 0.0, 1.0)],
        [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
        [(-4.0,-2.0,0.0), (0.0, 0.0, 0.0, 1.0)],
        [(-4.0,0.0, 0.0), (0.0, 0.0, 0.0, 1.0)], 
        [(-3.0,3.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
        [(-4.5,4.0,0.0), (0.0, 0.0, 0.0, 1.0)],
        [(-4.5,1.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
        [(-4.0,-1.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
        [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)]
        ]
        #self.usemask
        self.cancelled = False
        ###Clients###
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        ###Subscriptions###
        self.scanning = rospy.Subscriber('/scan', LaserScan, self.scan_inp)
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_inp)
        self.data_map = rospy.Subscriber('/map', OccupancyGrid, self.map_inp)
        self.move_stat = rospy.Subscriber('/move_base/status', GoalStatusArray, self.stat)
        self.move_feed = rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, self.feed)
        self.odometry = rospy.Subscriber('/odom', Odometry, self.odome)
        ###Publications###
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.simp_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=3)
        self.deb_pub = rospy.Publisher('/debug', String, queue_size=10)
        ###CallMainFunction###
        self.main_func()
    
    ###Subscriber Functions###
    """
    #Import both for knowing how far the pole is away from the robot 
    and was also used for object detection
    """
    def scan_inp(self, dist_d):
        laserd = numpy.array(dist_d.ranges)
        laser_middle = int(len(laserd) / 2)
        self.laser_min = dist_d.range_min
        #print(laserd)
        #middle_las = int((laserd.size)/2) #Right at the centre of the laser array
        #print (middle_las)
        self.laser_dist = laserd[laser_middle] #Should be around 320
    """    
    Creates the masks to search for the poles
    """
    def image_inp(self, Img_d):
        #Had issues with computational power needed to do calculations 
        #this stopped the image showing without lagging so need to take computations out
        self.image = self.bridge.imgmsg_to_cv2(Img_d, desired_encoding='bgr8')
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        self.img_output = hsv
        self.h, self.w, self.d = self.image.shape
        
        mask_b = cv2.inRange(self.img_output, lower_blue, upper_blue)
        mask_g = cv2.inRange(self.img_output, lower_green, upper_green)
        mask_r = cv2.inRange(self.img_output, lower_red, upper_red)
        mask_y = cv2.inRange(self.img_output, lower_yellow, upper_yellow)
       # mask_list = numpy.array([])        
        #mask_check = numpy.empty((self.h, self.w))
        if (not(self.found_Pole[self.blue])):
            search_top = self.h/2 # Top 1/2 ignored
            search_bot = self.h - 30
            mask_b[0:search_top, 0:self.w] = 0
            mask_b[search_bot:self.h, 0:self.w] = 0
            M = cv2.moments(mask_b)
            if M['m00'] > 1000000:
                self.move_pole(M, self.blue)
           # mask_list = numpy.append(mask_list, mask_b)
        if (not(self.found_Pole[self.green])):
            search_top = self.h/2 # Top 1/2 ignored
            search_bot = self.h - 30
            mask_g[0:search_top, 0:self.w] = 0
            mask_g[search_bot:self.h, 0:self.w] = 0            
            M = cv2.moments(mask_g)
            if M['m00'] > 1000000:
                self.move_pole(M, self.green)
           # mask_list = numpy.append(mask_list, mask_g)
        if (not(self.found_Pole[self.red])):
            search_top = self.h/2 # Top 1/2 ignored
            search_bot = self.h - 30
            mask_r[0:search_top, 0:self.w] = 0
            mask_r[search_bot:self.h, 0:self.w] = 0            
            M = cv2.moments(mask_r)
            if M['m00'] > 1000000:
                self.move_pole(M, self.red)
           # mask_list = numpy.append(mask_list, mask_r)
        if (not(self.found_Pole[self.yellow])):
            search_top = self.h/2 # Top 1/2 ignored
            search_bot = self.h - 30
            mask_y[0:search_top, 0:self.w] = 0
            mask_y[search_bot:self.h, 0:self.w] = 0
            M = cv2.moments(mask_y)
            if M['m00'] > 1000000:
                self.move_pole(M, self.yellow)
            #mask_list = numpy.append(mask_list, mask_y)
        #self.mask_all = numpy.append(self.mask_all, mask_list)
        #try:
            #print("Distance straight ahead is: " + str(self.laser_dist))
        #except:
           # print("Can't find laser data")
        cv2.imshow("window_bgr", self.image)
        cv2.waitKey(5)
    """    
    An attempt to increase resiliency of movebase by using the map to calculate locations to visit
    This would involve first taking in the 1D array of map data and converting it into a 2D display
    Then the image would be either segmented or randomly searched both finding locations free of obstacles
    Segmentation: split the image into areas of open space and calculate locations based on that
    Random Search: Pick out random empty spaces and go there, mark off areas you've vistied between the goals so to 
    eventually reduce all possible locations 
    """      
    def map_inp(self, map_d):
        self.map_data = map_d.data
        self.map_width = map_d.info.width
        self.map_height = map_d.info.height
        self.map_resol = map_d.info.resolution
        #self.map_store = numpy.reshape(self.map_data, (self.map_width,self.map_height))
        self.map_store = numpy.array(self.map_data).reshape(self.map_width,self.map_height)
        #test = cv2.CreateMat()
        #map_store[map_store == 100] = 0
        #map_store[map_store == 0] = 255
        #map_store[map_store == -1] = 128
        #cv2.imwrite('map_test.jpg', self.map_store)
        #print (self.map_store)
        #print (self.map_store.shape)
        #cv2.imshow("map_test", self.map_store);
        #cv2.waitKey();
        
    """    
    Give the status of the goals being completed, were useful for debugging
    """
    def stat(self, stat_d):
       stat_list = stat_d.status_list
       try:
           self.stat_list_st = stat_list[0].status
           self.stat_list_te = stat_list[0].text
           print(str(self.stat_list_st) + ": " + self.stat_list_te)
       except:
           return
           
    """    
    feedback did a similar job to status
    """
    def feed(self, feed_d):
       self.feed_data = feed_d
       
    """
    Odometry data was going to be taken in to stop the problem of no object detection.
    The aim was to calculate how far the pole was way and calculate a new movebase goal,
    which has object avoidance built in by using map, closer to the pole until it was found
    unfortuantley ran out of time to complete this
    """
    def odome(self, odom_d):
       self.odom_data = odom_d
     
     
    """
    Called if pole is seen in the bottom half of the image, will then move towards the pole
    """
    def  move_pole(self, M, num):
        if self.cancelled == False:
            self.client.cancel_goal()
            self.cancelled = True
        self.stop_move = True
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        cv2.circle(self.image, (cx, cy), 20, (0, 0, 255), -1)
        err = cx - self.w/2
        self.twist.linear.x = 0.5
        self.twist.angular.z = -float(err) / 100
        self.cmd_vel_pub.publish(self.twist)
        if self.laser_dist <= 1.0:# and M['m00'] < 25000000:
            self.deb_pub.publish(str("Found " + self.pole_nm[num]))
            print("Found " + self.pole_nm[num])
            self.found_Pole[num] = True
            self.stop_move = False
            self.cancelled = False
    
    """
    The Main function for this class, created after subscriptions and publications are initialised
    will attempt to use Move_base to move arround the map unless a pole is found, it will at that point
    give over to the move_pole function
    """
    def main_func(self): 
        t = rospy.Time(10).secs # t.secs=10
        print(t)
        while False in self.found_Pole:
            if self.stop_move == False and self.curr_goal < len(self.locations):
                goal = self.goal_pose(self.locations[self.curr_goal])
                self.client.send_goal(goal)
                self.client.wait_for_result(rospy.Duration.from_sec(30.0))#rospy.Duration(20))#.to_sec())
                self.curr_goal +=1
            elif self.curr_goal >= len(self.locations):
                break
                #print (self.curr_goal)
            
        print("Found All Colours")
        self.deb_pub.publish("Found All Colours")
        goal = self.goal_pose(self.locations[0])
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(30.0))
        exit(0)
        
    """
    Used to define the goal in a way that is acceptable to the move_base
    """
    def goal_pose(self, pose): 
        goal_pose = MoveBaseGoal() 
        goal_pose.target_pose.header.frame_id = 'map' 
        goal_pose.target_pose.pose.position.x = pose[0][0] 
        goal_pose.target_pose.pose.position.y = pose[0][1] 
        goal_pose.target_pose.pose.position.z = pose[0][2] 
        goal_pose.target_pose.pose.orientation.x = pose[1][0] 
        goal_pose.target_pose.pose.orientation.y = pose[1][1] 
        goal_pose.target_pose.pose.orientation.z = pose[1][2] 
        goal_pose.target_pose.pose.orientation.w = pose[1][3] 
        return goal_pose  
 
"""
Initial setup, creating a node and class
"""     
if __name__ == '__main__': 
    cv2.startWindowThread()    
    rospy.init_node('Pole_searcher') 
    ptrl = Pole_Finder()
    rospy.spin()
    cv2.destroyAllWindows()
    exit(0)
    

 #All code needed to launch a session of ROS   
'''
roscore
roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find turtlebot_gazebo)/worlds/empty.world
roslaunch uol_turtlebot_simulator object-search-training.launch
roslaunch turtlebot_rviz_launchers view_robot.launch
'''  
    
#Below is chunks of code with failed implimentation 

"""        
    def search(self):#, mask_all):
        #print(self.mask_all) 
        i = 0
        dec = False
        while i < 4:
            #print(self.mask_all)
            try:
                M = cv2.moments(self.mask_all[i]) 
                if M['m00'] > 0 and self.found_Pole[i] == False:
                    dec = True
                    #print("Pole In View")
                    i = 0
                    break
                else:
                    print("Pole not in view")
                i+=1 
            except:
                dec = True
                i = 4
        return dec, i
            
    def move(self):
       while False in self.found_Pole:
           if self.stop_move == False:
               goal = self.goal_pose(self.locations[self.curr_goal])
            #print(goal)
               
               self.deb_pub.publish(str(self.stat_list_te))
               self.client.send_goal(goal) 
               
               found_item_decision, pole_used = self.search()
               if found_item_decision == True:
                   self.client.cancel_goal()
                   self.move_pole(pole_used)
               self.client.wait_for_result(rospy.Duration(20))#.to_sec())
               self.stop_move = True
           elif self.stop_move == True:
               self.search()
           self.found_Pole = [True, True, True, True]      
    def  move_pole(self, p_u)  : 
        print("Moving to pole " + str(p_u))        
        
        while self.laser_dist > 0.4:
            M = cv2.moments(self.mask_all[p_u])
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(self.image, (cx, cy), 20, (0, 0, 255), -1)
            err = cx - self.w/2
            self.twist.linear.x = 0.2
            self.twist.angular.z = -float(err) / 100
            print self.twist.angular.z
            self.cmd_vel_pub.publish(self.twist)
            
    #self.client.cancel_goal()
    ###Moving Functions
    #def circle_spin(self):
        #2pi = 6.28318530718        
        #print (0.2 * rospy.Duration(20, 0).to_sec())
        #self.twist.angular.z = 0.2
        #spinangle = 0
        #while spinangle < 6.3:
        #    self.cmd_vel_pub.publish(self.twist)
        #    spinangle += 0.2 * int(rospy.Duration(1).to_sec())        
                    
        
         
        a = self.laser_dist
        if M['m00'] > 0 and a > 1.0:
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
        for pose in self.locations:
            goal = self.goal_pose(pose)
            #print(goal)
            self.deb_pub.publish(str(self.stat_list_te))
            self.client.send_goal(goal) 
            #self.client.cancel_goal()
            self.client.wait_for_result(rospy.Duration(20))
        
        while True:
            for mask in self.mask_list:  
                h, self.w, d = self.image.shape
                search_top = h/2 # Top 1/2 ignored
                search_bot = h - 30
                mask[0:search_top, 0:self.w] = 0
                mask[search_bot:h, 0:self.w] = 0
                self.M = cv2.moments(mask)
                dist = self.laser_dist
                if self.M['m00'] > 0 and dist > 1.0:
                    self.usemask = mask
                    self.changed = True
                elif self.M['m00'] > 0 and dist <= 1.0:
                    print("Found")
            if self.changed == True and dist > 1.0:
                while dist > 1.0:
                    self.usemask[0:search_top, 0:self.w] = 0
                    self.usemask[search_bot:h, 0:self.w] = 0
                    self.M = cv2.moments(self.usemask)
                    cx = int(self.M['m10']/self.M['m00'])
                    cy = int(self.M['m01']/self.M['m00'])
                    cv2.circle(self.image, (cx, cy), 20, (0, 0, 255), -1)
                    err = cx - self.w/2
                    self.twist.linear.x = 0.2
                    self.twist.angular.z = -float(err) / 100
                    self.cmd_vel_pub.publish(self.twist)
        exit(0)
     #circ_spin = Twist()
#        circ_spin.linear.x = 0.0
#        circ_spin.angular.z = 0.5
#        accum_dist = 0.0
#        while accum_dist < 2*(math.pi):
#            self.cmd_vel_pub.publish(circ_spin)
#            accum_dist += circ_spin.angular.z
#        circ_spin.angular.z = 0.0
#        self.cmd_vel_pub.publish(circ_spin)   
    
    
    
"""    
  
    
    
    
