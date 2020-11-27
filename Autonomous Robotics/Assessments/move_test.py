#!/usr/bin/env python

import rospy
import numpy as np
import actionlib
import cv2
#from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from actionlib_msgs.msg import GoalStatusArray, GoalID#, GoalStatus
from nav_msgs.msg import OccupancyGrid

waypoints = [ 
    [(1.0, 1.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
    [(-1.0, 3.0, 0.0), (0.0, 0.0, 0.0, 1.0)], 
    [(-1.0, 4.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
    [(-4.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
    [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)] 
 ] 
class Pole_Finder:
    def __init__(self):
        print("initialised")
        self.data_map = rospy.Subscriber('/map', OccupancyGrid, self.map_inp)
        self.goal_info = rospy.Subscriber('/move_base/status', GoalStatusArray, self.stat)
        self.move_feed = rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, self.feed)
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
        
    def map_inp(self, map_d):
        map_width = map_d.info.width
        map_height = map_d.info.height
        #map_resol = map_d.info.resolution
        map_store = np.zeros([map_width, map_height])
        map_data = map_d.data
        map_store = np.reshape(map_data, (map_width,map_height))
        #map_store[map_store == 100] = 0
        #map_store[map_store == 0] = 255
        #map_store[map_store == -1] = 128
        
        #cv2.imwrite('map_test.jpg', map_store)
        cv2.imshow("image", map_store);
        cv2.waitKey();        
        np.set_printoptions(threshold=np.nan)
        #print(map_store)
        
    def stat(self, stat_d):
       stat_list = stat_d.status_list
       stat_list_st = stat_list[0].status
       stat_list_te = stat_list[0].text
       print(stat_list_st)  
       print(stat_list_te)  
      
      
     
    def feed(self, feed_d):
        #print(feed_d)
        return 0 
       


if __name__ == '__main__':    
    rospy.init_node('pl_f') 
    pl_f = Pole_Finder()
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction) #client creation
    client.wait_for_server() #wait until connected to server
    #while True: 
    for pose in waypoints: 
        #g_pose = MoveBaseGoal()
        #g_pose.target_pose = self.Create_Pose_St()
        goal = pl_f.goal_pose(pose)
        #print(goal)
        client.send_goal(goal) 
        #client.send_goal(g_pose) 
        client.wait_for_result() 