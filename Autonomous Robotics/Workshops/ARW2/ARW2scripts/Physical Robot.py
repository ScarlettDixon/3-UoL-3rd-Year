# -*- coding: utf-8 -*-
"""
Created on Fri Feb  1 10:29:37 2019

@author: student
"""

import rospy
from geometry_msgs.msg import Twist # This is the message type the robot uses for velocities


class CommandVelocity():
    """Driving my robot
    """

    def __init__(self):
        rospy.loginfo("Starting node")
        self.pub = rospy.Publisher("/mobile_base/commands/velocity", Twist) # Creating a publisher with whatever name...
        
    # A function to send velocities until the node is killed
    def send_velocities(self):
        r = rospy.Rate(10) # Setting a rate (hz) at which to publish
        while not rospy.is_shutdown(): # Runnin until killed
            rospy.loginfo("Sending commands")
            twist_msg = Twist() # Creating a new message to send to the robot
            twist_msg.linear.x = 0.4
            twist_msg.linear.y = 0.0
            twist_msg.linear.z = 0.0
            
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0  
            twist_msg.angular.z = 1.4  
            # ... put something relevant into your message

            self.pub.publish(twist_msg) # Sending the message via our publisher
            r.sleep() # Calling sleep to ensure the rate we set above

if __name__ == '__main__':
    rospy.init_node("command_velocity")
    cv = CommandVelocity()
    cv.send_velocities() # Calling the function
    rospy.spin()
