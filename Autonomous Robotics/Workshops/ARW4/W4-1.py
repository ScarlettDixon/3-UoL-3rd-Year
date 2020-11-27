import rospy
import numpy
from geometry_msgs.msg import Twist 
from std_msgs.msg import Float32

wheel_radius = numpy.float32(0.1)
robot_radius = numpy.float32(1.0)


# computing the forward kinematics for a differential drive
#c_l = circumfrance_left
#c_r = circumfrance_right
#w_l = angularvelocity_left
#w_r = angularvelocity_right
def forward_kinematics(w_l, w_r):
	c_l = wheel_radius * w_l
	c_r = wheel_radius * w_r
	v = (c_l + c_r) / 2
	a = (c_r - c_l) / (2 * robot_radius)
	return (v, a)


# computing the inverse kinematics for a differential drive
def inverse_kinematics(v, a):
	c_l = v - (robot_radius * a)
	c_r = v + (robot_radius * a)
	w_l = c_l / wheel_radius
	w_r = c_r / wheel_radius
	return (w_l, w_r)


# inverse kinematics from a Twist message (This is what a ROS robot has to do)
def inverse_kinematics_from_twist(t):
	return inverse_kinematics(t.linear.x, t.angular.z)

def callback(left_inp):
	#rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
	left = left_inp.data
	(v, a) = forward_kinematics(left, 0.0)
	t2 = Twist()
	t2.linear.x = v
	t2.angular.z = a
	wheelpub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size = 2)
	wheelpub.publish(t2)
# Main
if __name__ == "__main__":

	(w_l, w_r) = inverse_kinematics(0.0, 1.0)
	print "w_l = %f,\tw_r = %f" % (w_l, w_r)

	(v, a) = forward_kinematics(w_l, w_r)
	print "v = %f,\ta = %f" % (v, a)

	#from geometry_msgs.msg import Twist
	t = Twist()

	t.linear.x = 0.3
	t.angular.z = 0.8
	
	(w_l, w_r) = inverse_kinematics_from_twist(t)
	print "w_l = %f,\tw_r = %f" % (w_l, w_r)

	rospy.init_node('WheelLeftSub', anonymous=True)
	W_sub = rospy.Subscriber("/wheel_vel_left", Float32, callback)
	rospy.spin()
	#self.pub()
#rostopic pub /wheel_vel_left std_msgs/Float32 "data: 1.0" -r 10
