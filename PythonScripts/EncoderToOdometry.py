#!/usr/bin/python

# import libraries
import time

# ROS libraries
import rospy
import std_msgs.msg

# define settings
WheelDiameter = 0.10
DistancePerCount = (3.14159 *  WheelDiameter) / 64

_PreviousLeftEncoderCounts = 0
_PreviousRightEncoderCounts = 0
last_time_encoder = 0

x = 0
y = 0
th = 0

vx = 0
vy = 0
vth = 0

delta_left = 0
delta_right = 0

def wheel_callback(left_encoder, right_encoder):
	current_time_encoder = time.time()
	
	delta_left = left_encoder - _PreviousLeftEncoderCounts
	delta_right = right_encoder - _PreviousRightEncoderCounts
	
	vx = delta_left * DistancePerCount
	vy = delta_right * DistancePerCount
	
	_PreviousLeftEncoderCounts = left_encoder
	_PreviousRightEncoderCounts = right_encoder
	last_time_encoder = current_time_encoder
	
	return

def main():
	# setup publishing odometry messages
	pub = rospy.Publisher('odom')
	rospy.init_node('odom_publisher',anonymous=True)
	
	# subscribe to wheel encoder messages
	encoder_ros_update = rospy.Subscriber("/%s/get/encoder_status" % socket.gethostname(), std_msg.Int32MultiArray, wheel_callback)
	
	current_time = time.time()
	last_time = time.time()
	
	rate = rospy.Rate(1)
	
	while not rospy.is_shutdown():
		current_time = time.time()
		
		# compute odometry
		dt = (current_time - last_time) / 1000	# time difference in seconds
		delta_x = (vx * cos(th) - vy * sin(th)) * dt)
		delta_y = (vx * sin(th) + vy * cos(th)) * dt)
		delta_th = vth * dt
		rate.sleep()
	
	
	
if __name__=='__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass