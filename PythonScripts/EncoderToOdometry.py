#!/usr/bin/python

# import libraries
import time

# ROS libraries
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from os.path import expanduser
import tf
import socket
import math

#import std_msgs.msg
from std_msgs.msg import *


# define settings
WheelDiameter = 100	# wheel diameter in mm
WheelBase = 220		# distance between wheels in mm
DistancePerCount = (3.14159 *  WheelDiameter) / (64) # mm / encoder tick

x = 0	# x coordinate in mm
y = 0	# y coordinate in mm
th = 0	# rotation in radians

vx = 0	# instantaneous x velocity
vy = 0	# instantaneous y velocity
vth = 0	# instantaneous angular velocity

_PreviousLeftEncoderCounts = 0
_PreviousRightEncoderCounts = 0
last_time_encoder = 0

delta_left = 0
delta_right = 0


def wheel_callback(data):

	
	current_time_encoder = time.time()

	left_encoder = data.data[2]
	right_encoder = data.data[3]

	global delta_left, delta_right, vx, by, vth, _PreviousLeftEncoderCounts, _PreviousRightEncoderCounts, last_time_encoder
	
	delta_left = left_encoder - _PreviousLeftEncoderCounts
	delta_right = right_encoder - _PreviousRightEncoderCounts
	
	vx = delta_left * DistancePerCount
	vy = delta_right * DistancePerCount
	
	vth = (left_encoder - right_encoder) / WheelBase

	print('Wheel callback! LE: %d RE: %d DL: %d DR: %d vx: %.3f vy: %.3f') % (left_encoder, right_encoder, delta_left, delta_right, vx, vy)
	
	_PreviousLeftEncoderCounts = left_encoder
	_PreviousRightEncoderCounts = right_encoder
	last_time_encoder = current_time_encoder
	
	return

def main():
	# setup publishing odometry messages
	odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
	rospy.init_node('odom_publisher',anonymous=True)
	odom_broadcaster = tf.TransformBroadcaster()
	
	frame_id = '/odom'
  	child_frame_id = '/base_footprint'
	
	# subscribe to wheel encoder messages
	encoder_ros_update = rospy.Subscriber("/mechbot_12/get/encoder_status", std_msgs.msg.Int32MultiArray, wheel_callback)
	
	current_time = time.time()
	last_time = time.time()
	
	rate = rospy.Rate(1)
	
	while not rospy.is_shutdown():
		current_time = time.time()

		global x, y, th
		
		# compute odometry
		dt = (current_time - last_time) / 1000	# time difference in seconds
		delta_x = (vx * math.cos(th) - vy * math.sin(th)) * dt
		delta_y = (vx * math.sin(th) + vy * math.cos(th)) * dt
		delta_th = vth * dt
		
		
		x += delta_x
		y += delta_y
		th += delta_th

		print('X: %d Y: %d Theta: %d') % (x, y, th)
		
		msg = Odometry()
		msg.header.stamp = rospy.Time.now()
      		msg.header.frame_id = '/odom'
		msg.child_frame_id = '/base_footprint'
		
		# create quaternion
		q = tf.transformations.quaternion_from_euler(0, 0, th)
		
		msg.pose.pose.position = Point(x, y, 0)
		msg.pose.pose.orientation.x = q[0]
		msg.pose.pose.orientation.y = q[1]
		msg.pose.pose.orientation.z = q[2]
		msg.pose.pose.orientation.w = q[3]

		#msg.twist.twist.linear.x = vx
		#msg.twist.twist.linear.y = vy
		#msg.twist.twist.angular.z = vth

		#odom_broadcaster.sendTransform((x, y, 0), q, rospy.Time.now(),'/base_footprint',"world")

		odom_pub.publish(msg)

		last_time = current_time
		rate.sleep()
	
if __name__=='__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
