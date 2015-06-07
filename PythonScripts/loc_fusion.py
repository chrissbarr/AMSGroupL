#!/usr/bin/python

"""
This script is intended to perform sensor fusion between the Vicon and platform odometry data.

It operates under the following principles:
	- When VICON is available and can be assumed to be reliable:
		- Return VICON for our current pose
		- Update the Odometry to be in sync with the current VICON pose
	- If VICON is unavailable / unreliable:
		- Return the Odometry pose as the current pose

"""

# import libraries
import time

# ROS libraries
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, PoseStamped
from os.path import expanduser
import tf
import socket
import math

from std_msgs.msg import *

from OurModules import functions_localisation as loc
from OurModules import functions_common as func

# setup publishing pose messages
pose_pub = rospy.Publisher('fusedPose', PoseStamped, queue_size=10)

odom_offset_x = odom_offset_y = odom_offset_th = 0	# variables to track the offset between VICON localisation and raw odometry localisation

vicon_x_prev = vicon_y_prev = -999
vicon_th_prev = 0

vicon_xy_jump_threshold = 0.2 # maximum distance allowed between successive Vicon Pose messages for data to be considered valid (m)
vicon_th_jump_threshold = math.pi / 2 # maximum rotation allower between successive Vicon Pose messages for data to be considered valid (rad)

def publish_current_pose(x, y, th):
	#create pose message
	msg = PoseStamped()

	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = 'world'

	q = tf.transformations.quaternion_from_euler(0, 0, th)

	msg.pose.position = Point(x,y,0)

	msg.pose.orientation.x = q[0]
	msg.pose.orientation.y = q[1]
	msg.pose.orientation.z = q[2]
	msg.pose.orientation.w = q[3]

	pose_pub.publish(msg)

def vicon_is_reliable(vicon_x, vicon_y, vicon_th):
	global vicon_x_prev, vicon_y_prev, vicon_th_prev
	vicon_is_good = True

	# first, check that the VICON is initialised...
	if(vicon_x == -999 and vicon_y == -999):
		vicon_is_good = False

	# make sure this isn't the first loop with Vicon data...
	if(vicon_x_prev != -999 and vicon_y_prev != -999):
		# next, check that the value hasn't jumped unreasonably far from the last vicon message
		if((math.fabs(vicon_x - vicon_x_prev) > vicon_xy_jump_threshold) or (math.fabs(vicon_y - vicon_y_prev) > vicon_xy_jump_threshold) or (func.angular_difference(vicon_th, vicon_th_prev) > vicon_th_jump_threshold):
			vicon_is_good = False

	vicon_x_prev = vicon_x
	vicon_y_prev = vicon_y
	vicon_th_prev = vicon_y

	return vicon_is_good

def localisation_fusion():
	global odom_offset_x, odom_offset_y, odom_offset_th
	# let's make our variables easier to address
	vicon_x = loc.vicon_x
	vicon_y = loc.vicon_y
	vicon_th = loc.vicon_th

	odom_x = loc.odom_x
	odom_y = loc.odom_y
	odom_th = loc.odom_th

	fused_x = fused_y = fused_th = -999

	#now let's do stuff with them
	if(vicon_is_reliable(vicon_x,vicon_y,vicon_th) == True):
		# VICON is working, so let's just use our VICON pose as our current pose
		fused_x = vicon_x
		fused_y = vicon_y
		fused_th = vicon_th

		# And we'll update the odometry offsets in case VICON stops working soon
		odom_offset_x = vicon_x - odom_x
		odom_offset_y = vicon_y - odom_y
		odom_offset_th = vicon_th - odom_th
	else:
		# VICON isn't reliable or working, so we have to use our odometry data instead...
		fused_x = odom_x + odom_offset_x
		fused_y = odom_y + odom_offset_y
		fused_th = odom_th + odom_offset_th

	return(fused_x, fused_y, fused_th)



def main():

	loc.init() # subscrive to VICON and Odometry messages
	
	rate = rospy.Rate(10)
	
	while not rospy.is_shutdown():

		(x, y, th) = localisation_fusion()

		publish_current_pose(x, y, th)

		rate.sleep()
	
if __name__=='__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
