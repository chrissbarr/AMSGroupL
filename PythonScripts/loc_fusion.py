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
import select

from std_msgs.msg import *

from OurModules import functions_localisation as loc
from OurModules import functions_common as func

import numpy as np


# setup publishing pose messages
pose_pub = rospy.Publisher('fusedPose', PoseStamped, queue_size=10)
odom_offset_pub = rospy.Publisher('odomOffset', PoseStamped, queue_size=10)
rospy.init_node("localisationFuser", anonymous=False) # name the script on the ROS network

odom_offset_x = odom_offset_y = odom_offset_th = 0	# variables to track the offset between VICON localisation and raw odometry localisation

vicon_x_prev = vicon_y_prev = vicon_x_prev2 = vicon_y_prev2 = -999
vicon_th_prev = 0

vicon_x_prev_list = [0,0,0,0,0]
vicon_y_prev_list = [0,0,0,0,0]

vicon_reliability_history = [False] * 20

vicon_only = False
odom_offset_calcd = False

vicon_noise_floor = 0.1

vicon_xy_jump_threshold = 0.5 # maximum distance allowed between successive Vicon Pose messages for data to be considered valid (m)
vicon_th_jump_threshold = 10 * math.pi / 2 # maximum rotation allower between successive Vicon Pose messages for data to be considered valid (rad)

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

def publish_offset_odom():
	global odom_offset_x, odom_offset_y, odom_offset_th, odom_offset_calcd


	if(odom_offset_calcd == True):
		odom_x = loc.odom_x + odom_offset_x
		odom_y = loc.odom_y + odom_offset_y
		odom_th = loc.odom_th + odom_offset_th

		#create pose message
		msg = PoseStamped()

		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = 'world'

		q = tf.transformations.quaternion_from_euler(0, 0, odom_th)

		msg.pose.position = Point(odom_x,odom_y,0)

		msg.pose.orientation.x = q[0]
		msg.pose.orientation.y = q[1]
		msg.pose.orientation.z = q[2]
		msg.pose.orientation.w = q[3]

		odom_offset_pub.publish(msg)

def vicon_is_reliable(vicon_x, vicon_y, vicon_th):
	#global vicon_x_prev, vicon_y_prev, vicon_th_prev, vicon_x_prev2, vicon_y_prev2
	global vicon_x_prev_list, vicon_y_prev_list
	vicon_is_good = True

	# first, check that the VICON is initialised...
	if(vicon_x == -999 and vicon_y == -999):
		vicon_is_good = False
		print("Vicon not initialised!")

	# make sure this isn't the first loop with Vicon data...
	elif(vicon_x_prev_list[0] != -999 and vicon_y_prev_list[0] != -999):

		# next, check that the value hasn't jumped unreasonably far from the last vicon message
		if(math.fabs(func.distance_between_points(vicon_x,vicon_y,np.mean(vicon_x_prev_list),np.mean(vicon_y_prev_list))) > vicon_xy_jump_threshold): #or (math.fabs(func.angular_difference(vicon_th, vicon_th_prev)) > vicon_th_jump_threshold)):
			vicon_is_good = False
			print("Large Vicon jump! From (%.3f %.3f %.3f to %.3f %.3f %.3f)") % (np.mean(vicon_x_prev_list), np.mean(vicon_y_prev_list), vicon_th_prev, vicon_x, vicon_y, vicon_th)

		# also check if the values are identical (Vicon is likely frozen then)
		if(np.max(vicon_x_prev_list) == np.min(vicon_x_prev_list) and np.max(vicon_y_prev_list) == np.min(vicon_y_prev_list)):
			vicon_is_good = False
			print("Vicon data stalled!")

		if(loc.time_since_last_vicon_message() > 0.2):
			vicon_is_good = False
			print("No Vicon updates!")


	for i in range(len(vicon_x_prev_list)-1, 0, -1):
		vicon_x_prev_list[i] = vicon_x_prev_list[i-1]
		vicon_y_prev_list[i] = vicon_y_prev_list[i-1]

	vicon_x_prev_list[0] = vicon_x
	vicon_y_prev_list[0] = vicon_y

	#print(vicon_x_prev_list)

	#vicon_x_prev2 = vicon_x_prev
	#vicon_y_prev2 = vicon_y_prev

	#vicon_x_prev = vicon_x
	#vicon_y_prev = vicon_y
	#vicon_th_prev = vicon_y

	return vicon_is_good

def localisation_fusion():
	global odom_offset_x, odom_offset_y, odom_offset_th, odom_offset_calcd
	global vicon_reliability_history, vicon_only
	# let's make our variables easier to address
	vicon_x = loc.vicon_x
	vicon_y = loc.vicon_y
	vicon_th = loc.vicon_th

	odom_x = loc.odom_x
	odom_y = loc.odom_y
	odom_th = loc.odom_th

	fused_x = fused_y = fused_th = -999

	reliable = vicon_is_reliable(vicon_x,vicon_y,vicon_th)

	#now let's do stuff with them
	if(reliable == True or vicon_only == True):
		# VICON is working, so let's just use our VICON pose as our current pose
		fused_x = vicon_x
		fused_y = vicon_y
		fused_th = vicon_th

		if(vicon_reliability_history.count(True) == len(vicon_reliability_history) and odom_offset_calcd == False):
			# And we'll update the odometry offsets in case VICON stops working soon
			odom_offset_x = vicon_x - odom_x
			odom_offset_y = vicon_y - odom_y
			odom_offset_th = vicon_th - odom_th
			print("Syncing odom to Vicon. New odom = (%.3f, %.3f, %.3f)") % (odom_x + odom_offset_x, odom_y + odom_offset_y, odom_th + odom_offset_th)
			odom_offset_calcd = True

		#print("Vicon is reliable, using Vicon data...")
	else:
		# VICON isn't reliable or working, so we have to use our odometry data instead...
		fused_x = odom_x + odom_offset_x
		fused_y = odom_y + odom_offset_y
		fused_th = odom_th + odom_offset_th
		#print("Substituting odometry data: %.3f %.3f %.3f") % (fused_x, fused_y, fused_th)


	for i in range(len(vicon_reliability_history)-1, 0, -1):
		vicon_reliability_history[i] = vicon_reliability_history[i-1]

	vicon_reliability_history[0] = reliable 

	#print(vicon_reliability_history)


	return(fused_x, fused_y, fused_th)

def main(argv):
	global vicon_only

	loc.init() # subscrive to VICON and Odometry messages

	if(sys.argv[1] == 'T'):
		vicon_only = True
		print("Vicon Data Only")
	else:
		vicon_only = False
		print("Odometry data will be used too")
	
	rate = rospy.Rate(10)

	key_pressed = False
	
	while key_pressed == False:

		(x, y, th) = localisation_fusion()

		publish_current_pose(x, y, th)

		publish_offset_odom()
		key_pressed = select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []) # is key pressed?
		rate.sleep()
	
if __name__=='__main__':
	try:
		main(sys.argv[1:])
	except rospy.ROSInterruptException:
		pass
