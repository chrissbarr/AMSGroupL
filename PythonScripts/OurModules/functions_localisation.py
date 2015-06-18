#!/usr/bin/env python

"""
Localisation-related functions.

"""

# system libraries
import time
import select
import sys
import socket
import os
import math
import pygame
from os.path import expanduser

import random
from random import randint

# ROS libraries
import rospy
import std_msgs.msg
from geometry_msgs.msg import Point, Quaternion, PoseStamped, Pose
from nav_msgs.msg import Odometry
import tf

current_x = current_y = current_th = -999
odom_x = odom_y = odom_th = -999
vicon_x = vicon_y = vicon_th = -999

vicon_last_update_time = 0

def odom_update(data):
	global odom_x, odom_y, odom_th

	(odom_x, odom_y, odom_th) = odom_parse(data)

def odom_parse(data):

	odom_x = data.pose.pose.position.x
	odom_y = data.pose.pose.position.y

	q = (
	    data.pose.pose.orientation.x,
	    data.pose.pose.orientation.y,
	    data.pose.pose.orientation.z,
	    data.pose.pose.orientation.w)

	# convert orientation from quaternion to euler angles, read yaw
	euler = tf.transformations.euler_from_quaternion(q)
	odom_th = euler[2]

	return (odom_x, odom_y, odom_th)

def vicon_pose_update(data):
	global vicon_x, vicon_y, vicon_th
	global current_x, current_y, current_th
	global vicon_last_update_time

	# read in position
	vicon_x = data.pose.position.x
	vicon_y = data.pose.position.y
	
	# read in orientation
	q = (
	    data.pose.orientation.x,
	    data.pose.orientation.y,
	    data.pose.orientation.z,
	    data.pose.orientation.w)
	
	# convert orientation from quaternion to euler angles, read yaw
	euler = tf.transformations.euler_from_quaternion(q)
	vicon_th = euler[2]

	current_x = vicon_x
	current_y = vicon_y
	current_th = vicon_th

	vicon_last_update_time = time.time()

def time_since_last_vicon_message():
	return time.time() - vicon_last_update_time

def fused_pose_update(data):
	global current_x, current_y, current_th

	# read in position
	current_x = data.pose.position.x
	current_y = data.pose.position.y
	
	# read in orientation
	q = (
	    data.pose.orientation.x,
	    data.pose.orientation.y,
	    data.pose.orientation.z,
	    data.pose.orientation.w)
	
	# convert orientation from quaternion to euler angles, read yaw
	euler = tf.transformations.euler_from_quaternion(q)
	current_th = euler[2]

def vicon_pose_initialise():
	PS = rospy.Subscriber("viconPose", PoseStamped, vicon_pose_update)

def odom_initialise():
	OD = rospy.Subscriber("odom", Odometry, odom_update)

def fused_pose_initialise():
	Pf = rospy.Subscriber("fusedPose", PoseStamped, fused_pose_update)

def init():
	vicon_pose_initialise()
	odom_initialise()
	fused_pose_initialise()

if __name__ == '__main__': # main loop
	try: # if no problems
		main(sys.argv[1:])
        
 	except Exception,e: # if a problem
		print(e)
		pass
 
