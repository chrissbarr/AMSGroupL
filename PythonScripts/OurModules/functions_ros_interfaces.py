#!/usr/bin/env python

"""
This script is a compilation of functions. 

These functions were originally included in several of the other scripts, but it became easier to consolidate them all in one place to avoid code duplication.

Most of these functions relate to "personality" elements of the software - sound playback and such. 
Supporting functions related to or used with these functions are probably also here, because I'm lazy.

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
import tf

current_x = current_y = current_th = target_x = target_y = target_th = -999

def current_pose_update(data):
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

def target_pose_update(data):
	global target_x, target_y, target_th

	# read in position
	target_x = data.position.x
	target_y = data.position.y
	
	# read in orientation
	q = (
	    data.orientation.x,
	    data.orientation.y,
	    data.orientation.z,
	    data.orientation.w)
	    
	# convert orientation from quaternion to euler angles, read yaw
	euler = tf.transformations.euler_from_quaternion(q)
	target_th = euler[2]

def current_pose_initialise():
	PS = rospy.Subscriber("currentPose", PoseStamped, current_pose_update)

def target_pose_initialise():
	DP = rospy.Subscriber("targetPose", Pose, target_pose_update)

def init():
	current_pose_initialise()
	target_pose_initialise()

def main(argv):
	PS = rospy.Subscriber("currentPose", PoseStamped, current_pose_update)
	DP = rospy.Subscriber("targetPose", Pose, target_pose_update)
	
	

if __name__ == '__main__': # main loop
	try: # if no problems
		main(sys.argv[1:])
        
 	except Exception,e: # if a problem
		print(e)
		pass
 
