#!/usr/bin/env python

"""
This script is a compilation of functions. 

These functions were originally included in several of the other scripts, but it became easier to consolidate them all in one place to avoid code duplication.

Most of these functions relate to "personality" elements of the software - sound playback and such. 
Supporting functions related to or used with these functions are probably also here, because I'm lazy.

"""

# system libraries
import select
import sys
import socket
import os
import math


#calculates difference between two angles
def angular_difference(angle1, angle2):
	# find the raw angular difference
	diff = angle1 - angle2
	
	#make sure it's the shortest distance around 0 etc
	diff = (diff + math.pi) % (2 * math.pi) - math.pi
		
	return diff	

def angle_between_points(x1, y1, x2, y2):
	dx = x1 - x2
	dy = y1 - y2
	
	heading = math.atan2(-dy,-dx)

	return heading
	
def distance_between_points(x1, y1, x2, y2):
	return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def quaternion_to_euler(data):
	# read in orientation
	q = (
	    data.orientation.x,
	    data.orientation.y,
	    data.orientation.z,
	    data.orientation.w)
	    
	# convert orientation from quaternion to euler angles, read yaw
	euler = tf.transformations.euler_from_quaternion(q)
	
	return euler[2]