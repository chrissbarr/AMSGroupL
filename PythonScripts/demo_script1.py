#!/usr/bin/env python

# """
# This program is the first demo for Group 12's mechbot.

# It is intended to demonstrate basic search-and-rescue capabilities in a limited fashion.

# This script will cause the robot to:
# 	-Move through a 2D grid of waypoints within a given area
# 		-Take a temperature reading at 4 equidistant headings
# 			-If the temperature reading suggests a person is present, an alert will sound
# 		-Move on to the next grid point, repeat the process
# 		-Avoid obstacles encountered during this process

# This script will demonstrate that the following capabilities are present within the system:
# 	- Localisation
# 		- Vicon (Primary localisation source for demonstration)
# 		- Platform Odometry (Used to demonstrate function continues in absence of Vicon data)
# 	- Navigation
# 		- Appropriate Motor Control (PID controller)
# 		- Point to point navigation (Fuzzy Proportional Control)
# 	- Object Avoidance
# 		- Objects blocking the robot's progress will be avoided
# 	- Temperature Measurement
	

# """

# system libraries
import time
import select
import sys
import socket
import os
import math
from os.path import expanduser

import random
from random import randint
import numpy as np

from OurModules import functions_nav_control as nav
from OurModules import functions_personality as pf
from OurModules import functions_common as com

# ROS libraries
import rospy
import std_msgs.msg
from geometry_msgs.msg import Point, Quaternion, PoseStamped, Pose
import tf

delay = 0.5 # update rate for main loop (s)

rospy.init_node("demo_script_controller", anonymous=False) # name the script on the ROS network

def generate_search_grid(start_x, start_y, x_cells, y_cells, cell_size, headings_per_point):
	# '''
	# Generates an array representing a search grid. 
	# The array will contain each point 'headings_per_point' times, each time with a different, equiangular heading.

	# start_x = x coordinate of corner of search grid (in m)
	# start_y = y coordinate of corner of search grid (in m)
	# x_cells = number of search points in the x axis
	# y_cells = number of search points in the y axis
	# cell_size = size of each 'cell' - the distance between grid points (in m)
	# headings_per_point = number of directions to face at each point. (i.e. 4 rotates in 90 degree intervals)

	# Movement sweeps in an optimised pattern, x-direction alternating as shown below:
	# oooooo
	#      v
	# oooooo
	# v
	# oooooo
	# 	 v
		 
	# This is preferable to the default iterated-loop pattern, which would result in unneeded movement:
	# oooooo
	# v<<<<<
	# oooooo
	# >>>>>v
	# oooooo
	# v<<<<<
	# '''
	search_grid_array=[]
	
	x_start = 0		# first x_cell index
	x_end = x_cells	# last x_cell index. We keep track so we can swap the x-sweep direction between rows.
	x_dir = int(math.copysign(1,x_end-x_start))
	
	for y in xrange(0, y_cells, 1):
		for x in xrange(x_start, x_end, x_dir):
			for th in xrange(0,headings_per_point):
				search_grid_array.append([start_x + (x * cell_size), start_y + (y * cell_size), th * math.pi/(headings_per_point/2)])
		
		x_start, x_end = x_end-x_dir, x_start-x_dir		# swap the start and end points so our path is more efficient
		x_dir = int(math.copysign(1,x_end-x_start))
		
	return search_grid_array, len(search_grid_array)

def loop_timing(delay,loop_start):
	loop_sleep = delay - (time.time() - loop_start) # if loop delay too low then will print data faster than updates are recieved
	if loop_sleep > 0:
		time.sleep(loop_sleep)
		
def measure_temperature():
	pf.play_sound_group(pf.sound_group_ping,100)
	#to do
	
	temperature = randint(0,80)
	time.sleep(.5)
	if(28 < temperature and temperature < 38):
		print("Person detected! (maybe...)")
		pf.play_sound_group(pf.sound_group_found,100)
		time.sleep(2)
		
def main(argv):
	print("main start")
	key_pressed = False
	time.sleep(1)
	pf.sound_init()
	nav.init()
	pf.sound_init()
	time.sleep(1)
	print("modules initialised")
	
	# First, generate the search grid
	grid, num_waypoints = generate_search_grid(3,0.8,4,3,0.75,4)
	print("grid generated)")
	waypoint_index = 0
	grid_finished = False
	print(grid)

	print("Num waypoints: %d") % (num_waypoints)
	pf.play_sound_group(pf.sound_group_search,100)
	

	while key_pressed == False:
		("while loop")
		loop_start = time.time() # get loop time at start for loop rate calculations
		
		d_x = grid[waypoint_index][0]
		d_y = grid[waypoint_index][1]
		d_th = grid[waypoint_index][2]
		
		if(nav.nav_status == nav.NAV_STATUS_COORDS_REACHED and nav.target_x == d_x and nav.target_y == d_y and com.angular_difference(nav.target_th, d_th)==0):
			# if the navigation system has reached the coordinate
			print("Waypoint %d has been reached.") % waypoint_index
			measure_temperature()
			if(waypoint_index < num_waypoints-1):
				waypoint_index += 1
				pf.play_sound_group(pf.sound_group_search,5)
			else:
				waypoint_index = 0
				#grid_finished = True
		else:
			#coordinate isn't set - set it
			print("move to waypoint %d") % (waypoint_index)
			nav.send_target_pose(d_x, d_y, d_th)
			
		loop_timing(delay,loop_start)
		key_pressed = select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []) # is key pressed?

if __name__ == '__main__': # main loop
	try: # if no problems
		main(sys.argv[1:])
        
 	except Exception,e: # if a problem
 		print ("Error!")
		print(e)
		pf.play_sound_group(pf.sound_group_error,100)
		pf.block_wait_sound_finish()
		pass
 