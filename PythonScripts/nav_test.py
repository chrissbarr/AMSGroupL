#!/usr/bin/env python

# """
# Simple demo script to move the mechbot through a short series of waypoints. Designed to test the navigation systems.
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
from OurModules import functions_common as com

# ROS libraries
import rospy
import std_msgs.msg
from geometry_msgs.msg import Point, Quaternion, PoseStamped, Pose
import tf

delay = 0.5 # update rate for main loop (s)

rospy.init_node("nav_test", anonymous=False) # name the script on the ROS network
		
def main(argv):
	print("nav test started...")
	nav.init()
	key_pressed = False
	
	waypoint_index = 0
	waypoint_list = [[3.5,0.8],[3.5,1.8],[2,0.8]]

	num_waypoints = len(waypoint_list)
	exit = False
	loop = 0

	print("Num waypoints: %d") % (num_waypoints)

	while key_pressed == False and exit == False:

		loop_start = time.time() # get loop time at start for loop rate calculations
		
		d_x = waypoint_list[waypoint_index][0]
		d_y = waypoint_list[waypoint_index][1]
		
		if(nav.nav_status == nav.NAV_STATUS_COORDS_REACHED and nav.target_x == d_x and nav.target_y == d_y):
			# if the navigation system has reached the coordinate
			print("Waypoint %d has been reached.") % waypoint_index

			if(waypoint_index < num_waypoints-1):
				if(loop>0):
					exit = True
				waypoint_index += 1
			else:
				waypoint_index = 0
				loop +=1
		else:
			#coordinate isn't set - set it
			print("move to waypoint %d") % (waypoint_index)
			nav.send_target_pose(d_x, d_y, -999)
			
		time.sleep(delay)
		key_pressed = select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []) # is key pressed?

if __name__ == '__main__': # main loop
	try: # if no problems
		main(sys.argv[1:])
        
 	except Exception,e: # if a problem
 		print ("Error!")
		print(e)
		pass
 