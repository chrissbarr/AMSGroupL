#!/usr/bin/env python

"""
This program drives the robot to a given set of coordinates.

It subscribes to the following topics:

- "currentPose", expects PoseStamped messages containing the current pose of the robot.
- "desiredPose", expects Pose messages containing the desired pose for the robot.

"""

# system libraries
import time
import select
import sys
import socket
import os
import math

import numpy as np

from OurModules import functions_common as cf
from OurModules import functions_ros_interfaces as ri
from OurModules import functions_motor_control as motor

# ROS libraries
import rospy
import std_msgs.msg
from geometry_msgs.msg import Point, Quaternion, PoseStamped, Pose
import tf

distance_threshold = 0.1 # units are in metres, reached target if x & y within 0.1 = 10cm of target position
rot_threshold = 0.2	# angle in radians, consider heading correct if within this number of radians to target point
travel_heading_error_window = 0.5 # If angle to target > this during travel, robot will stop and reorient
base_speed = 100 # Default speed robot travels at. Left and right motors are biased from this value to adjust steering.

#rotation settings
rot_P = 20.0
rot_I = 0.0
rot_D = 0.0
rot_error_sum = 0
rot_speed_offset = 5

#driving angle PID
driving_P = 25.0
driving_I = 0.0
driving_D = 0.0
driving_error_sum = 0

delay = 0.1 # update rate for main loop (s)

rospy.init_node("motor_pilot", anonymous=False) # name the script on the ROS network
time.sleep(0.2) # make sure publisher setup

# checks if the target coordinates are reached. Returns true if current x/y are near target x/y within set threshold
def coordinates_reached(dist_threshold):
	reached = False
	if(cf.distance_between_points(ri.current_x,ri.current_y,ri.target_x,ri.target_y) < dist_threshold):
		reached = True
	return reached
	
def turn_to_face(heading_error):
	global rot_P, rot_speed_offset
	#rotate to face heading
	# start motors moving based on angular difference
	motor_speed = int(round(rot_P * math.fabs(heading_error))) + rot_speed_offset
	#rot_error_sum += heading_error

	if(heading_error => 0):
		rot = 3
	if(heading_error < 0):
		rot = 2
						
	motor.publish_command(rot,motor_speed,motor_speed)

def main(argv):
	global driving_P, driving_I, driving_error_sum
	
	ri.init()

	moving = False	# tracks if we are currently moving towards the target point
	
	key_pressed = False
	
	justStarted = True # need to track if we've just started, otherwise we can get stuck on the first waypoint.

	while key_pressed == False:
		loop_start = time.time() # get loop time at start for loop rate calculations
		
		# calculate angular difference
		target_heading = cf.angle_between_points(ri.current_x,ri.current_y,ri.target_x,ri.target_y)
		heading_error = cf.angular_difference(target_heading,ri.current_th)
		
		if(ri.target_x != -999 and ri.target_y != -999):
			print("Current Position: %.3f %.3f %.3f | Target Position: %.3f %.3f %.3f | Heading Error: %.3f") % (ri.current_x,ri.current_y,ri.current_th,ri.target_x,ri.target_y,ri.target_th,heading_error)
					
			if(coordinates_reached(distance_threshold) == False):
				if(moving == False):
					if(math.fabs(heading_error) < rot_threshold):
						print("Heading achieved! Beginning move towards target!")
						# we're facing the right way, so stop and drive straight!
						# first, stop motors
						motor.publish_command(1,0,0)
						time.sleep(0.5) # make sure message has time to be enacted
						
						# drive straight
						motor.publish_command(0,base_speed,base_speed)
						moving = True
						
						rot_error_sum = 0 #reset PID integrator
						
						time.sleep(0.5) # make sure message has time to be enacted
					else:
						print("Orienting towards target...")
						turn_to_face(heading_error)
				else:
					#if we are moving but coordinates aren't reached...
					if(heading_error > travel_heading_error_window):
						# if we're off course by too much, stop and re-orientate towards target
						motor.publish_command(0,0,0)
						moving = False
						# loop should take over and make things work now we're stopped away from the target
					else:
						#adjust motors to aim towards target point
						motor_left_speed = base_speed - (heading_error * driving_P + driving_error_sum * driving_I)
						motor_left_right = base_speed + (heading_error * driving_P + driving_error_sum * driving_I)
						driving_error_sum += heading_error
						motor.publish_command(0,motor_left_speed,motor_left_right)
			else:
				print("Coordinates reached!")
				#if we've just reached the point, stop!
				if(moving == True or justStarted == True):
					justStarted = False
					motor.publish_command(0,0,0)
					moving = False
					driving_error_sum = 0
					print("Motors stopped!")
					
				# now, turn to face the desired heading (if there is one)	
				heading_offset = angular_difference(d_th,th)
				
				if(d_th != -999 and math.fabs(heading_offset) > rot_threshold):	#-999 means orientation doesn't matter, otherwise turn
					print("Matching desired orientation...")
					turn_to_face(heading_offset)
				else:
					#target position and orientation is reached!
					#stop the motors
					motor.publish_command(0,0,0)
					#and let all other scripts know
					#status_pub.publish(nav_coords_reached)
					
		loop_sleep = delay - (time.time() - loop_start) # if loop delay too low then will print data faster than updates are recieved
		
		if loop_sleep > 0:
			time.sleep(loop_sleep)
		
		key_pressed = select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []) # is key pressed?
	
	# finally, stop motors
	time.sleep(0.1) # make sure message has time to be enacted
	motor.publish_command(1,0,0)
	time.sleep(0.5) # make sure message has time to be enacted
	
	print ("Exit script, motors stopped")

if __name__ == '__main__': # main loop
	try: # if no problems
		main(sys.argv[1:])
        
	except rospy.ROSInterruptException: # if a problem
 		pass




