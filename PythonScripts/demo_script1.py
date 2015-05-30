#!/usr/bin/env python

"""
This program is the first demo for Group 12's mechbot.

It is intended to demonstrate basic search-and-rescue capabilities in a limited fashion.

This script will cause the robot to:
	-Move through a 2D grid of waypoints within a given area
		-Take a temperature reading at 4 equidistant headings
			-If the temperature reading suggests a person is present, an alert will sound
		-Move on to the next grid point, repeat the process
		-Avoid obstacles encountered during this process

This script will demonstrate that the following capabilities are present within the system:
	- Localisation
		- Vicon (Primary localisation source for demonstration)
		- Platform Odometry (Used to demonstrate function continues in absence of Vicon data)
	- Navigation
		- Appropriate Motor Control (PID controller)
		- Point to point navigation (Fuzzy Proportional Control)
	- Object Avoidance
		- Objects blocking the robot's progress will be avoided
	- Temperature Measurement
	

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
import numpy as np

# ROS libraries
import rospy
import std_msgs.msg
from geometry_msgs.msg import Point, Quaternion, PoseStamped, Pose
import tf

# SOUNDS
sound_folder = '{0}/Documents/Sounds/PortalTurret/'.format(expanduser('~')) # folder that contains the sound file
sound_group_startup = ['turret_deploy_2.ogg','turret_deploy_4.ogg']
sound_group_shutdown = ['turret_disabled_4.ogg','turret_retire_1.ogg','turret_retire_2.ogg','turret_retire_4.ogg','turret_retire_5.ogg','turret_retire_6.ogg','turret_retire_7.ogg']
sound_group_error = ['turret_disabled_2.ogg','turret_tipped_2.ogg','turret_tipped_3.ogg','turret_tipped_4.ogg']
sound_group_search = ['turret_search_4.ogg','turret_autosearch_2.ogg','turret_autosearch_3.ogg']
sound_group_found = ['turret_active_6.ogg','turret_active_7.ogg','turret_active_8.ogg','sp_sabotage_factory_good_prerange01.ogg']
sound_group_pickup = ['turret_pickup_3.ogg','turret_pickup_8.ogg','turret_pickup_7.ogg','turret_pickup_10.ogg','turretlaunched03.ogg','turretlightbridgeblock03.ogg']
sound_group_ping = ['ping.ogg']

# IMU
acc_x = 0
acc_y = 0
acc_z = 0

# PERSONALITY CORE SETTINGS
talkativity = 1.0

pickup_threshold = 0.1	# z-accel value needed to trigger 'pick-up' event
pickup_comment_time_threshold = 10	# minimum time between 'pick-up' remarks
last_pickup_comment_time = 0	# tracks time last comment was made
last_played_sound = 'none'

# NAVIGATION STATUS MESSAGES
nav_coords_reached = "COORDS REACHED"
nav_move_in_progress = "MOVING"
current_nav_string = ""

# DESIRED NAVIGATION COORDINATES
current_d_x = 0		#from desired pose topic
current_d_y = 0
current_d_th = 0

# MOTOR SPEED VARIABLES
# Only used to check current motor speed
motor_direction = 0
motor_speed_left = 0
motor_speed_right = 0

delay = 0.1 # update rate for main loop (s)

rospy.init_node("demo_script_controller", anonymous=False) # name the script on the ROS network

# setup publishing pose messages
pose_pub = rospy.Publisher('desiredPose', Pose, queue_size=10)
odom_broadcaster = tf.TransformBroadcaster()

def generate_search_grid(start_x, start_y, x_cells, y_cells, cell_size, headings_per_point):
	"""
	Generates an array representing a search grid. 
	The array will contain each point 'headings_per_point' times, each time with a different, equiangular heading.

	start_x = x coordinate of corner of search grid (in m)
	start_y = y coordinate of corner of search grid (in m)
	x_cells = number of search points in the x axis
	y_cells = number of search points in the y axis
	cell_size = size of each 'cell' - the distance between grid points (in m)
	headings_per_point = number of directions to face at each point. (i.e. 4 rotates in 90 degree intervals)

	Movement sweeps in an optimised pattern, x-direction alternating as shown below:
	oooooo
	     v
	oooooo
	v
	oooooo
		 v
		 
	This is preferable to the default iterated-loop pattern, which would result in unneeded movement:
	oooooo
	v<<<<<
	oooooo
	>>>>>v
	oooooo
	v<<<<<
	"""
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
		
	return search_grid_array, x_cells * y_cells

def msg_subscriber():
	# subscribe to ROS data updates
	#CP = rospy.Subscriber("currentPose", PoseStamped, current_pose_update)
	DP = rospy.Subscriber("desiredPose", Pose, desired_pose_update)
	NS = rospy.Subscriber("nav_status", std_msgs.msg.String, nav_status_update)
	IMU = rospy.Subscriber("/mechbot_12/get/IMU_status", std_msgs.msg.Float32MultiArray,imu_status_update)
	MD = rospy.Subscriber("/mechbot_12/set/motor_drive", std_msgs.msg.UInt8MultiArray, motor_drive_update)
	return (DP, NS)
	
def desired_pose_update(data):
	global current_d_x, current_d_y, current_d_th

	# read in position
	current_d_x = data.position.x
	current_d_y = data.position.y
	
	# read in orientation
	q = (
	    data.orientation.x,
	    data.orientation.y,
	    data.orientation.z,
	    data.orientation.w)
	    
	# convert orientation from quaternion to euler angles, read yaw
	euler = tf.transformations.euler_from_quaternion(q)
	current_d_th = euler[2]
	
def nav_status_update(message):
	global current_nav_string
	current_nav_string = message
	
def imu_status_update(data):
	global acc_x, acc_y, acc_z
	acc_x = data.data[0]
	acc_y = data.data[1]
	acc_z = data.data[2]
	
def motor_drive_update(data):
	motor_direction = data.data[0]
	motor_speed_left = data.data[1]
	motor_speed_right = data.data[2]

def send_desired_pose(x, y, th):
    print("Requesting move to point: x: %.3f  y: %.3f  Th: %.1f") % (x, y, th) # print 2D pose data to terminal
    #publish pose data to ros topic
    msg = Pose()

    q = tf.transformations.quaternion_from_euler(0, 0, th)

    msg.position = Point(x,y,0)

    msg.orientation.x = q[0]
    msg.orientation.y = q[1]
    msg.orientation.z = q[2]
    msg.orientation.w = q[3]

    pose_pub.publish(msg)

def loop_timing(delay,loop_start):
	loop_sleep = delay - (time.time() - loop_start) # if loop delay too low then will print data faster than updates are recieved
	if loop_sleep > 0:
		time.sleep(loop_sleep)

def sound_init():
	pygame.mixer.pre_init(44100, -16, 2, 4096) # set sound buffer. Prevents popping when playing sound files
	pygame.init()
	pygame.mixer.init()
		
def play_sound(sound_file): # subroutine to play the specified sound file
	global sound_folder
	if pygame.mixer.music.get_busy():
		pygame.mixer.fadeout(50) # fade out the current sound if is was one
	pygame.mixer.music.load(sound_folder + sound_file)
	pygame.mixer.music.play()
    
def play_sound_group(sound_group, probability):
	global last_played_sound
	play_sound_probability = randint(0,100)
	if(play_sound_probability <= probability):
		selected_sound_index = randint(0,len(sound_group)-1)

		# make sure we don't play the same clip twice in a row
		while(sound_group[selected_sound_index] == last_played_sound):
			selected_sound_index = randint(0,len(sound_group)-1)

		last_played_sound = sound_group[selected_sound_index]
		play_sound(sound_group[selected_sound_index])
		
def measure_temperature():
	play_sound_group(sound_group_ping,100)
	#to do
	
	temperature = randint(10,40)
	if(25 < temperature and temperature < 35):
		print("Person detected! (maybe...)")
		play_sound_group(sound_group_found,100)
		time.sleep(5)
		
def check_motors_stopped():
	if(motor_speed_left == 0 and motor_speed_right == 0):
		return True
	else:
		return False
		
def react_to_pickup():
	global last_pickup_comment_time, pickup_threshold, pickup_comment_time_threshold
	# detect if picked up from IMU data - z liftoff means picked up any time, otherwise if motors are stopped but platform experiences any acceleration
	if(math.fabs(acc_z) > pickup_threshold or ((math.fabs(acc_x) > pickup_threshold or math.fabs(acc_y) > pickup_threshold) and check_motors_stopped() == True)):
		print("Put me down!")
		# robot is being handled / carried. Respond if appropriate:
		if(time.time() - last_pickup_comment_time > pickup_comment_time_threshold):
			# enough time has passed to make another remark...
			play_sound_group(sound_group_pickup,90)
			last_pickup_comment_time = time.time()

def personality_core_init():
	random.seed()
	sound_init()
	play_sound_group(sound_group_startup,100)

def personality_core_update():
	react_to_pickup()
	
def personality_core_shutdown():
	play_sound_group(sound_group_shutdown,100)
	print ("Shutting down...")

	while (pygame.mixer.music.get_busy()):	# wait for sound to stop playing
		time.sleep(.1)

	pygame.mixer.quit()
	

def main(argv):
	key_pressed = False
	
	#subscribe to the relevant ROS topics
	(desired_pose_update, nav_status_update) = msg_subscriber()
	
	personality_core_init()
	
	# First, generate the search grid
	grid, num_waypoints = generate_search_grid(1,2,5,4,0.25,4)
	waypoint_index = 0
	grid_finished = False
	print(grid)
	
	
	
	
	while key_pressed == False:
		loop_start = time.time() # get loop time at start for loop rate calculations
		
		d_x = grid[waypoint_index][0]
		d_y = grid[waypoint_index][1]
		d_th = grid[waypoint_index][2]
		
		if(current_d_x == d_x and current_d_y == d_y and current_d_th == d_th):
			# if the navigation system has reached the coordinate
			if(current_nav_string == nav_coords_reached):
				print("Waypoint %d has been reached.") % waypoint_index
				measure_temperature()
				if(waypoint_index < num_waypoints):
					waypoint_index += 1
					play_sound_group(sound_group_search,5*talkativity)
				else:
					grid_finished = True
		else:
			#coordinate isn't set - set it
			send_desired_pose(d_x, d_y, d_th)
			
		personality_core_update()
		loop_timing(delay,loop_start)
		
		key_pressed = select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []) # is key pressed?
	
	personality_core_shutdown()
	

if __name__ == '__main__': # main loop
	try: # if no problems
		main(sys.argv[1:])
        
	#except rospy.ROSInterruptException: # if a problem
	#	play_sound_group(sound_group_error,100)
	#	time.sleep(2)
 	#	pass
 	except: # if a problem
		play_sound_group(sound_group_error,100)
		print ("Error!")
		while (pygame.mixer.music.get_busy()):	# wait for sound to stop playing
			time.sleep(.1)
		pass
 