#!/usr/bin/env python

"""
This program runs in the background, and is intended to add more personality to the mechbot.

It is designed to audibly react to certain events, including:
	- Being picked up / lifted
	- Being forcibly pushed along the ground
	- Being left alone for extended periods of time

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

# My Modules
from OurModules import functions_personality as pf
from OurModules import functions_servo_control as servo

# SOUNDS
sound_folder = '{0}/Documents/Sounds/PortalTurret/'.format(expanduser('~')) # folder that contains the sound file
sound_group_startup = ['turret_deploy_2.ogg','turret_deploy_4.ogg']
sound_group_shutdown = ['turret_disabled_4.ogg','turret_retire_1.ogg','turret_retire_2.ogg','turret_retire_4.ogg','turret_retire_5.ogg','turret_retire_6.ogg','turret_retire_7.ogg']
sound_group_error = ['turret_disabled_2.ogg','turret_tipped_2.ogg','turret_tipped_3.ogg','turret_tipped_4.ogg']
sound_group_search = ['turret_search_4.ogg','turret_autosearch_2.ogg','turret_autosearch_3.ogg']
sound_group_found = ['turret_active_6.ogg','turret_active_7.ogg','turret_active_8.ogg','sp_sabotage_factory_good_prerange01.ogg']
sound_group_pickup = ['turret_pickup_3.ogg','turret_pickup_8.ogg','turret_pickup_7.ogg','turret_pickup_10.ogg','turretlaunched03.ogg','turretlightbridgeblock03.ogg']
sound_group_push = ['turretsquashed04.ogg','turretsquashed06.ogg','turretshotbylaser07.ogg']
sound_group_ping = ['ping.ogg']
sound_group_lonely = ['turret_search_1.ogg','turret_autosearch_5.ogg']
sound_group_hello = ['sp_sabotage_factory_good_pass01.ogg','sp_sabotage_factory_template01.ogg']

# IMU
acc_x = -999
acc_y = -999
acc_z = -999

# Encoder
encoder_left = 0
encoder_right = 0

# PERSONALITY CORE SETTINGS
talkativity = 1.0
pickup_threshold = .3	# z-accel value needed to trigger 'pick-up' event
pickup_comment_time_threshold = 8	# minimum time between 'pick-up' remarks
pushed_comment_time_threshold = 4
handshake_comment_time_threshold = 4
last_pickup_comment_time = 0	# tracks time last comment was made
last_handshake_comment_time = 0
last_played_sound = 'none'
last_action_time = time.time()
lonely_threshold_time = 120

# MOTOR SPEED VARIABLES
# Only used to check current motor speed
motor_direction = 0
motor_speed_left = 0
motor_speed_right = 0

## Gripper Force Variable
gripper_force = 0

delay = 0.1 # update rate for main loop (s)

rospy.init_node("personality_controller", anonymous=False) # name the script on the ROS network

def msg_subscriber():
	# subscribe to ROS data updates
	#CP = rospy.Subscriber("currentPose", PoseStamped, current_pose_update)
	EN = rospy.Subscriber("/mechbot_12/get/encoder_status", std_msgs.msg.Int32MultiArray,encoder_status_update)
	IMU = rospy.Subscriber("/mechbot_12/get/IMU_status", std_msgs.msg.Float32MultiArray,imu_status_update)
	MD = rospy.Subscriber("/mechbot_12/set/motor_drive", std_msgs.msg.Int32MultiArray, motor_drive_update)
	GR = rospy.Subscriber("/mechbot_12/get/force_status", std_msgs.msg.Float32MultiArray, force_status_update)
	return (IMU, MD, EN)

def imu_status_update(data):
	global acc_x, acc_y, acc_z
	acc_x = data.data[0]
	acc_y = data.data[1]
	acc_z = data.data[2]
	#print("IMU")
	
def motor_drive_update(data):
	global last_action_time, motor_direction, motor_speed_left, motor_speed_right

	#print("motor drive update")

	motor_direction = data.data[0]
	motor_speed_left = data.data[1]
	motor_speed_right = data.data[2]
	if(motor_speed_left != 0 or motor_speed_right != 0):
		last_action_time = time.time()
	#print("MOTORS")

def encoder_status_update(data):
	global last_action_time
	global encoder_left, encoder_right
	encoder_left = data.data[0]
	encoder_right = data.data[1]

	if(encoder_left != 0 or encoder_right != 0):
		last_action_time = time.time()
	#print("ENCODERS")

def force_status_update(data):
	global gripper_force
	gripper_force = data.data[0]
	#print("gripper update %f") % gripper_force

def loop_timing(delay,loop_start):
	loop_sleep = delay - (time.time() - loop_start) # if loop delay too low then will print data faster than updates are recieved
	if loop_sleep > 0:
		time.sleep(loop_sleep)

def check_motors_stopped():
	if(motor_speed_left == 0 and motor_speed_right == 0):
		return True
	else:
		return False

def check_encoders_stopped():
	if(encoder_left == 0 and encoder_right == 0):
		return True
	else:
		return False

def check_IMU_published():
	if(acc_x == -999 and acc_y == -999 and acc_z == -999):
		return False
	else:
		return True
		
def react_to_pickup():
	global last_pickup_comment_time, pickup_threshold, pickup_comment_time_threshold, last_action_time
	
	if(check_IMU_published()):
		# detect if picked up from IMU data - z liftoff means picked up any time, otherwise if motors are stopped but platform experiences any acceleration
		if(math.fabs(acc_z+1) > pickup_threshold):# or ((math.fabs(acc_x) > pickup_threshold or math.fabs(acc_y) > pickup_threshold) and (check_motors_stopped() and check_encoders_stopped()))):
			print("Put me down!")
			# robot is being handled / carried. Respond if appropriate:
			if(time.time() - last_pickup_comment_time > pickup_comment_time_threshold):
				# enough time has passed to make another remark...
				pf.play_sound_group(sound_group_pickup,100)
				last_pickup_comment_time = time.time()
				last_action_time = time.time()

def react_to_pushed():
	global last_pickup_comment_time, pickup_threshold, pushed_comment_time_threshold, last_action_time

	# if the wheels are moving but the motors are off, someone may be pushing the mechbot along
	if(check_motors_stopped() == True and check_encoders_stopped() == False):

		# wait a little bit, otherwise we'd detect the mechbot's stopping motions (motors told off but wheels still move for a moment)
		time.sleep(0.5)

		# check again - now we can be more certain if this is deliberate
		if(check_motors_stopped() == True and check_encoders_stopped() == False):
			print("Motor speeds: %d %d, Encoder Values: %d %d") % ( motor_speed_left, motor_speed_right, encoder_left, encoder_right)
			print("Quit pushing me around!")
			if(time.time() - last_pickup_comment_time > pushed_comment_time_threshold):
				# enough time has passed to make another remark...
				pf.play_sound_group(sound_group_push,100)
				last_pickup_comment_time = time.time()
				last_action_time = time.time()
			
def react_to_lonely():
	global last_action_time, lonely_threshold_time
	if(time.time() - last_action_time > lonely_threshold_time):
		print("Getting lonely here...")
		pf.play_sound_group(sound_group_lonely,30)
		last_action_time = time.time()

def react_to_gripper_squeeze():
	global last_handshake_comment_time, handshake_comment_time_threshold
	if(gripper_force > 200):
		if(time.time() - last_handshake_comment_time > handshake_comment_time_threshold):
			print("Handshake?")
			pf.play_sound_group(sound_group_hello,100)
			last_action_time = time.time()
			last_handshake_comment_time = time.time()

def personality_core_init():
	random.seed()
	pf.sound_init()
	time.sleep(1)
	print("Hello! Personality Core initialised!")
	pf.play_sound_group(sound_group_startup,100)
	pf.block_wait_sound_finish()
	servo.publish_servo(1000,550)	#move servo arm out of way for navigation / object avoidance

def personality_core_update():
	react_to_pickup()
	react_to_lonely()
	react_to_pushed()
	react_to_gripper_squeeze()
	
def personality_core_shutdown():
	pf.play_sound_group(sound_group_shutdown,100)
	print ("Personality Core Disengaged. Shutting down...")

	pf.block_wait_sound_finish()

	pygame.mixer.quit()
	
def main(argv):
	key_pressed = False
	
	#subscribe to the relevant ROS topics
	(update1, update2, update3) = msg_subscriber()
	
	personality_core_init()
	
	while key_pressed == False:
		loop_start = time.time() # get loop time at start for loop rate calculations
		
		personality_core_update()
		loop_timing(delay,loop_start)
		
		key_pressed = select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []) # is key pressed?
	
	update1.unregister()
	update2.unregister()
	update3.unregister()
	personality_core_shutdown()	

if __name__ == '__main__': # main loop
	try: # if no problems
		main(sys.argv[1:])
        
 	except Exception,e: # if a problem
 		print ("Error!")
		print(e)
		pf.play_sound_group(sound_group_error,100)
		pf.block_wait_sound_finish()
		pass
 
