#!/usr/bin/env python

"""
This program accepts commands to drive the platform and translates them into commands which are sent to the motors.

"""

# system libraries
import time
import select
import sys
import socket
import os
import math

# ROS libraries
import rospy
import std_msgs.msg
from geometry_msgs.msg import Point, Quaternion, PoseStamped, Pose
import tf

threshold = 0.1 # units are in metres, reached target if x & y within 0.1 = 10cm of target position
rot_threshold = 0.1	# angle in radians, consider heading correct if within this number of radians to target point

#current pose variables
x = 0
y = 0
th = 0

#desired pose variables
d_x = 4.87
d_y = 1.85
d_th = 0

delay = 0.2 # update rate for main loop (s)

rospy.init_node("motor_pilot", anonymous=False) # name the script on the ROS network

pub = rospy.Publisher("/%s/set/motor_drive" % socket.gethostname(), std_msgs.msg.UInt8MultiArray, queue_size=10) # sets up the topic for publishing the motor commands

time.sleep(0.2) # make sure publisher setup

def publish_motor_command(md_d, md_l, md_r): # subroutine
	motor_data = std_msgs.msg.UInt8MultiArray() # definitions in std_msgs.msg - data to be published need to be in ROS format
	motor_data.data = [1,0,0,1]
	motor_data.data[0] = int(md_d)
	motor_data.data[1] = int(md_l)
	motor_data.data[2] = int(md_r)
	print ("Direction: %d, Left: %d, Right: %d") % (motor_data.data[0], motor_data.data[1], motor_data.data[2]) # print new motor speed on the terminal
	pub.publish(motor_data) # publish motor command to ROS

def pose_subscriber():
	# subscribe to ROS data updates
	PS = rospy.Subscriber("currentPose", PoseStamped, current_pose_update)
	DP = rospy.Subscriber("desiredPose", Pose, desired_pose_update)
	return (PS, DP)

def current_pose_update(data):
	global x, y, th

	x = data.pose.position.x
	y = data.pose.position.y
	#q = data.pose.orientation
	
	q = (
	    data.pose.orientation.x,
	    data.pose.orientation.y,
	    data.pose.orientation.z,
	    data.pose.orientation.w)
	    
	euler = tf.transformations.euler_from_quaternion(q)
	th = euler[2]

def desired_pose_update(data):
	global d_x, d_y, d_th

	d_x = data.position.x
	d_y = data.position.y
	q = (
	    data.orientation.x,
	    data.orientation.y,
	    data.orientation.z,
	    data.orientation.w)
	    
	euler = tf.transformations.euler_from_quaternion(q)
	th = euler[2]

def coordinates_reached():
	global threshold
	reached = False
	if(math.fabs(d_x-x) < threshold and math.fabs(d_y-y) < threshold):
		reached = True
	return reached

def main(argv):
	global x, y, th, d_x, d_y, d_th

	moving = False	# tracks if we are currently moving towards the target point

	#rotation PID
	rot_P = 20.0
	rot_I = 0.0
	rot_D = 0.0
	
	(current_pose_update, desired_pose_update) = pose_subscriber()
	
	key_pressed = False
	
	while key_pressed == False:
		loop_start = time.time() # get loop time at start for loop rate calculations
		print("Main loop beginning...")
		print("Current Position: %.3f %.3f %.3f | Target Position: %.3f %.3f %.3f") % (x,y,th,d_x,d_y,d_th)
		if(coordinates_reached() == False):
			print("Coordinates not reached...")
			if(moving == False):
				print("Not currently moving...")
				# calculate angular difference
				dx = x - d_x
				dy = y - d_y
				target_angle = math.atan2(-dy,-dx)
				target_angle  %= 2*math.pi
				angular_difference = math.atan2(math.sin(target_angle - th), math.cos(target_angle - th))
				print("%.3f radians off target")  % (angular_difference)
				if(math.fabs(angular_difference) < rot_threshold):
					print("Heading achieved!")
					# we're facing the right way, so stop and drive straight!
					# first, stop motors
					publish_motor_command(1,0,0)
					time.sleep(1) # make sure message has time to be enacted
					
					# drive straight
					publish_motor_command(0,50,50)
					moving = True
					time.sleep(1) # make sure message has time to be enacted
				else:
					print("Turning to face heading... ")
					#rotate to face heading
					# start motors moving based on angular difference
					motor_speed = int(round(rot_P * math.fabs(angular_difference)))

					if(angular_difference > 0):
						rot = 3
					else:
						rot = 2
					publish_motor_command(rot,motor_speed,motor_speed)
		else:
			print("Coordinates reached!")
			if(moving == True):
				publish_motor_command(1,0,0)
				moving = False
				print("Motors stopped!")
	
		loop_sleep = delay - (time.time() - loop_start) # if loop delay too low then will print data faster than updates are recieved
		if loop_sleep > 0:
			time.sleep(loop_sleep)
		key_pressed = select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []) # is key pressed?
	
	# finally, stop motors
	time.sleep(0.1) # make sure message has time to be enacted
	publish_motor_command(1,0,0)
	time.sleep(0.5) # make sure message has time to be enacted
	
	print ("Exit script, motors stopped")

if __name__ == '__main__': # main loop
	try: # if no problems
		main(sys.argv[1:])
        
	except rospy.ROSInterruptException: # if a problem
 		pass




