#!/usr/bin/env python

"""
Functions for sending commands to the Mechbot's motors.
"""

# system libraries
import select
import sys
import socket
import os
import math

# ROS libraries
import rospy
import std_msgs.msg

pub = rospy.Publisher("/%s/set/motor_drive2" % socket.gethostname(), std_msgs.msg.Int32MultiArray, queue_size=10) # sets up the topic for publishing the motor commands
pub_direct = rospy.Publisher("/%s/set/motor_drive" % socket.gethostname(), std_msgs.msg.Int32MultiArray, queue_size=10) # sets up the topic for publishing the motor commands

# send a command to the motors (direction, left_speed, right_speed)
def publish_command(md_d, md_l, md_r):
	motor_data = std_msgs.msg.Int32MultiArray() # definitions in std_msgs.msg - data to be published need to be in ROS format
	motor_data.data = [1,0,0,1]
	motor_data.data[0] = int(md_d)
	motor_data.data[1] = int(md_l)
	motor_data.data[2] = int(md_r)
	#print ("Direction: %d, Left: %d, Right: %d") % (motor_data.data[0], motor_data.data[1], motor_data.data[2]) # print new motor speed on the terminal
	pub.publish(motor_data) # publish motor command to ROS

# send a command to the motors (direction, left_speed, right_speed), bypassing object-avoidance
def publish_command_direct(md_d, md_l, md_r):
	motor_data = std_msgs.msg.Int32MultiArray() # definitions in std_msgs.msg - data to be published need to be in ROS format
	motor_data.data = [1,0,0,1]
	motor_data.data[0] = int(md_d)
	motor_data.data[1] = int(md_l)
	motor_data.data[2] = int(md_r)
	#print ("Direction: %d, Left: %d, Right: %d") % (motor_data.data[0], motor_data.data[1], motor_data.data[2]) # print new motor speed on the terminal
	pub_direct.publish(motor_data) # publish motor command to ROS

