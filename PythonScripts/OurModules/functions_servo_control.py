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

pub = rospy.Publisher("/%s/set/servo_drive" % socket.gethostname(), std_msgs.msg.Int32MultiArray, queue_size=10) # sets up the topic for publishing the motor commands

# send a command to the servos 
def publish_servo(gripper, arm):
	servo_data = std_msgs.msg.Int32MultiArray() # definitions in std_msgs.msg - data to be published need to be in ROS format
	servo_data.data = [1000,10000]
	servo_data.data[0] = int(gripper)
	servo_data.data[1] = int(arm)

	pub.publish(servo_data) # publish motor command to ROS

