#!/usr/bin/env python

"""
Encoder interface.
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

left_current, right_current, left_total, right_total = 0

# subscribe to wheel encoder messages
encoder_ros_update = rospy.Subscriber("/mechbot_12/get/encoder_status", std_msgs.msg.Int32MultiArray, wheel_callback)

def wheel_callback(data):
	global left_current, right_current, left_total, right_total

	left_current = data.data[0]
	right_current = data.data[1]
	left_total = data.data[2]
	right_total = data.data[3]
