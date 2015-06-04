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
from geometry_msgs.msg import Point, Quaternion, PoseStamped, Pose
import tf

pose_pub = rospy.Publisher('targetPose', Pose, queue_size=10)

# send target pose
def send_target_pose(x, y, th):
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