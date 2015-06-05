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

target_x = target_y = target_th = -999

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

def target_pose_update(data):
    global target_x, target_y, target_th

    # read in position
    target_x = data.position.x
    target_y = data.position.y
    
    # read in orientation
    q = (
        data.orientation.x,
        data.orientation.y,
        data.orientation.z,
        data.orientation.w)
        
    # convert orientation from quaternion to euler angles, read yaw
    euler = tf.transformations.euler_from_quaternion(q)
    target_th = euler[2]

def target_pose_subscribe():
    DP = rospy.Subscriber("targetPose", Pose, target_pose_update)

def init():
    target_pose_subscribe()

