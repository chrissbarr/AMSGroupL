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

# ROS libraries
import rospy
import std_msgs.msg
from geometry_msgs.msg import Point, Quaternion, PoseStamped

#current pose variables
x = 0
y = 0
th = 0

#desired pose variables
d_x = 0
d_y = 0
d_th = 0


motor_data = std_msgs.msg.UInt8MultiArray() # definitions in std_msgs.msg - data to be published need to be in ROS format


rospy.init_node("motor_pilot", anonymous=False) # name the script on the ROS network

pub = rospy.Publisher("/%s/set/motor_drive" % socket.gethostname(), std_msgs.msg.UInt8MultiArray, queue_size=10) # sets up the topic for publishing the motor commands

time.sleep(0.2) # make sure publisher setup

def publish_motor_command(): # subroutine
       print ("Direction: %d, Left: %d, Right: %d") % (motor_data.data[0], motor_data.data[1], motor_data.data[2]) # print new motor speed on the terminal
       pub.publish(motor_data) # publish motor command to ROS

def pose_subscriber():
    # subscribe to ROS data updates
    PS = rospy.Subscriber("poseStamped", PoseStamped, current_pose_update)
    DP = rospy.Subscriber("desiredPose", Pose, desired_pose_update)
    return (PS, DP)

def current_pose_update(data):
    global x, y, th

    x = data.pose.position.x
    y = data.pose.position.y

    q = data.pose.orientation

    th = euler_from_quaternion(q).yaw

def desired_pose_update(data):
    global d_x, d_y, d_th

    x = data.position.x
    y = data.position.y

    q = data.orientation

    th = euler_from_quaternion(q).yaw
    

def main(argv):
    (current_pose_update, desired_pose_update) = pose_subscriber()
    
    
    key_pressed = False
	
    while key_pressed == False:
	loop_start = time.time() # get loop time at start for loop rate calculations
        
    	publish_motor_command() # run subroutine, set exit_loop to false
	

	loop_sleep = delay - (time.time() - loop_start) # if loop delay too low then will print data faster than updates are recieved
        if loop_sleep > 0:
            time.sleep(loop_sleep)
        key_pressed = select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []) # is key pressed?
    

if __name__ == '__main__': # main loop
    try: # if no problems
        main(sys.argv[1:])
        
    except rospy.ROSInterruptException: # if a problem
        pass




