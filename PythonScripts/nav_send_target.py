#!/usr/bin/env python

#accepts arguments from command line and parses into desiredPose ROS message for navigation

# system libraries
import time
import sys
import select
import socket

# ROS
import rospy
import std_msgs.msg
from geometry_msgs.msg import Point, Quaternion, Pose
import tf

from OurModules import functions_nav_control as nav

rospy.init_node("send_desired_pose", anonymous=False) # name the script on the ROS network


# setup publishing pose messages
pose_pub = rospy.Publisher('desiredPose', Pose, queue_size=10)
odom_broadcaster = tf.TransformBroadcaster()

# main program

def main(argv):
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    th = float(sys.argv[3])

    print("x: %.3f  y: %.3f  Th: %.1f") % (x, y, th) # print 2D pose data to terminal
	
    nav.send_target_pose(x,y,th)
    
    key_pressed = False
    while key_pressed == False:
         key_pressed = select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []) # is key pressed?
         time.sleep(0.5)
         
    sys.exit()


if __name__ == '__main__':
    main(sys.argv[1:])

