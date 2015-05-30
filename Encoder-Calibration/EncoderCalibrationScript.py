#!/usr/bin/env python

# import libraries
import time
import datetime
import select
import socket
from os.path import expanduser
import sys
import os

# ROS libraries
import rospy
import std_msgs.msg


def main():
    time.sleep(1)
    os.system("python drive_motors.py &")
    os.system("rosbag record -O trialN /mechbot_12/get/IMU_status /mechbot_12/get/encoder_status /mechbot_12/get/motor_status /mechbot_12/get/vicon_pose /odom")
    time.sleep(16)
    ros_update_1.unregister() # unregister from update 1
    print "unregistered"
    sys.exit()


if __name__ == '__main__':
    main()
