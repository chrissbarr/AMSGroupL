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

usL = 0
usR = 0

rospy.init_node("get_us", anonymous=False) # name the script on the ROS network

def subscriber():
    # subscribe to ROS data updates
    SS = rospy.Subscriber("/mechbot_12/get/US_status", std_msgs.msg.Float32MultiArray, us_update)
    return (SS)
    
def us_update(data):
  
    usL = data.data[0]
    usR = data.data[1]
     
    print("UltrasonicL: %.3f UltrasonicR: %.3f") % (usL, usR)

    
def main():
    ros_update_1 = subscriber() # subscribe to ROS topics
    time.sleep(1) # give sufficient time to start getting data
    
    key_pressed = False
	
    while key_pressed == False:
        
        
        key_pressed = select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []) # is key pressed?
       
        time.sleep(0.1)
    ros_update_1.unregister() # unregister from update 1
    print "unregistered"
    sys.exit()


if __name__ == '__main__':
    main()

