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

sensor1 = 0
sensor2 = 0
sensor3 = 0
sensor4 = 0


rospy.init_node("get_ir", anonymous=False) # name the script on the ROS network

def subscriber():
    # subscribe to ROS data updates
    VS = rospy.Subscriber("/mechbot_12/get/IR_status", std_msgs.msg.Float32MultiArray, ir_update)
    return (VS)
    
def ir_update(data):
  
    sensor1 = data.data[0]
    sensor2 = data.data[1]
    sensor3 = data.data[2]
    sensor4 = data.data[3]
    
    print("Sensor1: %.3f Sensor2: %.3f Sensor3: %.3f Sensor4: %.3f") % (sensor1, sensor2, sensor3, sensor4)
    if (sensor1 > 300):
            print("Warning!")
            os.system('python motor_pilot.py 0 50 50 1 &')
    
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
