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

# sensors map range to analog value roughly according to power equation (i.e. dist = c * analog^b)
# below arrays hold 'c' and 'b' for each sensor

sensor_equation_multiplier = [118514, 39041, 81533, 112346]
sensor_equation_exponent = [-1.252, -1.052, -1.199, -1.251]

rospy.init_node("get_ir", anonymous=False) # name the script on the ROS network

def subscriber():
    # subscribe to ROS data updates
    VS = rospy.Subscriber("/mechbot_12/get/IR_status", std_msgs.msg.Float32MultiArray, ir_update)
    return (VS)
    
def ir_update(data):
  
	start_time = time.time()
	
    sensor1 = ir_to_mm(data.data[0],1)
    sensor2 = ir_to_mm(data.data[1],2)
    sensor3 = ir_to_mm(data.data[2],3)
    sensor4 = ir_to_mm(data.data[3],4)
    
    time_taken = time.time() - start_time
    
    print("Sensor1: %.3f Sensor2: %.3f Sensor3: %.3f Sensor4: %.3f | Time Taken: ") % (sensor1, sensor2, sensor3, sensor4, time_taken)

def ir_to_mm(analog_value, sensor_num):
	# arrays are 0 indexed, but sensor_num is setup for readability (1-4 for sensors)
	array_index = sensor_num-1
	#calculate distance according to each sensors profile
    distance_in_mm = sensor_equation_multiplier[array_index] * pow(analog_value,sensor_equation_exponent[array_index])
    return distance_in_mm
    
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
