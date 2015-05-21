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

min_distance = 150
max_distance = 500

sensor1 = 0
sensor2 = 0
sensor3 = 0
sensor4 = 0

# sensors map range to analog value roughly according to power equation (i.e. dist = c * analog^b)
# below arrays hold 'c' and 'b' for each sensor

sensor_equation_multiplier = [118514, 39041, 81533, 112346]
sensor_equation_exponent = [-1.252, -1.052, -1.199, -1.251]

rospy.init_node("get_ir", anonymous=False) # name the script on the ROS network

pub = rospy.Publisher('/%s/set/motor_drive' % socket.gethostname(), std_msgs.msg.UInt8MultiArray, queue_size=10) # sets up the topic for publishing the motor commands
time.sleep(0.1) # delay


def subscriber():
    # subscribe to ROS data updates
    VS = rospy.Subscriber("/mechbot_12/get/IR_status", std_msgs.msg.Float32MultiArray, ir_update)
    return (VS)
    
def ir_update(data):
    global sensor1, sensor2, sensor3, sensor4
  
    start_time = time.time()
    sensor1 = ir_to_mm(data.data[0],1) #rear sensor
    sensor2 = ir_to_mm(data.data[1],2) #right sensor
    sensor3 = ir_to_mm(data.data[2],3) #left sensor
    sensor4 = ir_to_mm(data.data[3],4) #front sensor
    
    time_taken = time.time() - start_time
    
    print("Sensor1: %.3fmm Sensor2: %.3fmm Sensor3: %.3fmm Sensor4: %.3fmm") % (sensor1, sensor2, sensor3, sensor4)

def ir_to_mm(analog_value, sensor_num):
    # arrays are 0 indexed, but sensor_num is setup for readability (1-4 for sensors)
    array_index = sensor_num-1
    #calculate distance according to each sensors profile
    distance_in_mm = sensor_equation_multiplier[array_index] * pow(analog_value,sensor_equation_exponent[array_index])
    #distance_in_mm = 118514 * pow(analog_value,-1.252)
    
    
    #if(distance_in_mm<min_distance):
     #   distance_in_mm = max_distance
    #if(distance_in_mm>max_distance):
     #   distance_in_mm=max_distance
        
    return distance_in_mm
    
# send a command to the motors (direction, left_speed, right_speed)
def publish_motor_command(md_d, md_l, md_r):
	motor_data = std_msgs.msg.UInt8MultiArray() # definitions in std_msgs.msg - data to be published need to be in ROS format
	motor_data.data = [1,0,0,1]
	motor_data.data[0] = int(md_d)
	motor_data.data[1] = int(md_l)
	motor_data.data[2] = int(md_r)
	print ("Direction: %d, Left: %d, Right: %d") % (motor_data.data[0], motor_data.data[1], motor_data.data[2]) # print new motor speed on the terminal
	pub.publish(motor_data) # publish motor command to ROS
    
def main():
    ros_update_1 = subscriber() # subscribe to ROS topics
    time.sleep(1) # give sufficient time to start getting data
    
    key_pressed = False
	
    while key_pressed == False:
    
    	if(sensor4 < 100):
            publish_motor_command(3, 30, 30) # stop
                          
    	elif(sensor2 < 100):
            publish_motor_command(3, 60, 60) # turn left	
                          			
    	elif(sensor3 < 100):
            publish_motor_command(2, 60, 60) # turn right	
						  
    	else:
        	publish_motor_command(0, 60, 60) # Drive Forward
            
    	rospy.sleep(0.1)
        
        
        key_pressed = select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []) # is key pressed?

    publish_motor_command(0, 0, 0) # stop
    rospy.sleep(0.5)
    publish_motor_command(0, 0, 0) # stop
    ros_update_1.unregister() # unregister from update 1
    print "unregistered"
    sys.exit()


if __name__ == '__main__':
    try:
    	main()
            
    except rospy.ROSInterruptException: # if a problem occurs
        pass
  



