#!/usr/bin/env python

"""
This script should be run at startup, or at least before the mechbot moves about.

It sits as an interface between the motors and the higher-level navigation systems.

During normal operation, this script will pass the values it receives on to the motors
in a transparent fashion.

If an obstacle is detected and it is likely that a collision will occur, this script will
alter the instructions being sent to the motors to attempt to avoid the collision.

It behaves on the following logic:

If the platform is driving forward and an obstacle exists in front of the platform:
    -rotate counter-clockwise until the object is no longer in front of the platform.
    -drive forwards until the right-facing sensors no longer detect the obstacle

If the platform is driving in reverse, similar operations will take place.

Rotational motor commands will not be prevented, as otherwise higher-level scripts
may cease functioning when close to obstacles. The goal is to avoid this occuring.
"""

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

"""
State variable tracks how the script is currently behaving. The following states are available:
state = 0 : default behaviour, motor commands are relayed directly as they are received, with no interference.
state = 1 : forward moves are blocked. Will be replaced with counter-clockwise rotations and forward move.
"""
state = 0
default_turn_direc = 3
default_turn_speed = 25

m_d = 0
m_l = 0
m_r = 0


ir_sensor1 = 0
ir_sensor2 = 0
ir_sensor3 = 0
ir_sensor4 = 0

us_left = 0
us_right = 0

# sensors map range to analog value roughly according to power equation (i.e. dist = c * analog^b)
# below arrays hold 'c' and 'b' for each sensor
sensor_equation_multiplier = [118514, 39041, 81533, 112346]
sensor_equation_exponent = [-1.252, -1.052, -1.199, -1.251]

rospy.init_node("ObjectAvoid", anonymous=False) # name the script on the ROS network

pub = rospy.Publisher('/%s/set/motor_drive' % socket.gethostname(), std_msgs.msg.Int32MultiArray, queue_size=10) # sets up the topic for publishing the motor commands
time.sleep(0.1) # delay


def subscriber():
    # subscribe to ROS data updates
    MD = rospy.Subscriber("/mechbot_12/set/motor_drive2", std_msgs.msg.Int32MultiArray, motor_update)
    IR = rospy.Subscriber("/mechbot_12/get/IR_status", std_msgs.msg.Float32MultiArray, ir_update)
    US = rospy.Subscriber("/mechbot_12/get/US_status", std_msgs.msg.Float32MultiArray, us_update)
    return (MD, IR, US)
    
def ir_update(data):
    global ir_sensor1, ir_sensor2, ir_sensor3, ir_sensor4
    ir_sensor1 = ir_to_mm(data.data[0],1) #rear sensor
    ir_sensor2 = ir_to_mm(data.data[1],2) #right sensor
    ir_sensor3 = ir_to_mm(data.data[2],3) #left sensor
    ir_sensor4 = ir_to_mm(data.data[3],4) #front sensor
    #print("Sensor1: %.3fmm Sensor2: %.3fmm Sensor3: %.3fmm Sensor4: %.3fmm") % (sensor1, sensor2, sensor3, sensor4)

def ir_to_mm(analog_value, sensor_num):
    array_index = sensor_num-1  # arrays are 0 indexed, but sensor_num is setup for readability (1-4 for sensors)
    distance_in_mm = sensor_equation_multiplier[array_index] * pow(analog_value,sensor_equation_exponent[array_index]) #calculate distance according to each sensors profile
    return distance_in_mm

def us_update(data):
    global us_left, us_right
    us_left = data.data[0]
    us_right = data.data[1]
    #print("UltrasonicL: %.3f UltrasonicR: %.3f") % (usL, usR)

def motor_update(data):
    global m_d, m_r, m_l

    print("motor update!")

    m_d = data.data[0] #direction (0 = forward, 1 = reverse, 2 = clockwise, 3 = counter-clockwise)
    m_l = data.data[1] #left speed
    m_r = m_l#data.data[2]  #right speed

    print("%s") % (m_d)

def publish_motor_command(md_d, md_l, md_r): # send a command to the motors (direction, left_speed, right_speed)
	motor_data = std_msgs.msg.Int32MultiArray() # definitions in std_msgs.msg - data to be published need to be in ROS format
	motor_data.data = [1,0,0,1]
	motor_data.data[0] = int(md_d)
	motor_data.data[1] = int(md_l)
	motor_data.data[2] = int(md_r)
	print("Motor data published! (%d, %d, %d)") % (md_d, md_l, md_r)
	pub.publish(motor_data) # publish motor command to ROS
    
def obstacle_in_direction(dir):
    """
    Checks if there is an obstacle in a given direction based on a set distance threshold, returns True / False.
    Direction 0 = in front, 1 = behind, 2 = right and 3 = left
    """
    if(dir == 0):   # is there something in front of the robot?
        if(ir_sensor4 < 90 or us_left < 1500 or us_right < 1500):
            return True
        else:
            return False
    if(dir == 1):   # is there something behind the robot?
        if(ir_sensor1 < 90):
            return True
        else:
            return False
    if(dir == 2):
        if(ir_sensor2 < 90):
            return True
        else:
            return False
    if(dir == 3):
        if(ir_sensor3 < 90):
            return True
        else:
            return False 

def main():
    global m_d, m_r, m_l, default_turn_direc, default_turn_speed
    (ros_update_1, ros_update_2, ros_update_3) = subscriber() # subscribe to ROS topics
    time.sleep(1) # give sufficient time to start getting data
    
    key_pressed = False
	
    while key_pressed == False:
        
        
        if((m_d == 0 or m_d == 1) and (m_l != 0 or m_r != 0)):   #if we're moving forwards or back (and motors are at non-zero speeds)
            print("Obstacle Ahead: %s | Behind: %s | Right: %s | Left: %s") % (obstacle_in_direction(0), obstacle_in_direction(1), obstacle_in_direction(2), obstacle_in_direction(3))
            if(obstacle_in_direction(m_d) == False):  
                print("Nothing in the way - proceed as normal!")
                publish_motor_command(m_d,m_l,m_r)   #pass motor commands through transparently
            else:
                print("We can't go that way right now!")
                #we can't drive that way right now, so let's do something else instead.
                #first we rotate until the way ahead is clear
                while(obstacle_in_direction(m_d) == True):
                    publish_motor_command(default_turn_direc,default_turn_speed,default_turn_speed)
                    time.sleep(0.3)

                #then we drive forwards (or backwards) until it looks like we're past the obstacle, or if we're going to hit something!
                while(obstacle_in_direction(m_d) == False and (obstacle_in_direction(2) == True or obstacle_in_direction(3) == True)):
                    publish_motor_command(m_d,m_l,m_r)
                    time.sleep(0.2)
        else:
        	publish_motor_command(m_d,m_l,m_r)   #pass motor commands through transparently

        time.sleep(.1)
        key_pressed = select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []) # is key pressed?

    publish_motor_command(0, 0, 0) # stop
    rospy.sleep(0.5)
    publish_motor_command(0, 0, 0) # stop
    ros_update_1.unregister() # unregister from update 1
    ros_update_2.unregister() # unregister from update 1
    ros_update_3.unregister() # unregister from update 1
    print "object avoidance unregistered"
    sys.exit()


if __name__ == '__main__':
    try:
    	main()        
    except rospy.ROSInterruptException: # if a problem occurs
        pass