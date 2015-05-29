#!/usr/bin/python

# make sure first line has location of python install. Use command 'which python' if you do not know

"""
save_encoders.py - Version 2.0 2015-01-21

This program saves the speeds of the two drive wheels on the MechBot
as well as the motor setpoint. It assumes the set points (i.e. speeds)
sent to both wheels are equal.

Created for the University of South Australia's MechBot platform.
Copyright (c) 2013-2015 Russell Brinkworth. All rights reserved.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details at:

http://www.gnu.org/licenses/gpl.html

"""

# import libraries
import time
import datetime
import select
import socket
from os.path import expanduser
import sys

# ROS libraries
import rospy
import std_msgs.msg

# define settings
runtime = 100 # maximum runtime (seconds)
delay = 0.1 # loop delay time (seconds)
save_data = True # save data? (True or False)
start_time = time.time() # time that the program started. Use to find update times for data

# define variables
encoder = [0.0, 0.0, 0.0] # velocities of wheels (set point, left, right). Define so list length known. Modify if different wheel set points are required

if(save_data): # setup to save data to file
    filename = 'Wheel_Speeds_{1}.txt'.format(expanduser("~"), datetime.datetime.now()) # modify if a different directory or filename required
    open(filename, 'w').close() # blank the file. Opening a file in write or 'w' mode empties it
    f = open(filename, 'a') # open file for appending
    f.write("Time\tLeft Total\tRight Total\n") # write titles on data columns. Tab delimited with a new line at the end

rospy.init_node("save_encoders", anonymous=False) # name the script on the ROS network

# subroutines
def set_point_update(data): # run if an update to the requested motor velocities. Normally is (direction, left speed, right speed, independant control) but from web interface left and right speeds are the same so only need one
	encoder[0] = ord(data.data[1]) # only get left wheel speed command convert char to int

def encoder_update(data): # run if an update to the encoder values data = (left velocity, right velocity, left displacement, right displacement)
    current_time = time.time() - start_time # find time of update
    encoder[1] = data.data[2] # get left wheel velocity
    encoder[2] = data.data[3] # get right wheel velocity
    print "Time: %.3f Left: %.0f Right: %.0f" % (current_time, encoder[1], encoder[2]) # formatted data printed to terminal
    if(save_data):
        f.write('{0}\t{1}\t{2}\n'.format(current_time, encoder[1], encoder[2])) # write data to file

def subscriber():
    # subscribe to data updates on ROS network
    ES = rospy.Subscriber("/mechbot_12/get/encoder_status", std_msgs.msg.Int32MultiArray, encoder_update) # run subroutine 'encoder_update' if new data found on topic 'Encoder_Status'
    MD = rospy.Subscriber("/mechbot_12/set/motor_drive", std_msgs.msg.UInt8MultiArray, set_point_update) # run subroutine 'set_point_update' if new data found on topic 'Motor_Drive'
    return (ES, MD) # return the handles for the subscribed statuses
    time.sleep(1) # delay to ensure time to properly subscribe to ROS topics
	
# main program
def main():
    (ros_update_1, ros_update_2) = subscriber() # subscribe to ROS topics
    key_pressed = False # variable to check for key press
    
    while key_pressed == False and time.time()-start_time <= runtime: # keep going if 'enter' key not pressed and maximum runtime not exceeded
        key_pressed = select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []) # is key pressed?
	time.sleep(delay) # loop delay to ensure do not take up too much of system resources
	
    ros_update_1.unregister() # unregister from update 1 so will not recieve any new updates
    ros_update_2.unregister() # unregister from update 2
    if(save_data):
        f.close() # close data file on disk
    print "unregistered"
    time.sleep(0.2) # make sure previous code has had time to finish
    sys.exit(0)

if __name__ == '__main__':
	main()
