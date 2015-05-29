#!/usr/bin/env python

"""
vicon_get.py - Version 2.0 2015-01-21

This program is used to access object specific pose information
from the vicon tracking system.

Created for the University of South Australia's MechBot platform.
Copyright (c) 2014-2015 Russell Brinkworth. All rights reserved.

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

# system libraries
import time
import datetime
import sys
import select
import socket
from os.path import expanduser

# ROS
import rospy
import std_msgs.msg

save_data = True

object_name = '12Lgroup' # name of the object you are looking for on the Vicon system

delay = 0.2 # update rate for main loop (s)

v_pose = [0.0, 0.0, 0.0] # vicon pose data (x, y, yaw)

rospy.init_node("vicon_get_pose", anonymous=False) # name the script on the ROS network


if(save_data): # setup to save data to file
    filename = 'ViconLogs\vicon_log_{1}.txt'.format(expanduser("~"), datetime.datetime.now()) # modify if a different directory or filename required
    open(filename, 'w').close() # blank the file. Opening a file in write or 'w' mode empties it
    f = open(filename, 'a') # open file for appending
    f.write("Time\tX\tY\tTheta\n") # write titles on data columns. Tab delimited with a new line at the end


# subroutines

def vicon_update(data):
    words = data.data.split() # raw vicon data is in the form (x, y, z, pitch, roll, yaw)
    if words[0] == object_name: # only get data if it has the name you are looking for
        v_pose[0] = float(words[1])/1000 # x, convert to m
        v_pose[1] = float(words[2])/1000 # y 
        v_pose[2] = float(words[6]) # yaw, keep in rad in range (pi, -pi)
        
    print("x: %.3f  y: %.3f  Th: %.1f") % (v_pose[0], v_pose[1], v_pose[2]) # print 2D pose data to terminal
    if(save_data):
        f.write('{0}\t{1}\t{2}\n'.format(v_pose[0], v_pose[1], v_pose[2])) # write data to file

	
def subscriber():
    # subscribe to ROS data updates
    VS = rospy.Subscriber("/mechbot_12/get/vicon_pose", std_msgs.msg.String, vicon_update)
    return (VS)


# main program

def main():
    (ros_update_1) = subscriber() # subscribe to ROS topics
    time.sleep(1) # give sufficient time to start getting data
    
    key_pressed = False
	
    while key_pressed == False:
        loop_start = time.time() # get loop time at start for loop rate calculations
        
        loop_sleep = delay - (time.time() - loop_start) # if loop delay too low then will print data faster than updates are recieved
        if loop_sleep > 0:
            time.sleep(loop_sleep)
        key_pressed = select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []) # is key pressed?

    ros_update_1.unregister() # unregister from update 1
    if(save_data):
        f.close() # close data file on disk
    print "unregistered"
    sys.exit()


if __name__ == '__main__':
    main()
