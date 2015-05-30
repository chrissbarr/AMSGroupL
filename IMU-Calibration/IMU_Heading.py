#!/usr/bin/env python

# Localisation script for the mechbots
# Written by Russell Brinkworth 2013 & 2014

import time
import datetime
import math
import signal
import re
import sys
import select
from os.path import expanduser


# ROS
from rostools.rosthread import RosThread
from rostools.topicmonitor import TopicMonitor # roscore needs to be running

try:
    import roslib
    import rospy
    import std_msgs.msg
except:
    raise Exception("The module 'roslib' or 'rospy' are missing. Have you installed ROS and/or added /opt/ros/fuerte/setup.sh to your bash profile?")

# Globals
gyro_scale = 0.01 # resolution of gyroscope (s/rad)
m_offset = -1.43833 # offset of magnotometer to align with external localisation system
delay = 0.1 # update rate for orientation information in s
Save_imu = True

imu = [-10000, 0, 0, 0, 0, 0, 0, 0, 0] # IMU data (x-axis accelerometer, y-axis accelerometer, z-axis accelerometer, x-axis magnetometer, y-axis magnetometer, z-axis magnetometer, x-axis gyro, y-axis gyro, z-axis gyro, z-axis gyro with offset removed)
Mag_scale = [1.0, 1.0] # IMU magnetometer scale (x, y)
Mag_offset = [0.0, 0.0] # IMU magnetometer offsets (x, y)
fusion_imu = [0.6, 0.4] # relative contributions of components in IMU to heading estimation (magnetometer, gyroscope)

if(Save_imu): # setup to save corrected magnotometer values to data to file
    filename = '{0}/Desktop/IMU_{1}.txt'.format(expanduser("~"), datetime.datetime.now())
    # Blank the file. Opening a file in write or 'w' mode empties it
    open(filename, 'w').close()
    f = open(filename, 'a') # Open File for Appending
    # Add titles
    f.write("Time\tMag_x\tMag_y\tMag_theta\tIMU_theta\n") # write titles on data columns
    start_time = time.time() # time that the program started. Use to find update times for data

# subroutines

def imu_update(data):
    for i in range(len(data.data)):
        imu[i] = data.data[i]

def subscriber():
    # subscribe to ROS data updates
    IS = rospy.Subscriber("/mechbotugv/hardware/IMU_Status", std_msgs.msg.Float32MultiArray, imu_update)
    print IS
    return (IS)
    time.sleep(3)

def find_imu_heading(heading_past, loop_time_past):
    loop_time = time.time()
    if imu[0] != -10000: # check to see if imu is present
        a_roll = math.atan2(imu[1], imu[2])
        a_pitch = math.atan2(-imu[0], math.pow(imu[2]*imu[2] + imu[1]*imu[1], 0.5))
        mag_x = (imu[3]*math.cos(a_pitch) + imu[4]*math.sin(a_roll)*math.sin(a_pitch) + imu[5]*math.cos(a_roll)*math.sin(a_pitch) - Mag_offset[0])/Mag_scale[0] # correct magnotometer x-axis for roll and pitch. Also incldes offset value to compensated for hard iron offset, to ensure equal variation about 0, i.e. max = -min
        mag_y = (imu[4]*math.cos(a_roll) - imu[5]*math.sin(a_roll) - Mag_offset[1])/Mag_scale[1]
        
        mag_yaw = -math.atan2(mag_y, mag_x) - m_offset # find magnetic yaw value
        if mag_yaw < -math.pi:
            mag_yaw = mag_yaw + 2*math.pi
        elif mag_yaw > math.pi:
            mag_yaw = mag_yaw - 2*math.pi
    else:
        mag_x = 0
        mag_y = 0
        mag_yaw = -500 # make it a crazy number if not available
                    
    if heading_past == -500.0:   # will only happen on first run, i.e. yaw is impossible value
        heading_past = mag_yaw # take magnotometer value on first run
            
    # find gyro angle update                                   
    gyro_yaw = imu[7]*math.sin(a_pitch) - imu[6]*math.cos(a_pitch)*math.sin(a_roll) - imu[8]*math.cos(a_roll)*math.cos(a_pitch) # correct gyro for offset of imu
    g_dyaw = gyro_yaw*(loop_time - loop_time_past)/gyro_scale # find change in yaw based on gyroscope
                                   
    # update angle
    imu_heading = math.atan2(math.sin(mag_yaw)*fusion_imu[0] + math.sin(heading_past+g_dyaw)*fusion_imu[1], math.cos(mag_yaw)*fusion_imu[0] + math.cos(heading_past+g_dyaw)*fusion_imu[1]) # fuse magnetometer and gyro
    
    if(Save_imu): # write to disk if requested to
        current_time = time.time() - start_time # find time of update
        f.write('{0}\t{1}\t{2}\t{3}\t{4}\n'.format(current_time, mag_x, mag_y, mag_yaw, imu_heading)) # write data to file
            
    return (imu_heading, loop_time) # return values


# main program

def main():
    (ros_update_1) = subscriber() # subscribe to ROS topics
    time.sleep(3) # give sufficient time to start getting data
	
    heading_past = -500.0 # previous value heading value, used for gyroscope
    l_time = time.time() # dummy variable to hold previous loop time for gyroscope calculation
    key_pressed = False
	
    while key_pressed == False:
        loop_start = time.time() # get loop time at start for loop rate calculations
        (heading_past, l_time) = find_imu_heading(heading_past, l_time) # find heading from IMU
        print (heading_past)
        loop_sleep = delay - (time.time() - loop_start) # if loop delay too low then will calculate pose faster than updates are recieved
        if loop_sleep > 0:
            time.sleep(loop_sleep)
        key_pressed = select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []) # is key pressed?

    ros_update_1.unregister() # unregister from update 1
    if(Save_imu):
        f.close() # close data file on disk
    print "unregistered"
    sys.exit()


if __name__ == '__main__':
    main()
