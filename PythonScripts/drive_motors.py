#!/usr/bin/env python

"""
drive_motors.py - Version 2.0 2015-01-19

This program creates a list of commands that are executed in order
to control the speeds of the motors on the MechBots.

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
import select
import sys
import socket

# ROS libraries
import rospy
import std_msgs.msg

motor_command_list = [[0, 15, 15, 5]]
# Variable is in the form [direction, left speed, right speed, time] with any number of elements so different motor commands can be queued up
# Direction: 0 = straight, 1 = reverse, 2 = clock-wise, 3 = counter-clock-wise
# Left / Right Speed: how fast the motors should turn. Max of 255
# Time: is the time (in seconds) to to the defined task. The resolution is 0.01s.

motor_data = std_msgs.msg.Int32MultiArray() # definitions in std_msgs.msg - data to be published need to be in ROS format
motor_data.data = [1, 75, 25, 1] # actual data to be sent to the drive control on the MechBot
# variable is in the form [direction, left speed, right speed, independant control]
# Direction: 0 = straight, 1 = reverse, 2 = clock-wise, 3 = counter-clock-wise
# Left / Right Speed: how fast the motors should turn. Max of 255
# Independent Control: 1 = send specified values to each wheel, otherwise use left speed for both left and right wheel
#   : if set to 1 and left and right wheels are different then MechBot will trace an arc
#   : in this example it is always 1

rospy.init_node("motor_drive_test", anonymous=False) # name the script on the ROS network

pub = rospy.Publisher("/%s/set/motor_drive2" % socket.gethostname(), std_msgs.msg.Int32MultiArray, queue_size=10) # sets up the topic for publishing the motor commands

time.sleep(0.2) # make sure publisher setup

def publish_motor_command(exit_loop): # subroutine
    for i in range(len(motor_command_list)): # run for number of commands specified
        if exit_loop == False: # Only do if user has not requested a stop
            loop_start = time.time()
            motor_data.data[0] = motor_command_list[i][0] # specify direction to travel
            motor_data.data[1] = motor_command_list[i][1] # change the speed to be sent to the left motor
            motor_data.data[2] = motor_command_list[i][2] # change the speed to be sent to the right motor
            print ("Direction: %d, Left: %d, Right: %d, Time: %.2f") % (motor_data.data[0], motor_data.data[1], motor_data.data[2], motor_command_list[i][3]) # print new motor speed on the terminal
            pub.publish(motor_data) # publish motor command to ROS
            while time.time() - loop_start < motor_command_list[i][3] and exit_loop == False: # keep the same values being sent to the motors for time specified or until key pressed
                if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []): # look for key press to exit
                    exit_loop = True # if key pressed then set exit to true
                    print ("Stopped By User")
                time.sleep(0.01) # mini sleep while looking for key presses

if __name__ == '__main__': # main loop
    try: # if no problems
        time.sleep(.2)
        publish_motor_command(False) # run subroutine, set exit_loop to false
        motor_data.data[1] = 0 # once all commands finished then stop the motors
        motor_data.data[2] = 0
        print ("Task Complete") # tells user command list is finished
        pub.publish(motor_data) # publish motor command to ROS
        time.sleep(0.5) # make sure message has time to be enacted
        pub.publish(motor_data) # publish motor command to ROS
        time.sleep(0.5) # make sure message has time to be enacted
        sys.exit(0) # cleanly terminate the program
        
    except rospy.ROSInterruptException: # if a problem
        pass




