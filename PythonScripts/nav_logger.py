#!/usr/bin/python


"""
Will log current pose and target pose to timestamped text file.

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

from OurModules import functions_nav_control as nav
from OurModules import functions_localisation as loc

# define settings
runtime = 100 # maximum runtime (seconds)
delay = 0.1 # loop delay time (seconds)
save_data = True # save data? (True or False)
start_time = time.time() # time that the program started. Use to find update times for data

if(save_data): # setup to save data to file
	filename = '{0}/data_log/nav_logger_{1}.txt'.format(expanduser("~"), datetime.datetime.now()) # modify if a different directory or filename required
	open(filename, 'w').close() # blank the file. Opening a file in write or 'w' mode empties it
	f = open(filename, 'a') # open file for appending
	f.write("Time\tX\tY\tTh\tT_X\tT_Y\tT_Th\n") # write titles on data columns. Tab delimited with a new line at the end

rospy.init_node("nav_logger", anonymous=False) # name the script on the ROS network

# main program
def main():

	nav.init()
	loc.init()

	key_pressed = False # variable to check for key press
	print("Time\tX\tY\tTh\tT_X\tT_Y\tT_Th\n")
	while key_pressed == False and time.time()-start_time <= runtime: # keep going if 'enter' key not pressed and maximum runtime not exceeded
		
		current_time = time.time() - start_time # find time of update

		print("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f") % (current_time, loc.vicon_x, loc.vicon_y, loc.vicon_th, nav.target_x, nav.target_y, nav.target_th)
		
		if(save_data):
			f.write('{0}\t{1}\t{2}\t{3}\t{4}\t{5}\t{6}\n'.format(current_time, loc.vicon_x, loc.vicon_y, loc.vicon_th, nav.target_x, nav.target_y, nav.target_th)) # write data to file
		
		key_pressed = select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []) # is key pressed?
		time.sleep(delay) # loop delay to ensure do not take up too much of system resources

	if(save_data):
		f.close() # close data file on disk

	print "unregistered"
	time.sleep(0.2) # make sure previous code has had time to finish

	sys.exit(0)

if __name__ == '__main__':
	main()
