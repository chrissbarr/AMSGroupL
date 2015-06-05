#!/usr/bin/env python

# system libraries
import time
import select
import sys
import socket
import os
import math


import rospy
from nav_msgs.msg import OccupancyGrid
import std_msgs.msg
import numpy as np

from OurModules import functions_localisation as loc

def map_broadcaster():
	pub = rospy.Publisher('mapBroadcaster',OccupancyGrid,queue_size=10)
	rospy.init_node('gridBroadcaster',anonymous=True)

#	loc.init()
	
	new_map = create_map()

	pub.publish(new_map)

	rate = rospy.Rate(2)

	
	key_pressed = False
	
	while key_pressed == False:
		
		key_pressed = select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []) # is key pressed?
		rate.sleep()
		
def add_obstacle(map, x, y):
	map.data[(map.info.width * y) + x] = 100
	

def create_map( ):
    test_map = OccupancyGrid()
    
    test_map.header.frame_id = 'world'
    
    test_map.info.resolution = 250 
    test_map.info.width = 5
    test_map.info.height = 5
    test_map.info.origin.position.x = 4000#loc.current_x #-test_map.info.width/2
    test_map.info.origin.position.y = 1000#loc.current_y #-test_map.info.height/2
    test_map.info.origin.position.z = 0 
    test_map.info.origin.orientation.x = 0.0 
    test_map.info.origin.orientation.y = 0.0 
    test_map.info.origin.orientation.z = 0.0 
    test_map.info.origin.orientation.w = 1.0
    test_map.data = [] 
    
    # create blank map of all 0's
    for i in range (0, test_map.info.width * test_map.info.height):
    	test_map.data.append(0)
    	
    # add in a few obstacles:
    add_obstacle(test_map,0,2)
    add_obstacle(test_map,1,2)
    add_obstacle(test_map,2,2)
    add_obstacle(test_map,3,2)
    add_obstacle(test_map,3,3)
    #for i in range (0, 9):
    #	add_obstacle(test_map,5,i)
    print test_map
    return test_map


if __name__=='__main__':
	try:
		map_broadcaster()
	except rospy.ROSInterruptException:
		pass
	
