#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
import std_msgs.msg
import numpy as np

def map_broadcaster():
	pub = rospy.Publisher('mapBroadcaster',OccupancyGrid,queue_size=10)
	rospy.init_node('gridBroadcaster',anonymous=True)
	
	new_map = create_map()

	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		pub.publish(new_map)
		rate.sleep()

def create_map( ):
    test_map = OccupancyGrid()
    test_map.info.resolution = 1.0 
    test_map.info.width = 3
    test_map.info.height = 3
    test_map.info.origin.position.x = 0.0 
    test_map.info.origin.position.y = 0.0 
    test_map.info.origin.position.z = 0.0 
    test_map.info.origin.orientation.x = 0.0 
    test_map.info.origin.orientation.y = 0.0 
    test_map.info.origin.orientation.z = 0.0 
    test_map.info.origin.orientation.w = 1.0
    test_map.data = [0,1,0,0,1,0,0,0,0] 
    print test_map
    return test_map


if __name__=='__main__':
	try:
		map_broadcaster()
	except rospy.ROSInterruptException:
		pass
	
