#!/usr/bin/python

"""
ROS_defaults.py - Version 0.1 2015-02-11

This program sets defaults for devices and systems on the ROS network

Created for the University of South Australia's MechBot platform.
Copyright (c) 2015, Russell Brinkworth. All rights reserved.

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

import rospy
import socket
import subprocess

from std_msgs.msg import Int16, Int16MultiArray
from sensor_msgs.msg import Image

def ROS_defaults(frame_width, frame_height, frame_rate, flip, volume):
    # MANDATORY: Create a new ROS node, must be a unique node_name specified.
    rospy.init_node("ROS_defaults", anonymous=False)
    
    rospy.loginfo("Latching onto ../get/camera_stream topic...")
    rospy.wait_for_message("/%s/get/camera_stream" % socket.gethostname(), Image, timeout=60)

    send_camera_defaults(frame_width, frame_height, frame_rate, flip)

    subprocess.call(["amixer -D pulse sset Master %d%%" % (volume)], shell=True) # set the volume

def send_camera_defaults(frame_width, frame_height, frame_rate, flip):
    
    rospy.loginfo("Creating necessary publishers...")
    camera_resolution_publisher = rospy.Publisher("/%s/set/camera_resolution" % socket.gethostname(), Int16MultiArray, queue_size=10)
    camera_frame_rate_publisher = rospy.Publisher("/%s/set/camera_frame_rate" % socket.gethostname(), Int16, queue_size=10)
    camera_flip_publisher = rospy.Publisher("/%s/set/camera_flip" % socket.gethostname(), Int16, queue_size=10)
    rospy.sleep(1) # sleep to ensure all publishers have time to be created

    camera_resolution = Int16MultiArray()
    camera_resolution.data = [frame_width, frame_height]
    camera_resolution_publisher.publish(camera_resolution) # publish initial camera resolution

    camera_frame_rate = Int16()
    camera_frame_rate.data = frame_rate
    camera_frame_rate_publisher.publish(camera_frame_rate) # publish initial camera framerate

    camera_flip = Int16()
    camera_flip.data = flip
    camera_flip_publisher.publish(camera_flip) # publish initial camera framerate

    rospy.loginfo("Setting default camera values:")
    rospy.loginfo("Camera resolution: [%d, %d]" % (camera_resolution.data[0], camera_resolution.data[1]))
    rospy.loginfo("Camera frame rate: [%d]" % camera_frame_rate.data)


if __name__ == '__main__':

    try:

        ROS_defaults(frame_width=432, frame_height=240, frame_rate=25, flip=2, volume=3)
	#ROS_defaults(frame_width=640, frame_height=480, frame_rate=25, flip=2, volume=40)

    except rospy.ROSInterruptException:

        rospy.loginfo("Node shutting down...")

# camera flip values
# 0: top left = bottom left
# 1: top left = top right
# -1: top left = bottom right
# else: no change (eg 2)
