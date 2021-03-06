#!/usr/bin/env python

# system libraries
import time
import sys
import select
import socket

# ROS
import rospy
import std_msgs.msg
from geometry_msgs.msg import Point, Quaternion, PoseStamped
import tf

object_name = '12Lgroup' # name of the object you are looking for on the Vicon system

delay = 0.2 # update rate for main loop (s)

v_pose = [-999.0, -999.0, 0.0] # vicon pose data (x, y, yaw)

rospy.init_node("vicon_to_pose", anonymous=False) # name the script on the ROS network


# setup publishing pose messages
pose_pub = rospy.Publisher('viconPose', PoseStamped, queue_size=10)
odom_broadcaster = tf.TransformBroadcaster()

# subroutines

def vicon_update(data):
    words = data.data.split() # raw vicon data is in the form (x, y, z, pitch, roll, yaw)
    if words[0] == object_name: # only get data if it has the name you are looking for
        v_pose[0] = float(words[1])/1000 # x, convert to m
        v_pose[1] = float(words[2])/1000 # y 
        v_pose[2] = float(words[6]) # yaw, keep in rad in range (pi, -pi)

        publish_pose(v_pose[0],v_pose[1],v_pose[2])
	
def subscriber():
    # subscribe to ROS data updates
    VS = rospy.Subscriber("/mechbot_12/get/vicon_pose", std_msgs.msg.String, vicon_update)
    return (VS)

def publish_pose(x, y, th):
        #create pose message
        msg = PoseStamped()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'world'

        q = tf.transformations.quaternion_from_euler(0, 0, th)

        msg.pose.position = Point(x,y,0)

        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        pose_pub.publish(msg)

# main program

def main():
    (ros_update_1) = subscriber() # subscribe to ROS topics
    time.sleep(1) # give sufficient time to start getting data
    
    key_pressed = False
	
    while key_pressed == False:
        loop_start = time.time() # get loop time at start for loop rate calculations
        #print("x: %.3f  y: %.3f  Th: %.1f") % (v_pose[0], v_pose[1], v_pose[2]) # print 2D pose data to terminal
	
        loop_sleep = delay - (time.time() - loop_start) # if loop delay too low then will print data faster than updates are recieved
        if loop_sleep > 0:
            time.sleep(loop_sleep)
        key_pressed = select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []) # is key pressed?

    ros_update_1.unregister() # unregister from update 1
    print "unregistered"
    sys.exit()


if __name__ == '__main__':
    main()

