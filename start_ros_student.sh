#!/bin/bash

# start_ros_student.sh - Version 0.1 - 2015-01-05
#
# The 'everything' starter. This script starts all of the core ROS
# processes on the MechBot (and associated) platforms. Screen is used to
# start each component in a separate terminal and to remain open but with
# display suppressed.
#
# -dm: Starts a new screen session in detached mode.
# -S <name>: Creates, or goes to, a screen session with 'name'.
# -X <command>: Sends the command to the specified screen session.
# -p 0: Selects the screen process.
# stuff <string>: Puts the string in as text.
# $(printf \\r): Ensures the command is executed (i.e. simulates 'Enter').
# sleep <int><unit>: Sleeps bash for the specified time.
#
# Created for the University of South Australia's MechBot platform.
# Copyright (c) 2013, Colin Smith & Russell Brinkworth. All rights reserved.
# Copyright (c) 2014 - 2015, Russell Brinkworth & Phil Skelton. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details at:
#
# http://www.gnu.org/licenses/gpl.html

# Shortcut variables.
ENTER="$(printf \\r)"

# Open all required screen sessions.
screen -dmS roscore
screen -dmS rosserial
screen -dmS ros_camera_streamer
screen -dmS rosvicon
screen -dmS LCD_command_parser
screen -dmS ROS_bridge
screen -dmS ROS_MJPEG_server
screen -dmS ROS_defaults
screen -dmS ROS_web_offloader

# setup screens for localisation and navigation scripts here

# Perform the necessary actions on each screen.
screen -S roscore -p 0 -X stuff "roscore $ENTER"
screen -S rosserial -p 0 -X stuff "rosrun rosserial_python serial_node.py /dev/ttyACM0 $ENTER"
screen -S ros_camera_streamer -p 0 -X stuff "source /home/odroid/catkin_ws/devel/setup.bash $ENTER"
screen -S ros_camera_streamer -p 0 -X stuff "rosrun ros_camera_streamer ros_camera_streamer $ENTER"
screen -S rosvicon -p 0 -X stuff "python /home/odroid/system_files/python/rosvicon.py $ENTER"
screen -S LCD_command_parser -p 0 -X stuff "python /home/odroid/system_files/python/LCD_command_parser.py $ENTER"
screen -S ROS_bridge -p 0 -X stuff "sleep 5 $ENTER" # sleep before starting web server to allow other things to start first
screen -S ROS_bridge -p 0 -X stuff "roslaunch rosbridge_server rosbridge_websocket.launch $ENTER"

screen -S ROS_web_offloader -p 0 -X stuff "python /home/odroid/system_files/python/rosweb_offloader.py $ENTER"



# The ROS_MJPEG_server has an issue with starting correctly. Therefore, we will start it once, kill it, and start it again.
screen -S ROS_MJPEG_server -p 0 -X stuff "sleep 20 $ENTER"
screen -S ROS_MJPEG_server -p 0 -X stuff "rosrun web_video_server web_video_server _port:=8181 $ENTER"
screen -S ROS_MJPEG_server -p 0 -X stuff "rosrun web_video_server web_video_server _port:=8181 $ENTER"
screen -S ROS_defaults -p 0 -X stuff "python /home/odroid/Documents/ROS_defaults.py $ENTER"

# start localisation and navigation scripts here
sleep 5

screen -dmS personality_screen
screen -S personality_screen -p 0 -X stuff "python /home/odroid/python_scripts/personality_background.py $ENTER"

screen -dmS object_avoidance_screen
screen -S object_avoidance_screen -p 0 -X stuff "python /home/odroid/python_scripts/object_avoidance_background.py $ENTER"

screen -dmS nav_pilot_screen
screen -S object_avoidance_screen -p 0 -X stuff "python /home/odroid/python_scripts/nav_pilot.py $ENTER"
