#!/bin/bash

# This script is designed to facilitate testing of the localisation systems

# When started, it will initialise a logging script, recording the following data:
# 	- Current Time
# 	- Current Position (x, y, th)
# 	- Target Position (x, y, th)
#	- Odometry Localisation Position (x, y, th)

# A basic waypoint script will then be initialised, moving the mechbot through a triangular series of waypoints repeatedly.

# By plotting the recorded data, it should be possible to see the robot's path as it moves from one waypoint to the next, and to determine the accuracy of the navigation system.

# Shortcut variables.
ENTER="$(printf \\r)"



# make sure object avoidance is disabled
screen -S object_avoidance_screen -p 0 -X stuff "$ENTER"
sleep 1
screen -S object_avoidance_screen -p 0 -X stuff "python /home/odroid/python_scripts/object_avoidance_background.py F $ENTER"

# make sure nav pilot is running correctly
screen -S nav_pilot_screen -p 0 -X stuff "$ENTER"
sleep 1
screen -S nav_pilot_screen -p 0 -X stuff "python /home/odroid/python_scripts/nav_pilot.py F $ENTER"

# start localisation script correctly
screen -S loc_fusion -p 0 -X stuff "$ENTER" # kill running script (if any)
sleep 1
screen -S loc_fusion -p 0 -X stuff "python /home/odroid/python_scripts/loc_fusion.py F $ENTER"

# start logging data
screen -dmS logging_screen
screen -S logging_screen -p 0 -X stuff "python /home/odroid/python_scripts/nav_logger.py $ENTER"

#start waypoint script
screen -dmS waypoint_test_screen
screen -S waypoint_test_screen -p 0 -X stuff "python /home/odroid/python_scripts/nav_test.py $ENTER"

sleep 75

# shutdown scripts
screen -S waypoint_test_screen -p 0 -X stuff "$ENTER" 	#end the script
sleep 2
screen -S waypoint_test_screen -p 0 -X stuff "exit $ENTER" 	#end the script

sleep 2

screen -S logging_screen -p 0 -X stuff "$ENTER" #end the script
sleep 2
screen -S logging_screen -p 0 -X stuff "exit $ENTER" #end the script

echo Exiting Localisation Test
