#!/bin/bash

# This script is designed to facilitate testing of the navigation systems

# When started, it will initialise a logging script, recording the following data:
# 	- Current Time
# 	- Current Position (x, y, th)
# 	- Target Position (x, y, th)

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

# start logging data
screen -dmS logging_screen
screen -S logging_screen -p 0 -X stuff "python /home/odroid/python_scripts/nav_logger.py $ENTER"

#start waypoint script
screen -dmS path_test_screen
screen -S path_test_screen -p 0 -X stuff "python /home/odroid/python_scripts/path_dijkstra_search.py $ENTER"

#start map script
screen -dmS map_gen_screen
screen -S map_gen_screen -p 0 -X stuff "python /home/odroid/python_scripts/path_dummy_map_broadcaster.py $ENTER"

sleep 140

# shutdown scripts
screen -S path_test_screen -p 0 -X stuff "$ENTER" 	#end the script
sleep 2
screen -S path_test_screen -p 0 -X stuff "exit $ENTER" 	#end the script

screen -S map_gen_screen -p 0 -X stuff "$ENTER" 	#end the script
sleep 2
screen -S map_gen_screen -p 0 -X stuff "exit $ENTER" 	#end the script

sleep 2

screen -S logging_screen -p 0 -X stuff "$ENTER" #end the script
sleep 2
screen -S logging_screen -p 0 -X stuff "exit $ENTER" #end the script

# make sure object avoidance is enabled
screen -S object_avoidance_screen -p 0 -X stuff "$ENTER"
sleep 1
screen -S object_avoidance_screen -p 0 -X stuff "python /home/odroid/python_scripts/object_avoidance_background.py T $ENTER"

echo Exiting Pathfinding Test
