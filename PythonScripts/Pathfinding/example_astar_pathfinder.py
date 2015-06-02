#!/usr/bin/env python

# system libraries
import time
import select
import sys
import socket
import os
import math
import collections
import heapq

import rospy 
import std_msgs.msg
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, Quaternion, PoseStamped, Pose
import tf

# NAVIGATION STATUS MESSAGES
nav_coords_reached = "COORDS REACHED"
nav_move_in_progress = "MOVING"
current_nav_string = ""

# DESIRED NAVIGATION COORDINATES
current_d_x = 0		#from desired pose topic
current_d_y = 0
current_d_th = 0

# setup publishing pose messages
pose_pub = rospy.Publisher('desiredPose', Pose, queue_size=10)
odom_broadcaster = tf.TransformBroadcaster()

class Graph:
	def __init__(self):
		self.edges = {}
	
	def neighbors(self, id):
		return self.edges[id]
		
class SquareGrid:
	def __init__(self, width, height):
		self.width = width
		self.height = height
		self.walls = []
		self.weights = {}
	
	def in_bounds(self, id):
		(x, y) = id
		return 0 <= x < self.width and 0 <= y < self.height
	
	def passable(self, id):
		return id not in self.walls
	
	def neighbors(self, id):
		(x, y) = id
		results = [(x+1, y), (x, y-1), (x-1, y), (x, y+1)]
		if (x + y) % 2 == 0: results.reverse() # aesthetics
		results = filter(self.in_bounds, results)
		results = filter(self.passable, results)
		return results
		
	def cost(self, a, b):
		return self.weights.get(b, 1)
		
class Queue:
	def __init__(self):
		self.elements = collections.deque()
	
	def empty(self):
		return len(self.elements) == 0
	
	def put(self, x):
		self.elements.append(x)
	
	def get(self):
		return self.elements.popleft()
		
class PriorityQueue:
	def __init__(self):
		self.elements = []
	
	def empty(self):
		return len(self.elements) == 0
	
	def put(self, item, priority):
		heapq.heappush(self.elements, (priority, item))
	
	def get(self):
		return heapq.heappop(self.elements)[1]

def pose_subscriber():
	# subscribe to ROS data updates
	DP = rospy.Subscriber("desiredPose", Pose, desired_pose_update)
	NS = rospy.Subscriber("nav_status", std_msgs.msg.String, nav_status_update)
	return (DP, NS)

def desired_pose_update(data):
	global current_d_x, current_d_y, current_d_th

	# read in position
	current_d_x = data.position.x
	current_d_y = data.position.y
	
	# read in orientation
	q = (
	    data.orientation.x,
	    data.orientation.y,
	    data.orientation.z,
	    data.orientation.w)
	    
	# convert orientation from quaternion to euler angles, read yaw
	euler = tf.transformations.euler_from_quaternion(q)
	current_d_th = euler[2]
	
def nav_status_update(message):
	global current_nav_string
	current_nav_string = message

def send_desired_pose(x, y, th):
    print("Requesting move to point: x: %.3f  y: %.3f  Th: %.1f") % (x, y, th) # print 2D pose data to terminal
    #publish pose data to ros topic
    msg = Pose()

    q = tf.transformations.quaternion_from_euler(0, 0, th)

    msg.position = Point(x,y,0)

    msg.orientation.x = q[0]
    msg.orientation.y = q[1]
    msg.orientation.z = q[2]
    msg.orientation.w = q[3]

    pose_pub.publish(msg)
		
def dijkstra_search(graph, start, goal):
	frontier = PriorityQueue()
	frontier.put(start, 0)
	came_from = {}
	cost_so_far = {}
	came_from[start] = None
	cost_so_far[start] = 0
	
	while not frontier.empty():
		current = frontier.get()
		
		if current == goal:
			break
		
		for next in graph.neighbors(current):
			new_cost = cost_so_far[current] + graph.cost(current, next)
			if next not in cost_so_far or new_cost < cost_so_far[next]:
				cost_so_far[next] = new_cost
				priority = new_cost
				frontier.put(next, priority)
				came_from[next] = current
	
	return came_from, cost_so_far

def reconstruct_path(came_from, start, goal):
	current = goal
	path = [current]
	while current != start:
		current = came_from[current]
		path.append(current)
	path.reverse()
	return path

def draw_tile(graph, id, style, width):
	r = "."
	if 'number' in style and id in style['number']: r = "%d" % style['number'][id]
	if 'point_to' in style and style['point_to'].get(id, None) is not None:
		(x1, y1) = id
		(x2, y2) = style['point_to'][id]
		if x2 == x1 + 1: r = '>'
		if x2 == x1 - 1: r = '<'
		if y2 == y1 + 1: r = 'v'
		if y2 == y1 - 1: r = '^'
	if 'start' in style and id == style['start']: r = "A"
	if 'goal' in style and id == style['goal']: r = "Z"
	if 'path' in style and id in style['path']: r = "@"
	if id in graph.walls: r = "#" * width
	return r

def draw_grid(graph, width=2, **style):
	for y in range(graph.height):
		for x in range(graph.width):
			print("%%-%ds" % width % draw_tile(graph, (x, y), style, width)),
		print()

def cell_to_coord(map,cell_x, cell_y):
	"""
	Converts grid x and y coordinates into real-world coordinates
	"""
	x = map.info.origin.position.x + (cell_x * map.info.resolution)
	y = map.info.origin.position.y + (cell_y * map.info.resolution)
	return (x, y)

def optimise_path(old_path):
	old_path_length = len(old_path)

	new_path = []
	new_path.append([])

	list_index = 0

	new_path[list_index].append(old_path[0][0])
	new_path[list_index].append(old_path[0][1])

	list_index+=1

	for i in range(1, old_path_length-2):
		if((old_path[i][0] == old_path[i-1][0] and old_path[i][0] == old_path[i+1][0]) or (old_path[i][1] == old_path[i-1][1] and old_path[i][1] == old_path[i+1][1])):
			# don't add to list
			print("cell skipped")
		else:
			new_path.append([])
			new_path[list_index].append(old_path[i][0])
			new_path[list_index].append(old_path[i][1])

			list_index+=1

	new_path.append([])
	new_path[list_index].append(old_path[old_path_length-1][0])
	new_path[list_index].append(old_path[old_path_length-1][1])

	return new_path



def rosmap_to_map(rosmap):
	global occupancy_grid
	width = rosmap.info.width
	height = rosmap.info.height
	
	new_map = SquareGrid(width,height)
	
	for i in range(height):
		for j in range(width):
			if(rosmap.data[(i * width) + j] == 100):
				new_map.walls.append((i,j))
	
	occupancy_grid = new_map
	
	g = occupancy_grid
	
	g_start = (1,1)
	g_goal = (3,8)
	
	came_from, cost_so_far = dijkstra_search(g, g_start, g_goal)
	draw_grid(g, width=2, point_to=came_from, start=g_start, goal=g_goal)
	print("\n\n")
	
	draw_grid(g, width=2, number=cost_so_far, start=g_start, goal=g_goal)
	print("\n\n")
	
	g_path = reconstruct_path(came_from, start=g_start, goal=g_goal)
	
	draw_grid(g, width=2, path=g_path)
	print("\n\n")
	
	print(g_path)
	print(len(g_path))

	optimised_path = optimise_path(g_path)

	print(optimised_path)
	print(len(optimised_path))

	waypoint_index = 0

	while(waypoint_index < len(optimised_path)-1):	#until we've reached the end of the path

		(d_x, d_y) = cell_to_coord(rosmap, optimised_path[waypoint_index][0], optimised_path[waypoint_index][1])

		if(current_d_x == d_x and current_d_y == d_y):
			# if the navigation system has reached the coordinate
			if(current_nav_string == nav_coords_reached):
				print("Waypoint %d has been reached.") % waypoint_index
				measure_temperature()
				if(waypoint_index < num_waypoints):
					waypoint_index += 1
				else:
					grid_finished = True
		else:
			#coordinate isn't set - set it
			print("Move to cell: [%d, %d]") % (optimised_path[waypoint_index][0], optimised_path[waypoint_index][1])
			send_desired_pose(d_x, d_y, -999)

		time.sleep(0.5)
	
	
def run():
	rospy.init_node('mapConverter',anonymous=True)
	# subscribe to ros occupancy-grid topic
	occupancy_grid_topic = rospy.Subscriber("mapBroadcaster", OccupancyGrid, rosmap_to_map)
	desired_pose_update, nav_status_update = pose_subscriber()
	
	key_pressed = False
	rate = rospy.Rate(1)
	
	while key_pressed == False:
		key_pressed = select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []) # is key pressed?
		rate.sleep()

	desired_pose_update.unregister()
	nav_status_update.unregister()
	occupancy_grid_topic.unregister()


if __name__=='__main__':
	try:
		run()
	except rospy.ROSInterruptException:
		pass
		
		
