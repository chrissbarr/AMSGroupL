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
from nav_msgs.msg import OccupancyGrid

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
		
def breadth_first_search(graph, start, goal):
	frontier = Queue()
	frontier.put(start)
	came_from = {}
	came_from[start] = None
	
	while not frontier.empty():
		current = frontier.get()
		
		if current == goal:
			break
		
		for next in graph.neighbors(current):
			if next not in came_from:
				frontier.put(next)
				came_from[next] = current
	
	return came_from

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
	
	

def run():
	rospy.init_node('mapConverter',anonymous=True)
	# subscribe to ros occupancy-grid topic
	occupancy_grid_topic = rospy.Subscriber("mapBroadcaster", OccupancyGrid, rosmap_to_map)
	
	key_pressed = False
	rate = rospy.Rate(1)
	
	while key_pressed == False:
		key_pressed = select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []) # is key pressed?
		rate.sleep()


if __name__=='__main__':
	try:
		run()
	except rospy.ROSInterruptException:
		pass
		
		
