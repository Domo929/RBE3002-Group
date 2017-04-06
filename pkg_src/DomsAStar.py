#!/usr/bin/env python

from Node import Node
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point	
import numpy as np

def findGrid(msg):
	global obstacles

	width = msg.info.width
	height = msg.info.height
	index = msg.data

	for x in range(0, width):
		for y in range(0, height):
			val = index[y*width+x]
			#print val
			if val == 100:
				obstacles[x][y] = 1
			elif val == 0:
				obstacles[x][y] = 0
				

def runAStar(start, goal):
	global obstacles
	frontier = PriorityQueue()
	frontier.put(start, 0)
	currentNode = start
	came_from[start] = None
	cost_so_far[start] = 0

	while not frontier.empty():
		currentNode = frontier.get()

		if currentNode.x == goal.x and currentNode.y == goal.y:
			break
			
		





if __name__ == '__main__':
	global obstacles

	map_sub = rospy.Subscriber('/map', OccupancyGrid, findGrid, queue_size=1)