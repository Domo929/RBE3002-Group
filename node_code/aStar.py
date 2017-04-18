#!/usr/bin/env python
import rospy, tf, math, time
from geometry_msgs.msg import Twist
from nav_msgs.msg import GridCells, Path
from Node import Node
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped

def aStarPathFinding(self, start, goal):
	self.path=[]
	if(start.state==-1):
		raise ValueError('Starting in a wall', start.x,start.y)
	openset = set()
	closedset = set()
	current = start
	current.gCost = 0
	openset.add(current)
	while openset:
		current = min(openset, key=lambda o:o.findF(start,goal))
		if current.x == goal.x and current.y == goal.y:
			path = []
			while current.parent:
				path.append(current)
				current = current.parent
			path.append(current)
			return self.toPublishable(path[::-1])
		openset.remove(current)
		closedset.add(current)
		for node in self.findChildren(current):
			if node in closedset or node.state == -1:
				continue
			if node in openset:
				new_f = current.findF(start, goal)
				if node.fCost > new_f:
					node.fCost = new_f
					node.parent = current
			else:
				current.gCost = node.gCost + 1
				node.findF(start,goal)
				node.parent = current
				openset.add(node)
		
	raise ValueError('No Path Found', start.x, start.y, goal.x, goal.y)
	print("Finished A*")
def toPublishable(self,listOfNodes):
	listOfPoints = []
	for node in listOfNodes:
		point = Point()
		point.x=node.x
		point.y=node.y
		listOfPoints.append(point)
	return listOfPoints	
def findChildren(self,currentNode):
	currentX=currentNode.x
	currentY=currentNode.y
	listOfNodes = []
	for x in range(-1,2):
		for y in range(-1,2):
			if(not(x == 0 and y == 0)):
				listOfNodes.append(self.nodes[currentX + x][currentY + y])
	return listOfNodes

#SETUP SERVICE