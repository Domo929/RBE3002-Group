#!/usr/bin/env python
import rospy, tf, math, time
from geometry_msgs.msg import Twist
from nav_msgs.msg import GridCells
from Node import Node
from geometry_msgs.msg import Point

class aStar:
	pubPath = rospy.Publisher('/mapData/Path',GridCells,queue_size=10)
	pubBuffer = rospy.Publisher('/mapData/Buffer',GridCells,queue_size=10)
	def __init__(self, nodeList):
		self.nodes = nodeList
		self.path = []

	def aStarPathFinding(self, start, goal):
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
					#print(current.x, current.y)
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
		raise ValueError('No Path Found')
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

	def addBuffer(self, radius):
		addedNodes = []
		for x in self.nodes:
			for node in x:
				if(node.state == -1): # if the node is a wall
					for child in self.findChildrenRadius(node,radius):
						#child.state = -1
						if(child.state != -1):
							addedNodes.append(child)
		print("Buffer was set")
		
		for node in addedNodes:
			node.state=-1

		pubPathInfo = GridCells()
		pubPathInfo.header.frame_id = "map"
		pubPathInfo.cell_width =1
		pubPathInfo.cell_height=1
		pubPathInfo.cells = self.toPublishable(addedNodes)
		self.pubBuffer.publish(pubPathInfo)
		return self.nodes


	def findChildrenRadius(self, currentNode, radius):
		width = len(self.nodes)
		height = len(self.nodes[0])
		currentX=currentNode.x
		currentY=currentNode.y
		listOfNodes = []
		searchX=0
		searchY=0

		for x in range(-radius, radius + 1):
			for y in range(-radius , radius + 1):
				searchX=currentX+x
				searchY=currentY+y
				notSelf=not(x==0 and y==0)
				notLargerThanGrid=not(searchX>=width) and not(searchY>=height)
				notLessThanGrid=not(searchX<0) and not(searchY<0)

				if(notSelf and notLargerThanGrid and notLessThanGrid):
					listOfNodes.append(self.nodes[searchX][searchY])

		return listOfNodes
