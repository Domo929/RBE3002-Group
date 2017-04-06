#!/usr/bin/env python

import rospy, tf, math, time
# Add additional imports for each of the message types use
from geometry_msgs.msg import Twist
from nav_msgs.msg import GridCells
from Node import Node
from geometry_msgs.msg import Point

class aStar:
	pubFrontier = rospy.Publisher('/mapData/Frontier', GridCells,queue_size=10)
	pubPath = rospy.Publisher('/mapData/Path',GridCells,queue_size=10)
	pubUnknown = rospy.Publisher('/mapData/Unexplored',GridCells,queue_size=10)

	def __init__(self, nodeList):
		self.nodes = nodeList
		self.path = []
		self.frontier = []
		self.unknown = []

	def aStar(self, start, goal):
	    openset = set()
	    closedset = set()
	    current = start
	    openset.add(current)
	    while openset:
	        current = min(openset, key=lambda o:o.gCost + o.hCost)
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
	        	if node in closedset:
	        		continue
	        	if node in openset and not(node.state==-1):
	        		node.cost=current.cost + 1
	                new_g = current.gCost + node.cost
	                if node.gCost > new_g:
	                    node.gCost = new_g
	                    node.parent = current
	                else:
		            	node.gCost = current.gCost + node.cost
		            	node.hCost = self.findManhatten(node,goal)
		                node.parent = current
		                openset.add(node)
	    raise ValueError('No Path Found')


	def findManhatten(self, point1, point2):
		return abs(point1.x-point2.x) + abs(point1.y-point2.y)

	def toPublishable(self,listOfNodes):
		listOfPoints = []
		for node in listOfNodes:
			point = Point(node.x, node.y)
			listOfPoints.append(point)
		return listOfPoints

	def findChildren(self,currentNode):
		listOfNodes=[]
		for x in range(-1, 2):
			for y in range(-1, 2):
				if(not(x == 0 and y ==0) and not(self.nodes[currentNode.x+x][currentNode.y+y]==0)):
					listOfNodes.append(self.nodes[currentNode.x+x][currentNode.y+y])
		return listOfNodes

		# currentX=currentNode.x
		# currentY=currentNode.y
		# listOfNodes = []

		# x = -1
		# y = -1
		# while x < 2:
		# 	y=-1
		# 	while y < 2:
		# 		if(not(x == 0 and y==0)):
		# 			listOfNodes.append(self.nodes[currentX+x][currentY+y])
		# 		y = y + 1
		# 	x = x + 1

		# return listOfNodes


	#start and goal are both point objects
	def runAStar(self,start,goal):


		startNode = self.nodes[int(start.x)][int(start.y)]
		endNode = self.nodes[int(goal.x)][int(goal.y)]
		currentNode = startNode
		while(not(currentNode.x == goal.x and currentNode.y == goal.y)):
			currentNode = self.findBestNode(startNode, endNode, currentNode)
			self.path.append(currentNode)
			pubPathInfo = GridCells()
			pubPathInfo.header.frame_id = "map"
			pubPathInfo.cell_width =1
			pubPathInfo.cell_height=1
			pubPathInfo.cells = self.path
			self.pubPath.publish(pubPathInfo)
		while(1==1):
			pubPath.publish(pubPathInfo)


