#!/usr/bin/env python

import rospy, tf, math
# Add additional imports for each of the message types use
from geometry_msgs.msg import Twist
from nav_msgs.msg import GridCells
import Node

class aStar:
	pubFrontier = rospy.Publisher('/mapData/Frontier', GridCells,queue_size=10)
	pubPath = rospy.Publisher('/mapData/Path',GridCells,queue_size=10)
	pubUnknown = rospy.Publisher('/mapData/Unknown',GridCells,queue_size=10)

	def __init__(self, nodeList):
		self.nodes = nodeList
		self.path = []
		self.frontier = []
		self.unknown = []

	#start and goal are both point objects
	def runAStar(start,goal):
		startNode = self.nodes[start.x][start.y]
		endNode = self.nodes[goal.x][goal.y]
		currentNode = startNode
		
		while(not(currentNode.x == goal.x and currentNode.y == goal.y)):
			currentNode = findBestNode(startNode, endNode, currentNode)
			path.append(currentNode)



	def findBestNode(startNode,endNode,currentNode):
		maxF = 0
		currentX=currentNode.x
		currentY=currentNode.y
		maxPoint = Point()
		maxPoint.x=-1
		maxPoint.y=-1 #if there is an error it will return -1

		x = -1
		y = -1
		while x < 2:
			y=-1
			while y < 2:
				if(not(x == 0 and y==0)): # do not check current node
					nextNode = self.nodes[currentX+x][currentY+y]
					tempF = nextNode.findF(startNode,endNode)
					if(tempF>maxF and nextNode.type == "Frontier"):
						maxF=tempF
						maxPoint.x=x
						maxPoint.y=y
				y = y + 1
			x = x + 1
		return maxPoint

