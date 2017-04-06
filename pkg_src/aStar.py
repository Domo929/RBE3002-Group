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
	    #The open and closed sets
	    openset = set()
	    closedset = set()
	    #Current point is the starting point
	    current = start
	    #Add the starting point to the open set
	    openset.add(current)
	    #While the open set is not empty
	    while openset:
	        #Find the item in the open set with the lowest G + H score
	        current = min(openset, key=lambda o:o.findF(start,goal))
	        #If it is the item we want, retrace the path and return it
	        if current.x == goal.x and current.y == goal.y:
	            path = []
	            while current.parent:
	                path.append(current)
	                current = current.parent
	            path.append(current)
	            print(path[::-1])
	            return self.toPublishable(path[::-1])
	        #Remove the item from the open set
	        openset.remove(current)
	        #Add it to the closed set
	        closedset.add(current)
	        #Loop through the node's children/siblings
	        for node in self.findChildren(current):
	            #If it is already in the closed set, skip it
	            if node in closedset:
	                continue
	            #Otherwise if it is already in the open set
	            if node in openset:
	                #Check if we beat the G score 
	                new_g = current.findG(goal)
	                if node.gCost > new_g:
	                    #If so, update the node to have a new parent
	                    node.gCost = new_g
	                    node.parent = current
	            else:
	                #If it isn't in the open set, calculate the G and H score for the node
	                node.findF(start,goal)
	                #Set the parent to our current item
	                node.parent = current
	                #Add it to the set
	                openset.add(node)
	    #Throw an exception if there is no path
	    raise ValueError('No Path Found')

	

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

		x = -1
		y = -1
		while x < 2:
			y=-1
			while y < 2:
				if(not(x == 0 and y==0)): # do not check current node
					listOfNodes.append(self.nodes[currentX+x][currentY+y])
				y = y + 1
			x = x + 1

		return listOfNodes


	#start and goal are both point objects
	def runAStar(self,start,goal):


		startNode = self.nodes[int(start.x)][int(start.y)]
		endNode = self.nodes[int(goal.x)][int(goal.y)]
		currentNode = startNode
		print("before while")
		while(not(currentNode.x == goal.x and currentNode.y == goal.y)):
			currentNode = self.findBestNode(startNode, endNode, currentNode)
			self.path.append(currentNode)
			print("changeState")
			print(currentNode.x)
			#self.nodes[currentNode.x][currentNode.y].state=1
			print(currentNode.y)
			
			pubPathInfo = GridCells()
			pubPathInfo.header.frame_id = "map"
			pubPathInfo.cell_width =1
			pubPathInfo.cell_height=1
			pubPathInfo.cells = self.path
			self.pubPath.publish(pubPathInfo)

		print("Before while pub")
		while(1==1):
			pubPath.publish(pubPathInfo)


