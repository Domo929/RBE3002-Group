#!/usr/bin/env python
import rospy, tf, math, time
from geometry_msgs.msg import Twist
from nav_msgs.msg import GridCells, Path
from Node import Node
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped

class aStar:
	
	pubPath = rospy.Publisher('/mapData/Path',GridCells,queue_size=10)
	pubBuffer = rospy.Publisher('/mapData/Buffer',GridCells,queue_size=10)

	def __init__(self, nodeList):
		
		self.nodes = nodeList
		
		self.path = []

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
	
	def addBuffer(self, radius):
		
		addedNodes = []
		
		for x in self.nodes:
			for node in x:
				if(node.state == -1):
					for child in self.findChildrenRadius(node,radius):
						if(child.state != -1):
							addedNodes.append(child)
		
		print("Buffer was set")
		
		for node in addedNodes:
			node.state=-1
		
		return self.toPublishable(addedNodes)
	
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

	def convertToGridScaling(self,arr,resolution):

	    for i in range(0, len(arr)):
	        arr[i].x=arr[i].x*resolution + 0.1
	        
	        arr[i].y=arr[i].y*resolution + 0.2
	    return arr

	def returnPathWaypoints(self, start, end, inputMap, resolution):
		
		print("Start Path Waypoints")
		
		self.nodes = inputMap
		
		path=[]
		
		print "Middle Middle Path waypoints"
		
		pathReturned = self.aStarPathFinding(start,end)

		#create gridcell output for path
		pathCell = GridCells()
		
		pathCell.header.frame_id = "map"
		
		pathCell.cell_width =resolution
		pathCell.cell_height=resolution
		
		pathCell.cells = self.convertToGridScaling(pathReturned,resolution)

	    #create list of waypoints
		listOfWaypoints = []
		print "Middle Path waypoints"
		for x in range(1, len(pathReturned)-1):  
			
			currentNode = pathReturned[x]
			
			nextNode = pathReturned[x+1]
			
			previousNode = pathReturned[x-1]

			angleToNextPoint = math.atan2((nextNode.y-currentNode.y),(nextNode.x-currentNode.x))
			
			prevToCurrentAngle = math.atan2((currentNode.y-previousNode.y),(currentNode.x-previousNode.x))

			if(prevToCurrentAngle!=angleToNextPoint):
				
				quaternion = tf.transformations.quaternion_from_euler(0,0,angleToNextPoint)

				tempPose = PoseStamped()
				
				tempPose.header.frame_id = 'map'
				
				tempPose.pose.position.x=currentNode.x
				tempPose.pose.position.y=currentNode.y
				
				tempPose.pose.orientation.x = quaternion[0]
				tempPose.pose.orientation.y = quaternion[1]
				tempPose.pose.orientation.z = quaternion[2]
				tempPose.pose.orientation.w = quaternion[3]

				listOfWaypoints.append(tempPose)
		
		print("After For Loop")

		#Last Waypoint
		tempPose = PoseStamped()
		tempPose.header.frame_id = 'map'
		tempPose.pose.position.x= pathReturned[-1].x
		tempPose.pose.position.y= pathReturned[-1].y
		listOfWaypoints.append(tempPose)

		waypointsToPublish = Path()
		waypointsToPublish.header.frame_id = 'map'
		waypointsToPublish.poses = listOfWaypoints
		
		for waypoint in listOfWaypoints:
			fancyString="{}, {}, {}"
			print("Waypoint")
			print(fancyString.format(waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.orientation.z))
	    

		return waypointsToPublish