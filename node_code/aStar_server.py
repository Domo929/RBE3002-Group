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
	self.nodes = inputMap
	path=[]
	pathReturned = self.aStarPathFinding(start,end)
#create gridcell output for path
	pathCell = GridCells()
	pathCell.header.frame_id = "map"
	pathCell.cell_width =resolution
	pathCell.cell_height=resolution
	pathCell.cells = self.convertToGridScaling(pathReturned,resolution)
#create list of waypoints
	listOfWaypoints = []
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

def handle_aStar(req):
	start=req.start
	end=req.end
	return aStarResponse()
	
def aStar_server():
	rospy.init_node('aStar_server')
	s = rospy.Service('aStar', aStarCompute, handle_aStar)
	rospy.spin()

if __name__ == "__main__":
	subMap = rospy.Subscriber('/map',OccupancyGrid,saveMap,queue_size=10)
	aStar_server()




#SETUP SERVICE