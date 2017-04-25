#!/usr/bin/env python
import rospy, tf, math, time
from geometry_msgs.msg import Twist
from nav_msgs.msg import GridCells, Path
from Node import Node
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped

class FindFrontiers:

	def __init__(self):
		self = self

	def findFrontierRegionCentriods(self,mapOG,threshold):
		print "Find Region Centriods started"
		frontierRegions = self.findFrontierRegions(mapOG,threshold) #returns a list of lists of points
		print "Frontier regions found"
		centriods = []

		for region in frontierRegions:
			print "in for loop"
			tempSumX = 0
			tempSumY = 0

			for point in region:
				tempSumX += point.x
				tempSumY += point.y

			centriod = Point()

			centriod.y = int(tempSumY/float(len(region))) # find average of x and y
			centriod.x = int(tempSumX/float(len(region))) # set this as the centriod and add to the list
			centriods.append(centriod)

		return centriods
	def findFrontierRegions(self,mapOG,threshold):
		print "Finding Frontier Regions"
		frontierRegions = []
		listOfFrontiers = self.findFrontiers(mapOG)
		print "list of frontier points found"
		frontierRegions.append([listOfFrontiers.pop()]) #start the first region with the first frontier point

		#for counter in range(len(listOfFrontiers)):
		while len(listOfFrontiers) > 0:
			print ("in for loop 2",len(listOfFrontiers))
			#rospy.sleep(rospy.Duration(3))
			frontierPoint = listOfFrontiers.pop()
			hasBeenAdded=False
			ii = 0
			for region in frontierRegions:
				#print "in for loop 3"
				if(hasBeenAdded):
					break
				for existingPoint in region:
					#print "in for loop 4"
					if(not(existingPoint.x == frontierPoint.x and existingPoint.y == frontierPoint.y)):
						if(self.isAdjacent(existingPoint,frontierPoint)): #if the point is adjacent to any other points add it
							frontierRegions[ii].append(frontierPoint) #to that regions list of points
						else:
							frontierRegions.append([frontierPoint])  #if not then create a new region
						hasBeenAdded=True
						break
						
				ii +=1
 
 		for region in frontierRegions: #remove any regions that are smaller than the width of the robot
			if(len(region)<threshold): #because they are insignifanct and do not need to be explored
				frontierRegions.remove(region)

		return frontierRegions
	def findFrontiers(self, mapOG):
		height = mapOG.info.height
		width = mapOG.info.width
		data = mapOG.data
		listOfFrontiers = []
		mapOfFrontiers = []
		pubFrontier = rospy.Publisher('/mapData/Frontier',GridCells,queue_size=10)
		
		#pathCell.cells = self.convertToGridScaling(pathReturned,resolution)
		#the logic here is that any known open cell adjacent
		#to an unknown cell must be a frontier
		for y in range(height):
			for x in range(width):
				currentDataPoint = data[x+y*(width)]
				if(currentDataPoint < 50 and not(currentDataPoint==-1)):
					if(self.isAdjacentToOpen(x,y,data,width)):
						point = Point()
						point.x = x
						point.y = y

						listOfFrontiers.append(point)

		pathCell = GridCells()
		pathCell.header.frame_id = "map"
		pathCell.cell_width =mapOG.info.resolution
		pathCell.cell_height=mapOG.info.resolution
		pathCell.cells = self.convertToGridScaling(listOfFrontiers,mapOG.info.resolution)

		while(not(rospy.is_shutdown())):
			print "publishing"
			pubFrontier.publish(pathCell)

		return listOfFrontiers
	def isAdjacentToOpen(self,x,y,data,width):
		for yAdd in range(-1,2):
			for xAdd in range(-1,2):
				xCheck = x+xAdd
				yCheck = y+yAdd


				if(data[xCheck+yCheck*(width)]==-1 and not(xAdd == 0 and yAdd==0)):
					return True
		return False
	def isAdjacent(self,existingPoint,frontierPoint):
		for yAdd in range(-1,2):
			for xAdd in range(-1,2):
				xCheck = existingPoint.x+xAdd
				yCheck = existingPoint.y+yAdd

				if(xCheck == frontierPoint.x and yCheck == frontierPoint.y and not(xAdd == 0 and yAdd == 0)):
					return True
		return False

	def convertToGridScaling(self,arr,resolution):

	    for i in range(0, len(arr)):
	        arr[i].x=arr[i].x*resolution-12.26
	        arr[i].y=arr[i].y*resolution-13.87
	    return arr