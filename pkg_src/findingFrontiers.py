#!/usr/bin/env python
import rospy, tf, math, time
from geometry_msgs.msg import Twist
from nav_msgs.msg import GridCells, Path
from Node import Node
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped

class findFrontiers:

	def __init__(self):
		self = self
		
	def findFrontierRegionCentriods(self,map,threshold):
		frontierRegions = findFrontierRegions(map,threshold) #returns a list of lists of points
		centriods = []

		for region in frontierRegions:
			tempSumX = 0
			tempSumY = 0

			for point in region:
				tempSumX += point.x
				tempSumY += point.y

			centiord = Point()

			centriod.y = int(tempSumY/float(len(region))) # find average of x and y
			centriod.x = int(tempSumX/float(len(region))) # set this as the centriod and add to the list
			centriods.append(centriod)

		return centriods

	def findFrontierRegions(self,map,threshold):
		frontierRegions = []
		listOfFrontiers = findFrontiers(map)

		frontierRegions.append([listOfFrontiers.pop()]) #start the first region with the first frontier point

		for counter in len(listOfFrontiers):
			frontierPoint = listOfFrontiers.pop()
			for region in frontierRegions:
				for existingPoint in region:
					if(isAdjacent(existingPoint,frontierPoint)): #if the point is adjacent to any other points add it
						region.append(frontierPoint)			 #to that regions list of points
					else:
						frontierRegions.append([frontierPoint])  #if not then create a new region
 
		for region in frontierRegions: #remove any regions that are smaller than the width of the robot
			if(len(region)<threshold): #because they are insignifanct and do not need to be explored
				frontierRegions.remove(region)

		return frontierRegions

	
	def findFrontiers(self, map):
		height = map.info.height
		width = map.info.width
		data = map.data

		listOfFrontiers = []
		point = Point()

		#the logic here is that any known open cell adjacent
		#to an unknown cell must be a frontier
		for y in range(height):
			for x in range(width):
				currentDataPoint = data[x+y(width)]
				if(currentDataPoint < 50):
					if(isAdjacentToOpen(x,y,data,width)):
						point = Point()
						point.x = x
						point.y = y

						listOfFrontiers.append(point)

		return listOfFrontiers


	def isAdjacentToOpen(self,x,y,data,width):
		for yAdd in range(-1,1):
			for xAdd in range(-1,1):
				xCheck = x+xAdd
				yCheck = y+yAdd

				if(data[xCheck+yCheck(width)==-1 and not(x == 0 and y==0)):
					return True
		return False