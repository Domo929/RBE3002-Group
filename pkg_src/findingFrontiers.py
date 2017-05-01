#!/usr/bin/env python
""" IMPORTS """
import rospy, tf, math, time
from nav_msgs.msg import GridCells, Path
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped

class FindFrontiers:

	def __init__(self):
		self = self

	"""
	Takes in a map and a threshold, and finds a list of frontier centroids

		Params: self, mapOG, threshold; self is the current object
										mapOG is the OccupancyGrid for the map
										threshold is what determines whether a frontier region is big enough be worth exploring
		Returns: centroids; a list of frontier centroids derived from the current map
	"""
	def findFrontierRegionCentriods(self,mapOG,threshold):
		print "Find Region Centriods started"
		frontierRegions = self.findFrontierRegions(mapOG,threshold) #returns a list of lists of points
		pubCents = rospy.Publisher('/mapData/Cents',GridCells,queue_size=10)
		print "Frontier regions found"
		centriods = []

		for region in frontierRegions:
			tempSumX = 0
			tempSumY = 0
			for point in region:
				tempSumX += point.x
				tempSumY += point.y

			centriod = Point()

			centriod.y = tempSumY/float(len(region)) # find average of x and y
			centriod.x = tempSumX/float(len(region)) # set this as the centriod and add to the list
			centriods.append(centriod)

		centsCell = GridCells()
		centsCell.header.frame_id = "map"
		centsCell.cell_width = mapOG.info.resolution
		centsCell.cell_height= mapOG.info.resolution
		centsCell.cells = centriods
		print "Publish Centeriods"
		pubCents.publish(centsCell)
		return centriods
	"""
	Takes in a map and a threshold, and finds a list of frontier regions
		
	"""
	def findFrontierRegions(self,mapOG,threshold):
		print "Finding Frontier Regions"
		frontierRegions = []
		listOfFrontiers = self.findFrontiers(mapOG)
		print "list of frontier points found"
		frontierRegions.append([listOfFrontiers.pop()]) #start the first region with the first frontier point

		pubRegions = rospy.Publisher('/mapData/Regions',GridCells,queue_size=10)
		regionPoints=[]:
		while len(listOfFrontiers) > 0:
			frontierPoint = listOfFrontiers.pop()
			hasBeenAdded=False
			ii = 0
			for region in frontierRegions:
				for existingPoint in region:
					if(self.isAdjacent(existingPoint,frontierPoint) and not(hasBeenAdded)): #if the point is adjacent to any other points add it
						frontierRegions[ii].append(frontierPoint) #to that regions list of points
						hasBeenAdded=True	

				ii +=1

			if(not(hasBeenAdded)): 
				print "New Region Added"
				frontierRegions.append([frontierPoint])  #if not then create a new region

		regionCell = GridCells()
		regionCell.header.frame_id = "map"
		regionCell.cell_width = mapOG.info.resolution
		regionCell.cell_height= mapOG.info.resolution
		regionCell.cells = frontierRegions[0]
		print("Frontier #: ", len(regionPoints))
		print("Region #: ", len(frontierRegions))
		print "publishing: Regions"
		print regionCell.cells
		pubRegions.publish(regionCell)
 
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
		pathCell.cells = self.convertGridToWorld(listOfFrontiers,mapOG.info.resolution, mapOG.info.origin)

		#while(not(rospy.is_shutdown())):
		print "publishing: Frontier"
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
	def isAdjacent(self,point1,point2):
		# for yAdd in range(-1,2):
		# 	for xAdd in range(-1,2):
		# 		# xCheck = point1.x+xAdd
		# 		# yCheck = point1.y+yAdd
		
		if(math.sqrt((point1.x-point2.x)**2+(point1.y-point2.y)**2) < .1):
			#print (math.sqrt((point1.x-point2.x)**2+(point1.y-point2.y)**2))
			return True
		return False
	def convertGridToWorld(self,arr,resolution, origin):
	    for i in range(0, len(arr)):
	        arr[i].x=arr[i].x*resolution+origin.position.x+(0.5*resolution)
	        arr[i].y=arr[i].y*resolution+origin.position.y+(0.5*resolution)
	    return arr