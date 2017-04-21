#!/usr/bin/env python 

#The messages we use, and the rospy import
import rospy
from nav_msgs.msg import OccupancyGrid, GridCells
from geometry_msgs.msg import Point	


#Set values that will be matched with OccupancyGrid vals to determine Node type
frontier_val   = -1
explored_val   = -2
unexplored_val = -3
obstacles_val  = -4
path_val       = -5

#Flag to avoud NameError Exceptions when it tries to run without recieving a map first
haveFirstMap = False

#Takes in an Occupancy Grid and sets global variables for key information
def readOccGrid(msg):
	global width
	global height
	global resolution
	global index

	width = msg.info.width
	height = msg.info.height
	resolution = msg.info.resolution
	index = msg.data


if __name__ == "__main__":

	#Starts and labels the ROS Node
	rospy.init_node('visualize_map')

	#The Global Variables that will be referenced
	global width
	global height
	global resolution
	global index

	#Publishers
	frontier_pub   = rospy.Publisher('mapData/frontier',     GridCells, queue_size=10)
	explored_pub   = rospy.Publisher('mapData/explored',     GridCells, queue_size=10)
	unexplored_pub = rospy.Publisher('mapData/unexplored',   GridCells, queue_size=10)
	obstacles_pub  = rospy.Publisher('mapData/obstacles',    GridCells, queue_size=10)
	path_pub       = rospy.Publisher('mapData/shortestPath', GridCells, queue_size=10)

	#Subscriber(s)
	map_sub        = rospy.Subscriber('/map', OccupancyGrid, readOccGrid, queue_size=10)

	#Wait to make sure we don't attempt to iterate over empty items
	while not haveFirstMap:
		pass

	#Do this till we tell you to shove it
	while not rospy.is_shutdown():

		#Empty the list each iteration
		frontier_list   = []
		explored_list   = []
		unexplored_list = []
		obstacles_list  = []
		path_list       = []

		#Nested For Loops to iterate over the 1 D Array
		for x in xrange(0, width):
			for y in xrange(0, height):

				#The val at that x,y position
				val = index[y*width+x]

				#match value and add a Point to that map
				if val == frontier_val:
					frontier_list.append(Point(resolution*x, resolution*y, 0))

				elif val == explored_val:
					explored_list.append(Point(resolution*x, resolution*y, 0))

				elif val == unexplored_val:
					unexplored_list.append(Point(resolution*x, resolution*y, 0))

				elif val == obstacles_val:
					obstacles_list.append(Point(resolution*x, resolution*y, 0))

				elif val == path_val:
					path_list.append(Point(resolution*x, resolution*y, 0))

		#Creates each Gridcells object to publish
		frontierGC = GridCells()
		frontierGC.header.frame_id = 'Frontier'
		frontierGC.cell_width = resolution
		frontierGC.cell_height = resolution
		frontierGC.cells = frontier_list

		exploredGC = GridCells()
		exploredGC.header.frame_id = 'Explored'
		exploredGC.cell_width = resolution
		exploredGC.cell_height = resolution
		exploredGC.cells = explored_list

		unexploredGC = GridCells()
		unexploredGC.header.frame_id = 'Unexplored'
		unexploredGC.cell_width = resolution
		unexploredGC.cell_height = resolution
		unexploredGC.cells = unexplored_list

		obstaclesGC = GridCells()
		obstaclesGC.header.frame_id = 'Obstacles'
		obstaclesGC.cell_width = resolution
		obstaclesGC.cell_height = resolution
		obstaclesGC.cells = obstacles_list

		pathGC = GridCells()
		pathGC.header.frame_id = 'Shortest_Path'
		pathGC.cell_width = resolution
		pathGC.cell_height = resolution
		pathGC.cells = path_list

		#Publishes the GridCells Objects
		frontier_pub.publish(frontierGC)
		explored_pub.publish(exploredGC)
		unexplored_pub.publish(unexploredGC)
		obstacles_pub.publish(obstaclesGC)
		path_pub.publish(pathGC)

		#Wait so we don't spam everything
		rospy.sleep(rospy.Duration(1))
