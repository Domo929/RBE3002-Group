#!/usr/bin/env python 

#The messages we use, and the rospy import
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import GridCells
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

def makeGridCells(label, cell_dim, cells):
	gc = GridCells()
	gc.header.frame_id = label
	gc.cell_width = cell_dim
	gc.cell_height = cell_dim
	gc.cells = cells

	return gc


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

	#Gross stuff to make sure it doesn't shit the bed if it hasnt recieved a message yet
	try:
		width
	except NameError:
		width = 0
	else:
		width = width

	try:
		height
	except NameError:
		height = 0
	else:
		height = height

	try:
		index
	except NameError:
		index = []
	else:
		index = index

	try:
		resolution
	except NameError:
		resolution = 1
	else:
		resolution = resolution

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
		frontierGC   = makeGridCells('Frontier',   resolution, frontier_list)
		exploredGC   = makeGridCells('Explored',   resolution, explored_list)
		unexploredGC = makeGridCells('Unexplored', resolution, unexplored_list)
		obstaclesGC  = makeGridCells('Obstacles',  resolution, obstacles_list)
		pathGC       = makeGridCells('Path',       resolution, path_list)


		#Publishes the GridCells Objects
		frontier_pub.publish(frontierGC)
		explored_pub.publish(exploredGC)
		unexplored_pub.publish(unexploredGC)
		obstacles_pub.publish(obstaclesGC)
		path_pub.publish(pathGC)

		#Wait so we don't spam everything
		rospy.sleep(rospy.Duration(1))