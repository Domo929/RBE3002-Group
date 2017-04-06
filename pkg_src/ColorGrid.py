#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point	

def findGrid(msg):
	global width
	global height
	global index
	global resolution

	print "I Got A Message"

	width = msg.info.width
	height = msg.info.height
	resolution = msg.info.resolution
	index = msg.data






if __name__ == "__main__":
	global index
	global width 
	global height
	global resolution
	global obstacle_pub
	global unexplored_pub
	rospy.init_node('ColorGrid')
	obstacle_pub = rospy.Publisher('mapData/Obstacles', GridCells, queue_size = 10)
	unexplored_pub = rospy.Publisher('mapData/Unexplored', GridCells, queue_size = 10)
	map_sub = rospy.Subscriber('/map', OccupancyGrid, findGrid, queue_size=1)

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

	print "begin"
	while not rospy.is_shutdown():
	
		lstOb = []
		lstUn = []
		for x in range(0, width):
			for y in range(0, height):
				val = index[y*width+x]
				#print val
				if val == 100:
					lstOb.append(Point(resolution*x,resolution*y,0))
				elif val == 0:
					lstUn.append(Point(resolution*x,resolution*y,0))

		ObGridCell = GridCells()
		ObGridCell.header.frame_id = 'map'
		ObGridCell.cell_width = resolution
		ObGridCell.cell_height = resolution
		ObGridCell.cells = lstOb

		UnGridCell = GridCells()
		UnGridCell.header.frame_id = 'map'
		UnGridCell.cell_width = resolution
		UnGridCell.cell_height = resolution
		UnGridCell.cells = lstUn

		obstacle_pub.publish(ObGridCell)
		unexplored_pub.publish(UnGridCell)

		rospy.sleep(rospy.Duration(1))
