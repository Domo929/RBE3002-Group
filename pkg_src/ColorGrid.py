#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point	

def findGrid(msg):
	global width
	global height
	global index
	global obstacle_pub
	global unexplored_pub

	print "I Got A Message"

	width = msg.info.width
	height = msg.info.height
	index = msg.data

	# lstOb = []
	# lstUn = []
	# for x in range(0, width):
	# 	for y in range(0, height):
	# 		val = index[x*width+y]
	# 		#print val
	# 		if val == 100:
	# 			lstOb.append(Point(x,y,0))
	# 		elif val == 0:
	# 			lstUn.append(Point(x,y,0))

	# ObGridCell = GridCells()
	# #ObGridCell.header.frame_id = 'map'
	# ObGridCell.cell_width = 0.3
	# ObGridCell.cell_height - 0.3
	# ObGridCell.cells.append(lstOb)

	# UnGridCell = GridCells()
	# #UnGridCell.header.frame_id = 'map'
	# UnGridCell.cell_width = 0.3
	# UnGridCell.cell_height = 0.3
	# UnGridCell.cells.append(lstUn)
	# while(1==1):

	# 	obstacle_pub.publish(ObGridCell)
	# 	unexplored_pub.publish(UnGridCell)






if __name__ == "__main__":
	global index
	global width 
	global height
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

	print "begin"
	while not rospy.is_shutdown():
	
		lstOb = []
		lstUn = []
		for x in range(0, width):
			for y in range(0, height):
				val = index[y*width+x]
				#print val
				if val == 100:
					lstOb.append(Point(x,y,0))
				elif val == 0:
					lstUn.append(Point(x,y,0))

		ObGridCell = GridCells()
		ObGridCell.header.frame_id = 'map'
		ObGridCell.cell_width = 0.3
		ObGridCell.cell_height = 0.3
		ObGridCell.cells = lstOb
		#ObGridCell.cells.append(Point(2,3,0))
		#ObGridCell.cells.append(Point(4,5,0))

		UnGridCell = GridCells()
		UnGridCell.header.frame_id = 'map'
		UnGridCell.cell_width = 0.3
		UnGridCell.cell_height = 0.3
		UnGridCell.cells = lstUn
		#UnGridCell.cells.append(Point(4,6,0))
		#UnGridCell.cells.append(Point(1,7,0))

		obstacle_pub.publish(ObGridCell)
		unexplored_pub.publish(UnGridCell)

		rospy.sleep(rospy.Duration(1))
