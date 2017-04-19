#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid

def talker():
	pub = rospy.Publisher('builtMap',OccupancyGrid, queue_size=10)


if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass