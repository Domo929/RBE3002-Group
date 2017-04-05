#!/usr/bin/env python
import aStar
import rospy
from nav_msgs.msg import OccupancyGrid



def saveMap(data):
	global mainMap
	mainMap = data.data


if __name__ == '__main__':
    # Change this node name to include your username.Sub
    global mainMap 
    mainMap = []
    subMap = rospy.Subscriber('/map',OccupancyGrid,saveMap)
    rospy.init_node('aStar')
    while(mainMap == []):
    	farts = 1

    start = Point()
    end = Point()
    end.x=10
    end.y=10
    end.z=0
	
    aStar = aStar(mainMap)
    aStar.runAStar(start,end)

