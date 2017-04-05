#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from Node import Node
from aStar import aStar



def saveMap(input):
	
    global mainMap
    width = input.info.width
    height = input.info.height
    #print(width)
    #print(height)
    
    mainMap = [[0 for x in range(width)] for y in range(height)]
    
    y=0
    while(y<height):
        x=0
        while(x<width):
            if(input.data[x*width+y]>50):
                tempNode = Node(x,y,-1)
            else:
                tempNode = Node(x,y,0)
            mainMap[x][y]=tempNode
            x+=1
             
        y+=1


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
	
    aStarObject = aStar(mainMap)
    aStarObject.runAStar(start,end)