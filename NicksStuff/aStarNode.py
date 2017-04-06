#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from Node import Node
from aStar import aStar
from nav_msgs.msg import GridCells


def saveMap(input):
    global mainMap
    width = input.info.width
    height = input.info.height
    
    mainMap = [[0 for x in range(width)] for y in range(height)]
    
    y=0
    while(y<height):
        x=0
        while(x<width):
            if(input.data[y*width+x]>50):
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
    print("Before while loop")
    while(mainMap == []):
        print("Waiting")
    print("After while loop")
    start = mainMap[1][1]
    end = mainMap[10][10]


    aStarObject = aStar(mainMap)
    print("BEFORE")
    pathReturned = aStarObject.aStar(start,end)
    print("AFTER")
    pubPath = rospy.Publisher('/mapData/Path',GridCells,queue_size=10)
    print(pathReturned)
    pubPathInfo = GridCells()
    pubPathInfo.header.frame_id = "map"
    pubPathInfo.cell_width =1
    pubPathInfo.cell_height=1
    pubPathInfo.cells = pathReturned
    pubPath.publish(pubPathInfo)