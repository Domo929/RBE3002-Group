#!/usr/bin/env python

import rospy, tf
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, PoseStamped
from Node import Node
from aStar import aStar
from nav_msgs.msg import GridCells, Path
import math


def saveMap(input):
    global mainMap
    width = input.info.width
    height = input.info.height
    
    mainMap = [[0 for x in range(width)] for y in range(height)]
    
    for y in range(0, height):
        for x in range(0, width):
            if(input.data[y*width+x]>50):
                tempNode = Node(x,y,-1)
            else:
                tempNode = Node(x,y,0)
            mainMap[x][y]=tempNode


if __name__ == '__main__':
    global mainMap 
    mainMap = []
    subMap = rospy.Subscriber('/map',OccupancyGrid,saveMap,queue_size=10)
    pubPath = rospy.Publisher('/mapData/Path',GridCells,queue_size=10)
    pubWaypoint = rospy.Publisher('/waypoint',Path,queue_size=10)

    rospy.init_node('aStar')

    start = mainMap[1][1]
    end = mainMap[10][10]

    aStar(mainMap).addBuffer(1)
    pathReturned = aStar(mainMap).aStarPathFinding(start,end)
    
    pubPathInfo = GridCells()
    pubPathInfo.header.frame_id = "map"
    pubPathInfo.cell_width =1
    pubPathInfo.cell_height=1
    pubPathInfo.cells = pathReturned
    
    lastPose = pathReturned[0]


    listOfWaypoints = []
    for x in range(1, len(pathReturned)-1):
        current = pathReturned[x]
        next = pathReturned[x+1]
        previous = pathReturned[x-1]

        angleToNextPoint = math.atan2(next.y-current.y,(next.x-current.x))
        prevToCurrentAngle = math.atan2(current.y-previous.y,(current.x-previous.x))

        if(prevToCurrentAngle!=angleToNextPoint):
            quaternion = tf.transformations.quaternion_from_euler(0,0,angleToNextPoint)
            
            tempPose = PoseStamped()
            tempPose.header.frame_id = 'map'
            tempPose.pose.position.x=current.x
            tempPose.pose.position.y=current.y
            tempPose.pose.orientation.x = quaternion[0]
            tempPose.pose.orientation.y = quaternion[1]
            tempPose.pose.orientation.z = quaternion[2]
            tempPose.pose.orientation.w = quaternion[3]

            listOfWaypoints.append(tempPose)

    tempPose = PoseStamped()
    tempPose.header.frame_id = 'map'
    tempPose.pose.position.x=pathReturned[len(pathReturned)-1].x
    tempPose.pose.position.y=pathReturned[len(pathReturned)-1].y
    listOfWaypoints.append(tempPose)

    waypointsToPublish = Path()
    waypointsToPublish.header.frame_id = 'map'
    waypointsToPublish.poses = listOfWaypoints
    
    while not rospy.is_shutdown():
        pubWaypoint.publish(waypointsToPublish)
        pubPath.publish(pubPathInfo)
        rospy.sleep(rospy.Duration(1))

