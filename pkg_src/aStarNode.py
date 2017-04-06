#!/usr/bin/env python

import rospy, tf
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, PoseStamped
from Node import Node
from aStar import aStar
from nav_msgs.msg import GridCells, Path
import math


def saveMap(input):
    print("savingMap")
    global mainMap
    width = input.info.width
    height = input.info.height
    print(width)
    print(height)
    
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

    print("Done Saving Map")


if __name__ == '__main__':
    # Change this node name to include your username.Sub
    global mainMap 
    mainMap = []
    subMap = rospy.Subscriber('/map',OccupancyGrid,saveMap)
    rospy.init_node('aStar')
    print("before while loop")
    while(mainMap == []):
    	print("Waiting")
    print("after while loop")

    start = mainMap[1][1]
    end = mainMap[10][10]

    print("after declairations")
    aStarObject = aStar(mainMap)
    print("After astar")
    pathReturned = aStarObject.aStar(start,end)
    pubPath = rospy.Publisher('/mapData/Path',GridCells,queue_size=10)
    pubWaypoint = rospy.Publisher('/waypoint',Path,queue_size=10)
    #print(pathReturned)
    pubPathInfo = GridCells()
    pubPathInfo.header.frame_id = "map"
    pubPathInfo.cell_width =1
    pubPathInfo.cell_height=1
    pubPathInfo.cells = pathReturned
    #print(pathReturned)
    lastPose = pathReturned[0]
    #print "pathReturned 0"
    #print pathReturned[0]
    poses2 = []
    for point in pathReturned:
        pose = lastPose
        goalPose = point

        angleToNextPoint = math.atan2(goalPose.y-pose.y,(goalPose.x-pose.x))

        quaternion = tf.transformations.quaternion_from_euler(0,0,angleToNextPoint)
        tempPose = PoseStamped()
        tempPose.header.frame_id = 'map'
        tempPose.pose.position.x=pose.x
        tempPose.pose.position.y=pose.y
        tempPose.pose.orientation.x = quaternion[0]
        tempPose.pose.orientation.y = quaternion[1]
        tempPose.pose.orientation.z = quaternion[2]
        tempPose.pose.orientation.w = quaternion[3]
        #tempPose.pose.orientation = quaternion
        poses2.append(tempPose)
        lastPose = point

    waypointsToPublish = Path()
    waypointsToPublish.header.frame_id = 'map'
    waypointsToPublish.poses = poses2
    while not rospy.is_shutdown():
        pubWaypoint.publish(waypointsToPublish)
        pubPath.publish(pubPathInfo)
        rospy.sleep(rospy.Duration(1))

