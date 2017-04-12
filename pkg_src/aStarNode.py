#!/usr/bin/env python

import rospy, tf
from drivingCode import *
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from Node import Node
from aStar import aStar
from nav_msgs.msg import GridCells, Path
import math


def saveMap(input):
    global mainMap
    global resolution
    width = input.info.width
    height = input.info.height
    resolution = input.info.resolution
    
    mainMap = [[0 for x in range(width)] for y in range(height)]
    
    for y in range(0, height):
        for x in range(0, width):
            if(input.data[y*width+x]>50):
                tempNode = Node(x,y,-1)
            else:
                tempNode = Node(x,y,0)
            mainMap[x][y]=tempNode
def setStart(msg):
	global StartPose
	global mainMap
	global start
	StartPose = Pose()
	StartPose.position = msg.pose.position
	StartPose.orientation = msg.pose.orientation
	start = mainMap[StartPose.position.x][StartPose.position.y]

def setGoal(msg):
	global GoalPose
	global mainMap
	global goal
	GoalPose = Pose()
	GoalPose.position = msg.pose.position
	GoalPose.orientation = msg.pose.orientation
	goal = mainMap[GoalPose.position.x][GoalPose.position.y]

if __name__ == '__main__':

    global mainMap 
    global StartPose
    global GoalPose
    global start
    global end
    global resolution
    resolution = 0
    mainMap = []
    start = None
    end = None

    subMap = rospy.Subscriber('/map',OccupancyGrid,saveMap,queue_size=10)

    StartSub = rospy.Subscriber('/initlalpose', PoseWithCovarianceStamped, setStart, queue_size=10)
    GoalSub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, setGoal, queue_size=10)

    pubPath = rospy.Publisher('/mapData/Path',GridCells,queue_size=10)
    pubWaypoint = rospy.Publisher('/waypoint',Path,queue_size=10)
    pubBuffer = rospy.Publisher('/mapData/Buffer',GridCells,queue_size=10)
    initDrivingCode()
    rospy.sleep(rospy.Duration(1))

    rospy.init_node('aStar')

    #Remove when time to check Stage position settings
    start = mainMap[2][2]
    goal = mainMap[10][10]

    # while start == None or end == None:
    # 	rospy.sleep(rospy.Duration(1))


    pubBuf = GridCells()
    pubBuf.header.frame_id = "map"
    pubBuf.cell_width =resolution
    pubBuf.cell_height=resolution

    temp=aStar(mainMap).addBuffer(1) #Adds Buffer
    for i in range(0, len(temp)):
        temp[i].x=temp[i].x*resolution + 0.6
        temp[i].y=temp[i].y*resolution + 0.2
    pubBuf.cells = temp


    
    pathReturned = aStar(mainMap).aStarPathFinding(start,end)

    for i in range(0, len(pathReturned)):
        pathReturned[i].x=pathReturned[i].x*resolution + 0.6
        pathReturned[i].y=pathReturned[i].y*resolution + 0.2

    
    pubPathInfo = GridCells()
    pubPathInfo.header.frame_id = "map"
    pubPathInfo.cell_width = resolution
    pubPathInfo.cell_height= resolution
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


    holder = reversed(listOfWaypoints)
    for waypoint in holder:
        navToPose(waypoint)

    waypointsToPublish = Path()
    waypointsToPublish.header.frame_id = 'map'
    waypointsToPublish.poses = listOfWaypoints
    
    print("Before Here")
    while not rospy.is_shutdown():
        pubBuffer.publish(pubBuf)
        pubWaypoint.publish(waypointsToPublish)
        pubPath.publish(pubPathInfo)
        rospy.sleep(rospy.Duration(1))
    print("After Here")

