#!/usr/bin/env python

import rospy, tf
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from Node import Node
from aStar import aStar
from nav_msgs.msg import GridCells, Path
import math


def timerCallback(event):
    print("Costmap stuff")
    #costmap= mikesFunction(costmap)
    #(trans, rot) = odom_list.lookupTransform('map', 'base_footprint', rospy.Time(0))
def costmapCallback(data):
    global costmap
    costmap=data

def convertToGridScaling(arr):
    global resolution
    for i in range(0, len(arr)):
        arr[i].x=arr[i].x*resolution + 0.1
        arr[i].y=arr[i].y*resolution + 0.2
    return arr
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
    global goal
    global resolution
    global costmap
    costmap=None
    resolution = 0
    mainMap = []
    start = None
    end = None

    subMap = rospy.Subscriber('/map',OccupancyGrid,saveMap,queue_size=10)
    StartSub = rospy.Subscriber('/initlalpose', PoseWithCovarianceStamped, setStart, queue_size=10)
    GoalSub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, setGoal, queue_size=10)
    costmapSub = rospy.Subscriber('/move_base/local_costmap/costmap',OccupancyGrid, queue_size=10)


    goal_pub = rospy.Publisher('move_base_simple/goal1', PoseStamped, queue_size=1)
    pubPath = rospy.Publisher('/mapData/Path',GridCells,queue_size=10)
    pubWaypoint = rospy.Publisher('/waypoint',Path,queue_size=10)
    pubBuffer = rospy.Publisher('/mapData/Buffer',GridCells,queue_size=10)

    rospy.init_node('aStar')


    #Tf setup
    # odom_list = tf.TransformListener()
    # odom_tf = tf.TransformBroadcaster()
    # odom_tf.sendTransform((0, 0, 0),(0, 0, 0, 1),rospy.Time.now(),"base_footprint","odom")

    #Costmap callback
    rospy.Timer(rospy.Duration(0.2), timerCallback)


    rospy.sleep(rospy.Duration(1))
    start = mainMap[3][3]
    end = mainMap[10][10]

    #create gridcell output for buffercells
    buffCell = GridCells()
    buffCell.header.frame_id = "map"
    buffCell.cell_width =resolution
    buffCell.cell_height=resolution
    buffCell.cells = convertToGridScaling(aStar(mainMap).addBuffer(1))

    pathReturned = aStar(mainMap).aStarPathFinding(start,end)

    #create gridcell output for path
    pathCell = GridCells()
    pathCell.header.frame_id = "map"
    pathCell.cell_width =resolution
    pathCell.cell_height=resolution
    pathCell.cells = convertToGridScaling(pathReturned)

    #create list of waypoints
    listOfWaypoints = []
    for x in range(1, len(pathReturned)-1):  
        currentNode = pathReturned[x]
        nextNode = pathReturned[x+1]
        previousNode = pathReturned[x-1]

        angleToNextPoint = math.atan2((nextNode.y-currentNode.y),(nextNode.x-currentNode.x))
        prevToCurrentAngle = math.atan2((currentNode.y-previousNode.y),(currentNode.x-previousNode.x))

        if(prevToCurrentAngle!=angleToNextPoint):
            quaternion = tf.transformations.quaternion_from_euler(0,0,angleToNextPoint)
            
            tempPose = PoseStamped()
            tempPose.header.frame_id = 'map'
            tempPose.pose.position.x=currentNode.x
            tempPose.pose.position.y=currentNode.y
            tempPose.pose.orientation.x = quaternion[0]
            tempPose.pose.orientation.y = quaternion[1]
            tempPose.pose.orientation.z = quaternion[2]
            tempPose.pose.orientation.w = quaternion[3]

            listOfWaypoints.append(tempPose)
#Last Waypoint
    tempPose = PoseStamped()
    tempPose.header.frame_id = 'map'
    tempPose.pose.position.x= pathReturned[-1].x
    tempPose.pose.position.y= pathReturned[-1].y
    listOfWaypoints.append(tempPose)
#First Waypoint
    tempPose = PoseStamped()
    tempPose.header.frame_id = 'map'
    tempPose.pose.position.x= pathReturned[0].x
    tempPose.pose.position.y= pathReturned[0].y
    listOfWaypoints.insert(0,tempPose)
#turn list of waypoints into publishable path
    waypointsToPublish = Path()
    waypointsToPublish.header.frame_id = 'map'
    waypointsToPublish.poses = listOfWaypoints
    listOfWaypoints.pop(0) #remove starting position
    for waypoint in listOfWaypoints: #go through all waypoints and publish them
        pubBuffer.publish(buffCell)
        pubWaypoint.publish(waypointsToPublish)
        pubPath.publish(pathCell)
        rospy.sleep(rospy.Duration(1))
        goal_pub.publish(waypoint)
    # uncomment to output list of waypoint with pretty formatting
    # for waypoint in listOfWaypoints:
    #     fancyString="{}, {}, {}"
    #     print("Waypoint", waypoint)
    #     print(fancyString.format(waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.orientation.z))
    print("Before Here")
    while not rospy.is_shutdown():
        pubBuffer.publish(buffCell)
        pubWaypoint.publish(waypointsToPublish)
        pubPath.publish(pathCell)
        rospy.sleep(rospy.Duration(1))
    print("After Here")

