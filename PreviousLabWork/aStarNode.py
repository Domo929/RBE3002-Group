#!/usr/bin/env python

import rospy, tf
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped, Pose
from Node import Node
from aStar import aStar
from nav_msgs.msg import GridCells, Path
import math
from nav_msgs.msg import Odometry


def timerCallback(event):
    global resolution
    global start
    global goal
    global mainMap
    global aStar

    print("Costmap stuff")
    print(start)

    waypointsToPublish = aStar.returnPathWaypoints(start, goal, mainMap, resolution)

    #for waypoint in listOfWaypoints: #go through all waypoints and publish them
    goal_pub.publish(waypointsToPublish)
    print("Published Waypoint")

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
    global aStar
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
    aStar = aStar(mainMap)
    aStar.addBuffer(1)


def setGoal(msg):
	global GoalPose
	global mainMap
	global goal
	GoalPose = Pose()
	GoalPose.position = msg.pose.position
	GoalPose.orientation = msg.pose.orientation
	goal = mainMap[GoalPose.position.x][GoalPose.position.y]

def setCurrentPosition(msg):
    global start
    global mainMap
    global resolution

    start = mainMap[int(msg.x/resolution)+3][int(msg.y/resolution)+3]



def readOdom(msg):
    """Read odometry messages and store into global variables."""
    global pose
    global xPosition
    global yPosition
    global theta
    global odom_list
    global odom_tf
    global start
    global mainMap
    try:
        pose = msg.pose
        geo_quat = pose.pose.orientation
        q = [geo_quat.x, geo_quat.y, geo_quat.z, geo_quat.w]
        odom_tf.sendTransform((pose.pose.position.x, pose.pose.position.y, 0), 
                (pose.pose.orientation.x, pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w),rospy.Time.now(),"base_footprint","odom")
        #Convert transform to global usable coordinates (x, y, theta)
        (trans, rot) = odom_list.lookupTransform('map', 'base_footprint', rospy.Time(0))
        roll, pitch, yaw = euler_from_quaternion(rot)
        theta = yaw * (180.0/math.pi)
        #xPosition = trans[0]
        #yPosition = trans[1]
        start = mainMap[xPosition][yPosition]
    except:
        print "aStarNode waiting for tf..."


if __name__ == '__main__':

    global mainMap 
    global StartPose
    global GoalPose
    global pose
    global start
    global goal
    global resolution
    global costmap
    global odom_list
    global odom_tf
    global aStar
    rospy.init_node('aStar')
    resolution=.3

    costmap=None
    resolution = 0
    mainMap = []
    #start = None
    #end = None

    subMap = rospy.Subscriber('/map',OccupancyGrid,saveMap,queue_size=10)
    #StartSub = rospy.Subscriber('/odom', Odometry, readOdom)
    #sub = rospy.Subscriber('/odom', Odometry, readOdom)
    GoalSub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, setGoal, queue_size=10)
    costmapSub = rospy.Subscriber('/move_base/local_costmap/costmap',OccupancyGrid, queue_size=10)

    rospy.Subscriber('/currentLocation', Point, setCurrentPosition, queue_size=10)

    goal_pub = rospy.Publisher('move_base_simple/goal1', Path, queue_size=1)
    pubPath = rospy.Publisher('/mapData/Path',GridCells,queue_size=10)
    pubWaypoint = rospy.Publisher('/waypoint',Path,queue_size=10)
    pubBuffer = rospy.Publisher('/mapData/Buffer',GridCells,queue_size=10)

    

    odom_list = tf.TransformListener()
    odom_tf = tf.TransformBroadcaster()
    odom_tf.sendTransform((0, 0, 0),(0, 0, 0, 1),rospy.Time.now(),"base_footprint","odom")
    
    rospy.sleep(rospy.Duration(1))

    #print mainMap
    # start = mainMap[3][3]
    goal = mainMap[10][10]
    #aStar = aStar(mainMap)


    #Tf setup
    # odom_list = tf.TransformListener()
    # odom_tf = tf.TransformBroadcaster()
    # odom_tf.sendTransform((0, 0, 0),(0, 0, 0, 1),rospy.Time.now(),"base_footprint","odom")

    #Costmap callback
    rospy.Timer(rospy.Duration(15), timerCallback)
    #timerCallback(1)
    print "done"
    while(not rospy.is_shutdown()):
        a= 1
    # rospy.sleep(rospy.Duration(1))
    # start = mainMap[3][3]
    # end = mainMap[10][10]