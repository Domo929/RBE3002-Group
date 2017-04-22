#!/usr/bin/env python

import rospy, tf, numpy, math
import actionlib
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import *
from nav_msgs.msg import Odometry
from findingFrontiers import FindFrontiers
from geometry_msgs.msg import Twist, Point

def readOdom(msg):
    """Read odometry messages and store into global variables."""
    global xPosition
    global yPosition
    global theta
    global odom_list
    global odom_tf
    global pose
    try:
        currentPose = msg.pose
        pose = msg.pose
        geo_quat = pose.pose.orientation
        q = [geo_quat.x, geo_quat.y, geo_quat.z, geo_quat.w]
        odom_tf.sendTransform((pose.pose.position.x, pose.pose.position.y, 0), 
                (pose.pose.orientation.x, pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w),rospy.Time.now(),"base_footprint","odom")
        #Convert transform to global usable coordinates (x, y, theta)
        (trans, rot) = odom_list.lookupTransform('map', 'base_footprint', rospy.Time(0))
        roll, pitch, yaw = euler_from_quaternion(rot)
        theta = yaw * (180.0/math.pi)
        xPosition = trans[0]
        yPosition = trans[1]

    except:
        print "drivingCode waiting for tf..."

def saveMap(msg):
	global map 
	global hasMap

	map = msg
	hasMap = True

def sendGoal(centriod):
	global ac
	goal = MoveBaseGoal()

	goal.target_pose.pose.position.x=centriod.x
	goal.target_pose.pose.position.y=centriod.y
	
	ac.wait_for_server()

	ac.send_goal(goal)
	ac.wait_for_result() # this probably wont work. It is going to have to be switched to use
	return ac.get_state() # feedback insead of result. 

# def rotate(angle):
#     global pose
#     global newPath
#     Kp = 5

#     quaternion = (  #add values into quaternion in order to convert
#     pose.pose.orientation.x,
#     pose.pose.orientation.y,
#     pose.pose.orientation.z,
#     pose.pose.orientation.w)
#     euler = tf.transformations.euler_from_quaternion(quaternion)
#     yaw = euler[2] + 3.14

#     twist = Twist() #all values default to 0
#     initOrien = yaw
#     currentPose = yaw

#     amountToTurn = yaw + angle

#     amountToTurn = amountToTurn%6.28

#     if(angle>0):
#         setSpeed= 0.4
#     else:
#         setSpeed=-0.4

#     while(abs(amountToTurn - yaw) > .005 and not rospy.is_shutdown() and not newPath):
#         pub.publish(twist)
#         if(abs(setSpeed * (amountToTurn - yaw) *Kp) > abs(setSpeed)):
#             twist.angular.z = setSpeed
#         else:
#             twist.angular.z = setSpeed * abs(amountToTurn - yaw) *Kp

#         currentPose = yaw
#         quaternion = (  #add values into quaternion in order to convert
#         pose.pose.orientation.x,
#         pose.pose.orientation.y,
#         pose.pose.orientation.z,
#         pose.pose.orientation.w)
#         euler = tf.transformations.euler_from_quaternion(quaternion)
#         yaw = euler[2] + 3.14
#         #print yaw

#     stop = Twist()#all values default to 0 so this msg will stop the robot
#     pub.publish(stop)

def rotateDegrees(angle):
    """Rotate and angle in degrees."""
    rotate(angle * (math.pi / 180))

if __name__ == '__main__':
	global map
	global hasMap 
	global odom_list
	global odom_tf
	global xPosition
	global yPosition
	global ac
	global pose
	global pub 

	rospy.init_node('Exploration')
	
	threshold = 1
	hasMap= False
	listOfFrontierCentriods = []
	frontierFunctions = FindFrontiers()
	#sub = rospy.Subscriber('/odom', Odometry, readOdom)
	#odom_list = tf.TransformListener()
	#odom_tf = tf.TransformBroadcaster()
	#odom_tf.sendTransform((0, 0, 0),(0, 0, 0, 1),rospy.Time.now(),"base_footprint","odom")
	ac = actionlib.SimpleActionClient('move_base',MoveBaseAction)
	
	print "Begining Exploration"
	subMap = rospy.Subscriber('/map',OccupancyGrid,saveMap,queue_size=3)
	#sub = rospy.Subscriber('/odom', Odometry, readOdom)
	#pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size=10) # Publisher for commanding robot motion
    
	ac.wait_for_server()
	print "Service setup successful"

	#rotateDegrees(360) #rotate in place to create inital map
	print "inital rotate complete"

	while(not(hasMap) and not(rospy.is_shutdown())): #wait to recieve inital map
		rospy.sleep(rospy.Duration(1))
		print "Waiting for map"

	rospy.sleep(rospy.Duration(2))
	print "Map recieved"
	listOfFronierCentriods = frontierFunctions.findFrontierRegionCentriods(map,threshold)
	print "Centriods Found"
	while(len(listOfFrontierCentriods) > 0 and not(rospy.is_shutdown())): #while there is a frontier
		#find closest frontier centriiods
		minDistance = -1
		for centriod in listOfFrontierCentriods:
			tempDistance = sqrt((centriod.x-xPosition)**2+(centriod.y-yPosition)**2)
			if(tempDistance < minDistance or minDistance == -1):
				minDistance = tempDistance
				minCentriod = centriod

		sendGoal(centriod) #sends goal and doesn't proceed until the robot is at the goal position
		rospy.sleep(rospy.Duration(2))
		listOfFronierCentriods = findFrontierRegionCentriods(map,threshold)

	print "Area has been fully explored"