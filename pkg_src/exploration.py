#!/usr/bin/env python


'''Aren't currently broadcasting tf'''
'''Goal needs to be PoseStamped when sent to actionServer'''
'''Feedback provides server implementers a way to tell an ActionClient about the incremental progress of a goal. For moving the base, this might be the robot's current pose along the path.'''
'''For move base, the result isn't very important, but it might contain the final pose of the robot'''




import rospy, tf, numpy, math
import actionlib
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import *
from nav_msgs.msg import Odometry
from findingFrontiers import FindFrontiers
from geometry_msgs.msg import Twist, Point
from tf.transformations import euler_from_quaternion

def readOdom(msg):
    """Read odometry messages and store into global variables."""
    global xPosition
    global yPosition
    global theta
    global odom_list
    global odom_tf
    global pose
    try:
	    pose = msg.pose
	    odom_tf.sendTransform((pose.pose.position.x, pose.pose.position.y, 0), 
	            (pose.pose.orientation.x, pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w),rospy.Time.now(),"base_footprint","odom")
	    (trans, rot) = odom_list.lookupTransform('map', 'base_footprint', rospy.Time(0))
	    roll, pitch, yaw = euler_from_quaternion(rot)
	    theta = yaw * (180.0/math.pi)
	    xPosition = trans[0]
	    yPosition = trans[1]

    except:
        print "drivingCode waiting for tf..."

def saveMap(msg):
	global mapOG 
	global hasMap

	mapOG = msg
	hasMap = True
def sendGoal(centriod):
	global ac
	goal = MoveBaseGoal()

	goal.target_pose.header.frame_id='map'
	goal.target_pose.pose.position.x=centriod.x
	goal.target_pose.pose.position.y=centriod.y
	goal.target_pose.pose.orientation.w = 1

	
	ac.wait_for_server()
	ac.send_goal(goal)
	ac.wait_for_result() # this probably wont work. It is going to have to be switched to use
	return ac.get_state() # feedback insead of result. 

if __name__ == '__main__':
	global mapOG
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
	ac = actionlib.SimpleActionClient('move_base',MoveBaseAction)
	sub = rospy.Subscriber('/odom', Odometry, readOdom)
	odom_list = tf.TransformListener()
	odom_tf = tf.TransformBroadcaster()
	odom_tf.sendTransform((0, 0, 0),(0, 0, 0, 1),rospy.Time.now(),"base_footprint","odom")
	rospy.sleep(rospy.Duration(2))

	print "Begining Exploration"
	subMap = rospy.Subscriber('/move_base/global_costmap/costmap',OccupancyGrid,saveMap,queue_size=3)
	ac.wait_for_server()
	print "Service setup successful"

	while(not(hasMap) and not(rospy.is_shutdown())): #wait to recieve inital map
		rospy.sleep(rospy.Duration(1))
		print "Waiting for map"

	print "Map recieved"
	listOfFrontierCentroids = frontierFunctions.findFrontierRegionCentriods(mapOG,threshold)
	print "Centriods Found"
	print ("length",len(listOfFrontierCentroids) )
	print listOfFrontierCentroids

	while(len(listOfFrontierCentroids) > 0 and not(rospy.is_shutdown())): #while there is a frontier
		#find closest frontier centriiods
		print ("length",len(listOfFrontierCentroids) )
		minDistance = -1
		for centroid in listOfFrontierCentroids:
			tempDistance = math.sqrt((centroid.x-xPosition)**2+(centroid.y-yPosition)**2)
			if(tempDistance < minDistance or minDistance == -1):
				minDistance = tempDistance
				minCentriod = centroid

		print sendGoal(centroid) #sends goal and doesn't proceed until the robot is at the goal position
		rospy.sleep(rospy.Duration(2))
		listOfFrontierCentroids = frontierFunctions.findFrontierRegionCentriods(mapOG,threshold)

	print "Area has been fully explored"