#!/usr/bin/env python
""" IMPORTS """
import rospy, tf, numpy, math, actionlib 
from nav_msgs.msg import OccupancyGrid, Odometry
from move_base_msgs.msg import *
from findingFrontiers import FindFrontiers
from geometry_msgs.msg import Twist, Point
from tf.transformations import euler_from_quaternion



"""
Takes in an angle in radians and publishes Twist messages to rotate the robot to that angle

	Params: angle; the angle that you wish to rotate to
	Returns: None; rotates the robot to the given angle
"""
def rotate(angle):

    global pose
    Kp = 5

    quaternion = (  #add values into quaternion in order to convert
    pose.pose.orientation.x,
    pose.pose.orientation.y,
    pose.pose.orientation.z,
    pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2] + 3.14

    twist = Twist() #all values default to 0
    initOrien = yaw
    currentPose = yaw

    amountToTurn = yaw + angle

    amountToTurn = amountToTurn%6.28

    if(angle>0):
        setSpeed= 0.5
    else:
        setSpeed=-0.5

    while(abs(amountToTurn - yaw) > .1 and not rospy.is_shutdown()):
        pub.publish(twist)
        if(abs(setSpeed * (amountToTurn - yaw) *Kp) > abs(setSpeed)):
            twist.angular.z = setSpeed
        else:
            twist.angular.z = setSpeed * abs(amountToTurn - yaw) *Kp

        currentPose = yaw
        quaternion = (  #add values into quaternion in order to convert
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2] + 3.14


    stop = Twist()#all values default to 0 so this msg will stop the robot
    pub.publish(stop)

"""
Takes in an odometry message and sets global values to use in other methods
	
	Params: msg; the odometry message received from subscribing to the /odom topic
	Returns: None; sets global values for the robots current pose
"""
def readOdom(msg):
    global xPosition
    global yPosition
    global theta
    global odom_list
    global odom_tf
    global pose
    try:
	    pose = msg.pose
	    (trans, rot) = odom_list.lookupTransform('map', 'base_footprint', rospy.Time(0)) #Looks up the current robot pose transform
	    roll, pitch, yaw = 
	    euler_from_quaternion(rot)
	    theta = yaw * (180.0/math.pi)
	    xPosition = trans[0]
	    yPosition = trans[1]
    except:
        print "drivingCode waiting for tf..."
"""
Takes in an OccupancyGrid and sets global values to use in other methods
	
	Params: msg; the OccupancyGrid message received from subscribing to the /move_base/global_costmap/costmap topic
	Returns: None; sets global values for the current map
"""
def saveMap(msg):
	global mapOG 
	global hasMap

	mapOG = msg
	hasMap = True
"""
Takes in a centroid and navigates to it

	Params: centroid; the centroid that the robot will navigate to
	Returns: ac.get_state(); the state that the navigation request returns
"""
def sendGoal(centriod):
	global ac
	goal = MoveBaseGoal()

	goal.target_pose.header.stamp = rospy.get_rostime()
	goal.target_pose.header.frame_id='map'
	goal.target_pose.pose.position.x=centriod.x
	goal.target_pose.pose.position.y=centriod.y
	goal.target_pose.pose.orientation.w = 1.0
	
	ac.wait_for_server()
	print goal
	ac.send_goal(goal)
	ac.wait_for_result()
	return ac.get_state()
"""
SLAM a previously unexplored environment
"""
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

	"""

		Subscriber, action client, and publisher setup

	"""
	sub = rospy.Subscriber('/odom', Odometry, readOdom)
	ac = actionlib.SimpleActionClient('move_base',MoveBaseAction)
	pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size=10)


	odom_list = tf.TransformListener()
	rospy.sleep(rospy.Duration(2))


	"""

		Initial rotate, lets us see as much of the current environment as possible from our current location

	"""
	rotate(3.14)
	rotate(3.14)


	print "Begining Exploration"
	subMap = rospy.Subscriber('/move_base/global_costmap/costmap',OccupancyGrid,saveMap,queue_size=3)
	ac.wait_for_server()
	print "Service setup successful"

	while(not(hasMap) and not(rospy.is_shutdown())): #Wait to recieve inital map
		rospy.sleep(rospy.Duration(1))
		print "Waiting for map"
	print "Map recieved"


	listOfFrontierCentroids = frontierFunctions.findFrontierRegionCentriods(mapOG,threshold) #Find a list of frontier centroids

	#While there is a frontier navigate to navigate to the last centroid
	while(len(listOfFrontierCentroids) > 0 and not(rospy.is_shutdown())):
		minDistance = -1
		for centroid in listOfFrontierCentroids:
			tempDistance = math.sqrt((centroid.x-xPosition)**2+(centroid.y-yPosition)**2)
			if(tempDistance < minDistance or minDistance == -1):
				minDistance = tempDistance
				minCentriod = centroid

		print ("SEND GOAL: ", sendGoal(centroid)) #sends goal and doesn't proceed until the robot is at the goal position

		rospy.sleep(rospy.Duration(1))
		"""

			Rotate in place to explore more of the map

		"""
		rotate(3.14)
		rotate(3.14)
		rospy.sleep(rospy.Duration(1))
		listOfFrontierCentroids = frontierFunctions.findFrontierRegionCentriods(mapOG,threshold)

	print "Area has been fully explored"