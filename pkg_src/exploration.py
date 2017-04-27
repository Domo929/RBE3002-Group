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
        #print yaw

    stop = Twist()#all values default to 0 so this msg will stop the robot
    pub.publish(stop)

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
	    # odom_tf.sendTransform((pose.pose.position.x, pose.pose.position.y, 0), 
	    #         (pose.pose.orientation.x, pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w),rospy.Time.now(),"base_footprint","odom")
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

	goal.target_pose.header.stamp = rospy.get_rostime()
	goal.target_pose.header.frame_id='map'
	goal.target_pose.pose.position.x=centriod.x
	goal.target_pose.pose.position.y=centriod.y
	goal.target_pose.pose.orientation.w = 1.0

	
	ac.wait_for_server()
	print goal
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
	pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size=10) # Publisher for commanding robot motion
	odom_list = tf.TransformListener()

	rospy.sleep(rospy.Duration(2))


	print "Rotation sent to goal"
	rotate(3.14)
	print "Rotation 1 complete"
	rotate(3.14)
	print "Rotation Complete"


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
	#print listOfFrontierCentroids

	while(len(listOfFrontierCentroids) > 0 and not(rospy.is_shutdown())): #while there is a frontier
		#find closest frontier centriiods
		#print ("length",len(listOfFrontierCentroids) )
		minDistance = -1
		for centroid in listOfFrontierCentroids:
			tempDistance = math.sqrt((centroid.x-xPosition)**2+(centroid.y-yPosition)**2)
			if(tempDistance < minDistance or minDistance == -1):
				minDistance = tempDistance
				minCentriod = centroid

		print ("SEND GOAL: ", sendGoal(centroid)) #sends goal and doesn't proceed until the robot is at the goal position

		rospy.sleep(rospy.Duration(1))
		print "Rotation sent to goal"
		rotate(3.14)
		print "Rotation 1 complete"
		rotate(3.14)
		print "Rotation Complete"
		rospy.sleep(rospy.Duration(1))
		listOfFrontierCentroids = frontierFunctions.findFrontierRegionCentriods(mapOG,threshold)

	print "Area has been fully explored"