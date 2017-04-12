#!/usr/bin/env python

import rospy, tf, math
from kobuki_msgs.msg import BumperEvent
# Add additional imports for each of the message types use
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


#drive to a goal subscribed as /move_base_simple/goal
def navToPose(goal):
    global pose
    global goalPose

    quaternion = (  #add values into quaternion in order to convert
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]
    print "yaw"
    print yaw
    quaternion = (  #add values into quaternion in order to convert
        goalPose.orientation.x,
        goalPose.orientation.y,
        goalPose.orientation.z,
        goalPose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    goalYaw = euler[2]
    
    angleToGoal = math.atan2(goalPose.position.y-2-pose.position.y,(goalPose.position.x-2-pose.position.x))
    distanceToGo = math.sqrt((goalPose.position.y-2-pose.position.y)**2+(goalPose.position.x-2-pose.position.x)**2)

    print "a"
    print goalPose.position.y-2- pose.position.y

    print "b"
    print goalPose.position.x-2-pose.position.x


    print "spin!"
    rotate(angleToGoal-yaw)
    print "move!"
    print distanceToGo
    driveStraight(.4,distanceToGo)
    print "distanceToGo"
    print "spin!"
    quaternion = (  #update current yaw
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]
    
    rotate(goalYaw-yaw)
    print "done"
    pass

#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    global pose

    Kp = 5
    dTraveled = 0
    initialPos = pose #set inital position

    twist=Twist()  #create message and set values to driving striaght at 'speed'
    twist.linear.x=speed
    #all other values default to 0
    error = distance -dTraveled
    while(abs(error) > 0.005 and not rospy.is_shutdown()): #while you have traveled less than distance
        dTraveled = math.sqrt((pose.position.x-initialPos.position.x)**2+(pose.position.y-initialPos.position.y)**2) # sqrt(delta x^2 + delta y^2) Distance traveled
        #print dTraveled
        if(speed*error*Kp>speed):
            twist.linear.x=speed
            #all other values default to 0
        else:
            twist.linear.x=speed*error*Kp  #publish speeds
            #all other values default to 0

        pub.publish(twist)  #publish speeds
        error = distance - dTraveled


    stop = Twist()#all values default to 0 so this msg will stop the robot
    pub.publish(stop)
    
#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
    global pose
    Kp = 5

    quaternion = (  #add values into quaternion in order to convert
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2] + 3.14

    twist = Twist() #all values default to 0
    initOrien = yaw
    currentPose = yaw

    amountToTurn = yaw + angle

    amountToTurn = amountToTurn%6.28

    if(angle>0):
        setSpeed= 0.4
    else:
        setSpeed=-0.4

    while(abs(amountToTurn - yaw) > .005 and not rospy.is_shutdown()):
        pub.publish(twist)
        if(abs(setSpeed * (amountToTurn - yaw) *Kp) > abs(setSpeed)):
            twist.angular.z = setSpeed
        else:
            twist.angular.z = setSpeed * abs(amountToTurn - yaw) *Kp

        currentPose = yaw
        quaternion = (  #add values into quaternion in order to convert
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2] + 3.14
        #print yaw

    stop = Twist()#all values default to 0 so this msg will stop the robot
    pub.publish(stop)

#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
    global L
    global pose

    twist = Twist()
    #omega = speed/(radius+(L/2))
    #vl=speed-omega*L
    dTravel = 2*3.14*radius
    time = dTravel/speed
    omega = angle/time
    v = omega*(radius)

    twist.angular.z = omega
    twist.linear.x=v
    #################################
    Kp = 5

    quaternion = (  #add values into quaternion in order to convert
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2] + 3.14

    initOrien = yaw
    currentPose = yaw

    amountToTurn = yaw + angle

    amountToTurn = amountToTurn%6.28

    while(abs(amountToTurn - yaw) > .005 and not rospy.is_shutdown()):
        pub.publish(twist)
        currentPose = yaw
        quaternion = (  #add values into quaternion in order to convert
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2] + 3.14
        #print yaw

    stop = Twist()#all values default to 0 so this msg will stop the robot
    pub.publish(stop)

#Bumper Event Callback function
def readBumper(msg):
    if (msg.state == 1):
        executeTrajectory()

def odomPrint(msg):
    global pose
    pose = msg.pose.pose
    #print pose

def readGoal(goal):
    global goalPose
    goalPose = goal.pose
    navToPose(goalPose)


# This is the program's main function
def initDrivingCode():
    # Change this node name to include your username
    rospy.init_node('sample_Lab_2_node')

    # These are global variables. Write "global <variable_name>" in any other function to gain access to these global variables 
    global pub
    global goalPose
    global pose
    global odom_tf
    global odom_list
    global odom
    global L 
    L = 23
    global D
    D = 7.6
   
    # Replace the elipses '...' in the following lines to set up the publishers and subscribers the lab requires
    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=4)
    bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events
    odom = rospy.Subscriber("/odom", Odometry, odomPrint)
    goal_sub = rospy.Subscriber("/move_base_simple/goal",PoseStamped,readGoal)
    # Use this object to get the robot's Odometry 
    #odom_list = tf.TransformListener()
    #print odom_list
    # Use this command to make the program wait for some seconds
    #rospy.sleep(rospy.Duration(1, 0))

    print "Lab 2 complete!"