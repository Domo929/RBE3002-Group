#!/usr/bin/env python

import rospy, tf, numpy
from kobuki_msgs.msg import BumperEvent
from tf.transformations import euler_from_quaternion
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped




#drive to a goal subscribed as /move_base_simple/goal
def navToPose(goal):
    print "In NavtoPose"

    #Robots Current Pose
    (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
    xPos = trans[0]
    yPos = trans[1]
    roll, pitch, yaw = euler_from_quaternion(rot)
    theta = yaw

    xTarget = goal.pose.position.x
    yTarget = goal.pose.position.y
    quatern = goal.pose.orientation
    quaternionGoal = [quatern.x, quatern.y, quatern.z, quatern.w]
    roll, pitch, yaw = euler_from_quaternion(quaternionGoal)


    xDiff = xTarget-xPos
    yDiff = yTarget-yPos
    distance = math.sqrt(math.pow(xDiff, 2)+math.pow(yDiff, 2))

    print "spin!"
    initialRotate = math.atan2(yDiff, xDiff)-theta
    rotate(initialRotate)

    print "move!"
    driveStraight(.1, distance)

    print "spin!"
    finalRotate = yaw-theta
    rotate(finalRotate)

    print "done"

#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():
    driveStraight(.1,0.60)
    rotate(-90)
    driveStraight(.1,0.45)
    rotate(135)
    pass  # Delete this 'pass' once implemented

#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):
    global pub

    r=3.5/100.0
    b=23.0/100.0
    lin_vel=(r/2)*(u1+u2)
    ang_vel=(r/b)*(u1-u2)

    twistMsg=Twist()
    twistMsg.linear.x=lin_vel
    twistMsg.angular.z=ang_vel

    stopMsg=Twist()
    stopMsg.linear.x=0
    stopMsg.angular.z=0


    startTime = rospy.Time.now().secs
    while (rospy.Time.now().secs - startTime < time and not rospy.is_shutdown()):
        pub.publish(twistMsg)

    pub.publish(stopMsg)#Stop when done

#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    global pose

    twistSpeed=Twist()
    twistSpeed.linear.x = speed
    twistSpeed.angular.z = 0

    stopMsg=Twist()
    stopMsg.linear.x=0
    stopMsg.angular.z=0

    initialX = pose.position.x
    initialY = pose.position.y
    atTarget = False

    while(not atTarget):
        currentX = pose.position.x
        currentY = pose.position.y
        currentDistance = math.sqrt(math.pow((currentX-initialX),2)+math.pow((currentY-initialY),2))
        if(currentDistance >= distance):
            atTarget = True
        else:
            pub.publish(twistSpeed)
            rospy.sleep(rospy.Duration(0.15))

    pub.publish(stopMsg)

def rotateDegrees(angle):
    rotate(angle*(math.pi/180))

#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
    global odom_list
    global pose

    transformer = tf.TransformerROS()
    rotation = numpy.array([ [math.cos(angle), -math.sin(angle), 0],
                             [math.sin(angle),  math.cos(angle), 0],
                             [0              ,  0              , 1]])
    odom_list.waitForTransform('odom', 'base_footprint', rospy.Time(0), rospy.Duration(4.0))
    (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
    T_o_t = transformer.fromTranslationRotation(trans, rot)
    R_o_t = T_o_t[0:3,0:3]

    goal_rot = numpy.dot(rotation, R_o_t)
    goal_o = numpy.array([ [goal_rot[0,0], goal_rot[0,1], goal_rot[0,2], T_o_t[0,3]],
                            [goal_rot[1,0], goal_rot[1,1], goal_rot[1,2], T_o_t[1,3]],
                            [goal_rot[2,0], goal_rot[2,1], goal_rot[2,2], T_o_t[2,3]],
                            [0            , 0            , 0            , 1         ]])

    atTarget = False

    while(not atTarget):
        (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
        currentAngle = transformer.fromTranslationRotation(trans, rot)
        inTolerance = abs(currentAngle-goal_o) < 0.1
        if(inTolerance.all()):
            spinWheels(0,0,0)
            atTarget = True
        else:
            if(angle>0):
                spinWheels(1, -1, .15)
            else:
                spinWheels(-1, 1, .15)


#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
    pass  # Delete this 'pass' once implemented

#Bumper Event Callback function
def readBumper(msg):
    print "msg"
    if (msg.state == 1):
        executeTrajectory()

# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
# Start the timer with the following line of code: 
#   rospy.Timer(rospy.Duration(.01), timerCallback)
def timerCallback(event):
    global pose
    pose = Pose()

    (position, orientation) = odom_list.lookupTransform('...','...', rospy.Time(0)) #finds the position and oriention of two objects relative to each other (hint: this returns arrays, while Pose uses lists)
	
    pass # Delete this 'pass' once implemented

def readOdom(odom_data):
    global pose
    pose = odom_data.pose.pose

# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('nchollan_Lab_2_node')

    # These are global variables. Write "global <variable_name>" in any other function to gain access to these global variables 
    global pub
    global pose
    global odom_tf
    global odom_list
   
    # Replace the elipses '...' in the following lines to set up the publishers and subscribers the lab requires
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=5) # Publisher for commanding robot motion
    bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, readBumper, queue_size=5) # Callback function to handle bumper event
    odom_sub = rospy.Subscriber("odom", Odometry, readOdom, queue_size=5)
    navGoal_sub = rospy.Subscriber('move_base_simple/goal1', PoseStamped, navToPose, queue_size=5)

    # Use this object to get the robot's Odometry 
    odom_list = tf.TransformListener()
    odom_tf = tf.TransformBroadcaster()
    
    
    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))
    print "Starting Lab 2"
    while not rospy.is_shutdown():
    	rospy.sleep(rospy.Duration(1))

