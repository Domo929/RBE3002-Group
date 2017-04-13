#!/usr/bin/env python

import rospy, tf, numpy, math
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

wheel_rad = 3.5 / 100.0 #cm
wheel_base = 23.0 / 100.0 #cm

def sendMoveMsg(linearVelocity, angularVelocity):
    """Send a movement (twist) message."""
    global pub
    msg = Twist()
    msg.linear.x = linearVelocity
    msg.angular.z = angularVelocity
    pub.publish(msg)


def navToPose(goal):
    """Drive to a goal subscribed to from /move_base_simple/goal"""
    #compute angle required to make straight-line move to desired pose
    global xPosition
    global yPosition
    global theta
    #capture desired x and y positions
    desiredY = goal.pose.position.y
    desiredX = goal.pose.position.x

    #capture desired angle
    quat = goal.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    desiredT = yaw * (180.0/math.pi)
    #compute distance to target
    distance = math.sqrt(math.pow((desiredX - xPosition), 2) + math.pow((desiredY - yPosition), 2))
    adjustedX = (goal.pose.position.x - xPosition)
    adjustedY = (goal.pose.position.y - yPosition)
    print "goal position",goal.pose.position.x, goal.pose.position.y
    print "current position", xPosition, yPosition
    print "difference in positions",adjustedX, adjustedY
    #compute initial turn amount
    initialTurn = (math.atan2(adjustedY, adjustedX) * (180 / math.pi)) - theta

    print "moving from (" + str(xPosition) + ", " + str(yPosition) + ") @ " + str(theta) + " degrees"
    print "moving to (" + str(desiredX) + ", " + str(desiredY) + ") @ " + str(desiredT) + " degrees"
    print "distance: " + str(distance) + ", initial turn: " + str(initialTurn)
    print "spin!" #turn to calculated angle
    rotateDegrees(initialTurn)
    print "move!" #move in straight line specified distance to new pose
    driveStraight(0.15, distance)
    rospy.sleep(2)
    print "spin!" #spin to final angle
    finalTurn = desiredT - theta
    print "rotate " + str(finalTurn) +  " to " + str(desiredT)
    rotateDegrees(finalTurn)
    print "done"
    return "Done"


def executeTrajectory():
    """This function sequentially calls methods to perform a trajectory."""
    driveStraight(1, 0.6)
    rotateDegrees(-90)
    driveStraight(1, .45)
    rotate(135)


def spinWheels(u1, u2, time):
    """This function accepts two wheel velocities and a time interval."""
    global pub

    r = wheel_rad
    b = wheel_base
    #compute wheel speeds
    u = (r / 2) * (u1 + u2)
    w = (r / b) * (u1 - u2)
    start = rospy.Time().now().secs
    #create movement and stop messages
    move_msg = Twist()
    move_msg.linear.x = u
    move_msg.angular.z = w
    stop_msg = Twist()
    stop_msg.linear.x = 0
    stop_msg.angular.z = 0
    #publish move message for desired time
    while(rospy.Time().now().secs - start < time and not rospy.is_shutdown()):
        pub.publish(move_msg)
    pub.publish(stop_msg)


def driveStraight(speed, distance):
    """This function accepts a speed and a distance for the robot to move in a straight line"""
    global pose
    initialX = pose.pose.position.x
    initialY = pose.pose.position.y

    atTarget = False
    while (not atTarget and not rospy.is_shutdown()):
        currentX = pose.pose.position.x
        currentY = pose.pose.position.y
        currentDistance = math.sqrt(math.pow((currentX - initialX), 2) + math.pow((currentY - initialY), 2))
        #currentDistance *=3.3
        print("current distance",currentDistance)
        if (currentDistance >= distance):
            atTarget = True
            sendMoveMsg(0, 0)
        else:
            sendMoveMsg(speed, 0)
            rospy.sleep(0.15)


def driveSmooth(speed, distance):
    """This function accepts a speed and a distance for the robot to move in a smoothed straight line."""
    global pose

    initialX = pose.pose.position.x
    initialY = pose.pose.position.y
    atTarget = False
    rampSpeed = 0.0
    sleepTime = 0.05
    rampPercentage = 0.3
    step = speed / ((rampPercentage * (distance / speed)) / sleepTime)
    print "Step size: " + str(step)
    while (not atTarget and not rospy.is_shutdown()):
        currentX = pose.pose.position.x
        currentY = pose.pose.position.y
        currentDistance = math.sqrt(math.pow((currentX - initialX), 2) + math.pow((currentY - initialY), 2))
        if (currentDistance >= distance):
            atTarget = True
            sendMoveMsg(0, 0)
        else:
            if ((distance - currentDistance) <= distance * rampPercentage and rampSpeed >= 0):
                rampSpeed -= step
                sendMoveMsg(rampSpeed, 0)
            elif ((distance - currentDistance) >= distance * (1.0 - rampPercentage) and rampSpeed <= speed):
                rampSpeed += step
                sendMoveMsg(rampSpeed, 0)
            else:
                sendMoveMsg(speed, 0)
            rospy.sleep(sleepTime)

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
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2] + 3.14
        #print yaw

    stop = Twist()#all values default to 0 so this msg will stop the robot
    pub.publish(stop)
# def rotate(angle):
#     """Accepts an angle and makes the robot rotate around it."""
#     global odom_list
#     global pose

#     #This node was created using Coordinate system transforms and numpy arrays.
#     #The goal is measured in the turtlebot's frame, transformed to the odom.frame 
#     transformer = tf.TransformerROS()   
#     rotation = numpy.array([[math.cos(angle), -math.sin(angle), 0], #Create goal rotation
#                             [math.sin(angle), math.cos(angle), 0],
#                             [0,          0,          1]])

#     #Get transforms for frames
#     odom_list.waitForTransform('odom', 'base_footprint', rospy.Time(0), rospy.Duration(4.0))
#     (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
#     T_o_t = transformer.fromTranslationRotation(trans, rot)
#     R_o_t = T_o_t[0:3,0:3]

#     #Setup goal matrix
#     goal_rot = numpy.dot(rotation, R_o_t)
#     goal_o = numpy.array([[goal_rot[0,0], goal_rot[0,1], goal_rot[0,2], T_o_t[0,3]],
#                     [goal_rot[1,0], goal_rot[1,1], goal_rot[1,2], T_o_t[1,3]],
#                     [goal_rot[2,0], goal_rot[2,1], goal_rot[2,2], T_o_t[2,3]],
#                     [0,             0,             0,             1]])

#     #Continues creating and matching coordinate transforms.
#     done = False
#     while (not done and not rospy.is_shutdown()):
#         (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
#         state = transformer.fromTranslationRotation(trans, rot)
#         within_tolerance = abs((state - goal_o)) < .1
#         if ( within_tolerance.all() ):
#             spinWheels(0,0,0)
#             done = True
#         else:
#             if (angle > 0):
#                 spinWheels(1,-1,.1)
#             else:
#                 spinWheels(-1,1,.1)

def rotateDegrees(angle):
    """Rotate and angle in degrees."""
    rotate(angle * (math.pi / 180))

def readBumper(msg):
    """Bumper event callback"""
    if (msg.state == 1):
        # What should happen when the bumper is pressed?
        #Stop forward motion if bumper is pressed
        print "Bumper pressed!"
        executeTrajectory()


#Odometry Callback function.
def readOdom(msg):
    """Read odometry messages and store into global variables."""
    global pose
    global xPosition
    global yPosition
    global theta
    global odom_list
    global odom_tf
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
        xPosition = trans[0]
        yPosition = trans[1]
    except:
        print "Waiting for tf..."


# This is the program's main function
if __name__ == '__main__':
    rospy.init_node('sample_Lab_2_node')
#def initDrivingCode():
    global pub
    global pose
    global odom_list
    global odom_tf
    #navServer = rospy.Service('/nav_to_pose', PoseStamped, navToPose)
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size=10) # Publisher for commanding robot motion
    bumper_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events
    goal_sub = rospy.Subscriber('move_base_simple/goal1', PoseStamped, navToPose, queue_size=100)
    sub = rospy.Subscriber('/odom', Odometry, readOdom)
    odom_list = tf.TransformListener()
    odom_tf = tf.TransformBroadcaster()
    odom_tf.sendTransform((0, 0, 0),(0, 0, 0, 1),rospy.Time.now(),"base_footprint","odom")

    while not rospy.is_shutdown():
        rospy.spin()
    
    print "Lab 2 complete!"