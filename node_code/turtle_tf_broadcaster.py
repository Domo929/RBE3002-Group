#!/usr/bin/env python  
import roslib, rospy, tf
from nav_msgs.msg import Odometry

def handle_turtle_pose(msg):
    br = tf.TransformBroadcaster()
    pos = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
    ori = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
    br.sendTransform(pos,ori,rospy.Time.now(),'base_footprint','/odom')

if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')
    rospy.Subscriber('/odom',
                     Odometry,
                     handle_turtle_pose)
    rospy.spin()