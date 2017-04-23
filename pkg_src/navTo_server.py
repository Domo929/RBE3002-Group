#!/usr/bin/env python

import roslib
roslib.load_manifest('DMN_lab3')
import rospy, actionlib

class navToServer:
	def __init__(self):
		self.server = actionlib.SimpleActionServer('move_base', MoveBaseAction, self.execute, False)
		self.server.start()
	def execute(self,goal):
		goal_sub = rospy.Publisher('move_base_simple/goal', PoseStamped, goal, queue_size=100)
		self.server.set_succeeded()


if __name__=='__main__':
	rospy.init_node('navTo_server')
	server = navToServer()
	rospy.spin()