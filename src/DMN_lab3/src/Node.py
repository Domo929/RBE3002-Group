#!/usr/bin/env python

import rospy
import numpy as np

class Node(object):
	"""docstring for Node"""
	def __init__(self, x, y, loc, typeOfNode):
		self.x = x
		self.y = y
		self.gcost = 0
		self.hcost = 0
		self.fcost = 0

	def findF(self, start, end):
		F = findG(start, end) + findH(start, end)
		self.fcost = F
		return F

	def findG(self, start, end):
	def findH(self, start, end):

	def findManhattan(self, nodeToCheck):
		Xdif = nodeToCheck.x - self.x
		yDif = nodeToCheck.y - self.y
		return Xdif + yDif

	def findDirect(self, nodeToCheck):
		Xdif = nodeToCheck.x - self.x
		yDif = nodeToCheck.y - self.y
		val = np.sqrt(Xdif**2 + yDif**2)
		return val

		
		