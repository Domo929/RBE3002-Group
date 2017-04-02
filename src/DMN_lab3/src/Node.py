#!/usr/bin/env python

import rospy
import numpy as np
from enum import Enum

TypeNode = Enum('Start', 'Goal', 'Wall', 'Fronier')

class Node(object):
	"""docstring for Node"""
	def __init__(self, x, y, loc, typeOfNode):
		self.x = x
		self.y = y
		self.gcost = 0
		self.hcost = 0
		self.fcost = 0
		self.typeOfNode = typeOfNode


	#
	def findF(self, start, end):
		F = findG(start, end) + findH(self, end)
		self.fcost = F
		return F

	
	def findG(self, start, end):
		pass

	#Finds the Heuristic Distance
	def findH(self, goal):
		H = findManhattan(self, goal)
		self.h = H
		return H

	
	def findManhattan(self, nodeToCheck):
		Xdif = nodeToCheck.x - self.x
		yDif = nodeToCheck.y - self.y
		return Xdif + yDif

	
	def findDirect(self, nodeToCheck):
		Xdif = nodeToCheck.x - self.x
		yDif = nodeToCheck.y - self.y
		val = np.sqrt(Xdif**2 + yDif**2)
		return val

		
		