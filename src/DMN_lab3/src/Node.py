#!/usr/bin/env python

import numpy as np
from enum import Enum

#Change to show Mike something

#TODO see if this is actually how Python Enum works
TypeNode = Enum(Start, Goal, Wall, Frontier, Unknown)

class Node(object):
	
	def __init__(self, x, y, typeOfNode):
		self.x = x
		self.y = y
		self.gCost = 0
		self.hCost = 0
		self.fCost = 0
		self.typeOfNode = typeOfNode


	#Uses Manhattan for G and Direct for H
	def findF(self, start, end):
		self.fCost = findG(start, self) + findH(self, end)
		return  self.fCost
	#finds the known distance using Manhattan
	def findG(self, goal):
		self.gcost = findManhattan(self, goal)
		return self.gCost

	#Finds the Heuristic Distance using direct
	def findH(self, goal):
		self.hCost = findDirect(self, goal)
		return self.hCost

	#determines distance between two nodes using the Manhattan distance
	def findManhattan(self, nodeToCheck):
		Xdif = np.absolute(nodeToCheck.x - self.x)
		yDif = np.absolute(nodeToCheck.y - self.y)
		return Xdif + yDif

	#finds the distance between two nodes using the Direct Distance
	def findDirect(self, nodeToCheck):
		Xdif = nodeToCheck.x - self.x
		yDif = nodeToCheck.y - self.y
		val = np.sqrt(Xdif**2 + yDif**2)
		return val

		
		