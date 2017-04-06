#!/usr/bin/env python

import numpy as np

# -1 Obstacle
# 0 Unexplored
# 1 Explored
# 2 Fronteir

class Node(object):
	
	def __init__(self, x, y, state):
		self.x = x
		self.y = y
		self.state = state
		self.gCost = 0
		self.hCost = 0
		self.fCost = 0
		self.parent = None

	#Uses Manhattan for G and Direct for H
	def findF(self, start, end):
		self.fCost = self.findG(start) + self.findH(end)
		return  self.fCost

	#finds the known distance using Manhattan
	def findG(self, start):
		self.gCost = self.findManhattan(start)
		print("gCost",self.gCost)
		return self.gCost

	#Finds the Heuristic Distance using direct
	def findH(self, end):
		self.hCost = self.findDirect(end)
		print("hCost",self.hCost)
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

	

		
		