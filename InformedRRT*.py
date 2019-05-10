# DENIM PATEL

#MOTION PLANNING PROJECT


#______________________ import libraries 
# basic libraries
import numpy as np 
import math 

# for visualization purpose
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

import random
import copy 

# __________________________________________




class informedRRTPlanner():

	def __init__(self, start, goal, obstacleList, randArea, expandDis=0.7, goalSampleRate=25, maxIter=50):

		# save x and y position of start 
		self.start = Node(start[0], start[1]) 
 		
 		# save x and y pocosition of goal
 		self.goal = Node(goal[0], goal[1])
		
 		# whare to generate new points? given by user to limit the scope of seach space
		self.minrand = randArea[0]
		self.maxrand = randArea[1]
		
		# expandDis contains the value/distance between new node
		self.expandDis = expandDis

		# heuristic percentage to make system more biased towards goal
		# more the goal sample rate, more heuristic will be applied
		self.goalSampleRate = goalSampleRate

		self.maxIter = maxIter
		self.obstacleList = obstacleList
		self.ellipse = []
	
	def indexofNearestNode(self, nodes, rnd):
		# ////////////////////////////////////////////
		# return the index of the node which is nearest to the currren node
		# //////////////////////////////////////////

		# calculate distance of all the nodes within the list of nodes
		dList = [(node.x - rnd[0])**2 + (node.y - rnd[1])**2 for node in nodes]
		
		# return the index of the node whose value is minimum
		return dList.index(min(dList))

	def computeNewNode(self, theta, nind, nearestNode):
		#///////////////////////////////// 
		# based on the sampled point find new node
		# //////////////////////////////////

		# make a copy of the current node
		newNode = copy.deepcopy(nearestNode)

		# put the value of this node as d*cos(theta) and d*sin(theta)
		newNode.x += self.expandDis * math.cos(theta)
		newNode.y += self.expandDis * math.sin(theta)

		newNode.cost += self.expandDis
		newNode.parent = nind 
		return newNode

	def notInCollision(self, newNode, obstacleList):
		# ////////////////////////////////////
		# returns False means Collision and True means Safe
		#///////////////////////////////////////// 

		# it only implements the circular obstacles
		# ox- x point oy- y point size - radius 
		for (ox, oy, size) in obstacleList:
			dx = ox - newNode.x 
			dy = oy - newNode.y 

			# find the distance from the centre pixel 
			d = dx * dx  + dy * dy

			# here 1.1 is basically a buffer
			if d <= 1.1 *( size**2):
				return False #collision

		return True # safe

	def isNearGoal(self, node):
		# /////////////////////////////
		# returns True is is near to the goal
		# ////////////////////////////////

		# find euclidian distance
		d = self.distance(node, self.goal)
		# check nearness
		if d < self.expandDis:
			return True # node is near to goal

		return False  # node is not near to goal

	def doRewiring(self, newNode, nearInds):
		# //////////////////////////////////
		# rewires, the current node is added to the nearest node of current tree
		# //////////////////////////////////

		# find number of nodes in the list
		nnode = len(self.listOfNode)

		# iterate for number times
		for i in nearInds:
			# take a node from the list and assume it is the nearest node
			nearNode = self.listOfNode[i]

			# calculate distance from first node to the new node
			d = math.sqrt( (nearNode.x - newNode.x)**2 + (nearNode.y - newNode.y)**2 )

			# calculate the cost of nearnode using new node and distance
			scost = newNode.cost + d

			# if newly computed cost is lesser than a particular value
			# rewire----------------
			if nearNode.cost > scost:

				# find angle to propagate into that direction
				theta = math.atan2(newNode.y - nearNode.y , newNode.x - nearNode.x)
				
				# if new point is not in collision otherwise abort the mission
				if self.isCollisionInExtend(nearNode, theta, d):
					
					# change nearnode's parent to newnode
					nearNode.parent = nnode - 1
					
					# update cost
					nearNode.cost = scost
		print("Re-wiring Complete")



	def isCollisionInExtend(self, nearNode, theta, d):
		# ////////////////////////////////////////////
		# return True if it is fully safe to extend till distance d is specified
		# //////////////////////////////////////////

		# make a copy of the node
		tmpNode = copy.deepcopy(nearNode)

		# iterate for small "expandthis" segments
		for i in range(int(d / self.expandDis)):

			# calculate the distance of the new node
			tmpNode.x += self.expandDis * math.cos(theta)
			tmpNode.y += self.expandDis * math.sin(theta)

			# for this new distance, check collision
			if not self.notInCollision(tmpNode, self.obstacleList):
				return False # collision occured

		return True



	def findNearNodes(self, newNode):
		# ///////////////////////////////////
		# finds nodes nearer to the current node
		# ////////////////////////////////////

		# number of nodes into the list
		nnode = len(self.listOfNode)

		r = 50.0 * math.sqrt((math.log(nnode) / nnode))

		# create list that only contains list of distance
		dlist = [(node.x - newNode.x) ** 2 + (node.y - newNode.y) ** 2 for node in self.listOfNode]

		# returns nodes nearer to certain threshold
		nearinds = [dlist.index(i) for i in dlist if i <= r ** 2]

		return nearinds

	def decideParentFrom(self, newNode, nearInds):
		# ////////////////////////////////////
		# returns the node which should be parent of current node
		# ///////////////////////////////////////

		# special case
		if len(nearInds) == 0:
			return newNode 


		dList = []
		# for all the nodes 
		for i in nearInds:
			
			# calculate distance from newnode
			dx = newNode.x - self.listOfNode[i].x
			dy = newNode.y - self.listOfNode[i].y
			d = math.sqrt(dx ** 2 + dy ** 2)

			# angle too
			theta = math.atan2(dy, dx)
			
			# if possible to extend from current goal
			if self.isCollisionInExtend(self.listOfNode[i], theta, d):
				
				# add cost to array
				dList.append(self.listOfNode[i].cost + d)
			else:
				# add infinity
				dList.append(float('inf'))

		# find minimum cost from the array
		minCost = min(dList)

		# find Node whose cost is equal to minimum cost
		minInd = nearInds[dList.index(minCost)]

		# special case -> no possibility to connect to any node 
		if minCost == float('inf'):
			print("mincost is inf")
			return newNode

		# update
		newNode.cost = minCost
		newNode.parent = minInd

		return newNode

	def getPath(self, lastIndex):
		#/////////////////////////////// 
		# returns path
		# ////////////////////////////

		# path contains goal node
		path = [[self.goal.x, self.goal.y]]

		# loop untill start node is reached
		while self.listOfNode[lastIndex].parent is not None:
			# find a node with index
			node = self.listOfNode[lastIndex]

			# add x and y co-ord into array
			path.append([node.x, node.y])
			
			# update index
			lastIndex = node.parent

		#append start node to the list too! 
		path.append([self.start.x, self.start.y])

		# now you have co-ordinates of all the path points
		return path

	def nodeBestNearGoal(self):

		# create an array contains the list of distances from goal point
		disgList = [self.farFromGoal(node.x, node.y) for node in self.listOfNode]

		# array of nodes which are very close to goal node
		goalInds = [disgList.index(i) for i in disgList if i <= self.expandDis]

		# special case
		if len(goalInds) == 0:
			return None 

		# minimum cost out of nodes nearer to goal node
		minCost = min([self.listOfNode[i].cost for i in goalInds])

		# iterate over all the nodes 
		for i in goalInds:
			# return the index of the node whose
			if self.listOfNode[i].cost == minCost:
				return i

		return None 

	def farFromGoal(self, x, y):
		# ////////////////////////////////
		# returns distance from current position to goal position
		# ///////////////////////////////////
		return np.linalg.norm([x - self.goal.x, y - self.goal.y])

	def InformedRRTStarSearch(self, animation=True):
		# ///////////////////////////////////
		# function that performs Informed RRT*
		# ////////////////////////////////////

		# start a list with only start node
		self.listOfNode = [self.start]

		# initialize with worst case
		currentBestCost = float('inf')
		currentBestPathLength = float('inf')
		
		# initialize params
		treeSize = 0
		pathSize = 0
		solutionSet = set()
		path = None

		# calculate the minimum distance possibel
		minimumCost = math.sqrt(pow(self.start.x - self.goal.x, 2) + pow(self.start.y - self.goal.y, 2))
		
		# contains the co-ordinate of the center point connectin start and end point
		centerPointCoord = np.matrix([[(self.start.x + self.goal.x) / 2.0], [(self.start.y + self.goal.y) / 2.0], [0]])

		# find  
		a1 = np.matrix([[(self.goal.x - self.start.x) / minimumCost], [(self.goal.y - self.start.y) / minimumCost], [0]])

		# 
		id1_t = np.matrix([1.0, 0.0, 0.0]) # first column of idenity matrix transposed
		
		# 
		M = np.dot(a1 , id1_t)
		
		# find Singular value decomposition of Matrix M
		U, S, Vh = np.linalg.svd(M, 1, 1)
		
		# C is a 3 x 3 matrixcontains the direction coumn vectors of ellipsoid*
		C = np.dot(np.dot(U, np.diag([1.0, 1.0, np.linalg.det(U) * np.linalg.det(np.transpose(Vh))])), Vh)
		
		# iterate for maximum time
		for i in range(self.maxIter):
			# Sample space is defined by currentBestCost 
			# minimumCost is the minimum distance between the start point and the goal 
			# centerPointCoord is the midpoint between the start and the goal 
			# currentBestCost changes when a new path is found 

			# find new point via "special" sampling
 			rnd = self.newPointSample(currentBestCost, minimumCost, centerPointCoord, C)

 			# find node nearest to this point
			nind = self.indexofNearestNode(self.listOfNode, rnd)
			nearestNode = self.listOfNode[nind]

			# find angle of nearest node 
			theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)
			
			# expand by some distance but
			newNode = self.computeNewNode(theta, nind, nearestNode)
			
			# check the following condition
			dist = self.distance(nearestNode, newNode)
			if self.notInCollision(newNode, self.obstacleList) and self.isCollisionInExtend(nearestNode, theta, dist):
				
				# find indices of nodes nearer to newnode
				nearInds = self.findNearNodes(newNode)

				# newnode is shifted to parentnode
				newNode = self.decideParentFrom(newNode, nearInds)
				print("new Node added")
				self.listOfNode.append(newNode)
				self.doRewiring(newNode, nearInds)

				# check if are nearer to goal
				if self.isNearGoal(newNode):
					# add the last node to set
					solutionSet.add(newNode)

					# last index - will be used for backprop
					lastIndex = len(self.listOfNode) -1 

					# do backprop and save all x and y coord
					tempPath = self.getPath(lastIndex)

					# find length of path
					tempcurrentBestPathLength = self.getLengthofPath(tempPath)
					if tempcurrentBestPathLength < currentBestPathLength:
						
						# save these values for reference
						path = tempPath
						currentBestCost = tempcurrentBestPathLength
						print("[INFO] Shorter Path found")
						print("[INFO] search region reduced")

			if animation:
				self.visualize(rnd)
			
		return path

	def newPointSample(self, cMax, minimumCost, centerPointCoord, C):
		# ////////////////////////////////////////
		# returns random 
		# /////////////////////////////////////////

		if cMax < float('inf'):

			# find 3 values as shown below
			r = [cMax /2.0, math.sqrt(cMax**2 - minimumCost**2)/2.0, math.sqrt(cMax**2 - minimumCost**2)/2.0]

			# find diagonal matrix using above values
			L = np.diag(r)

			# sample unit ball
			xBall = self.sampleInSphere()

			# 
			rnd = np.dot(np.dot(C, L), xBall) + centerPointCoord

			# first two components are fetched and only used
			rnd = [rnd[(0,0)], rnd[(1,0)]]
		else:
			# this will be iterated for only for the first time
			# rnd = self.sampleFreeSpace()
			if random.randint(0,100) > self.goalSampleRate:
				rnd = [random.uniform(self.minrand, self.maxrand),random.uniform(self.minrand, self.maxrand)]
			else:
				rnd = [self.goal.x, self.goal.y]

		return rnd

	def sampleInSphere(self):
		# //////////////////////////////////
		# retuns array of 
		# /////////////////////////////////
																		
		# find two random values
		a = random.random()
		b = random.random()

		# a should be smaller than b
		if b < a:	# if a is big? swap two
			a, b = b, a

		# here as a is always smaller than b,
		# a/b is always smaller then 1
		# theta is always smaller then 2*pi   
		theta = 2 * math.pi * a / b

		# sample value will be always smaller than 1 
		sample = (b * math.cos(theta), b * math.sin(theta))
		# return array of 3 elements for calculations 0 is appended
		return np.array([[sample[0]], [sample[1]], [0]])

	def getLengthofPath(self, path):
		# ///////////////////////////
		# returns length of the path
		# ///////////////////////////

		# initialize the length
		currentBestPathLength = 0

		# for all the [x,y] present in path
		for i in range(1, len(path)): 
			node1_x = path[i][0] # extract first value
			node1_y = path[i][1] # extract second value	
			node2_x = path[i-1][0]
			node2_y = path[i-1][1]

			# calculate euclidian  distance between these two points and add it to total path length
			currentBestPathLength += math.sqrt((node1_x - node2_x)**2 + (node1_y - node2_y)**2)

		return currentBestPathLength

	def distance(self, node1, node2):
		# //////////////////////////////////
		# returns euclidian distance between two points
		# //////////////////////////////////

		return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)



	def visualize(self, rnd=None):
		# //////////////////////////////////////////
		# Draws graph using matplotlib
		# /////////////////////////////////////////////
		
		plt.clf() #clear figure

		if rnd is not None: 
			# show the randomly selected point for exploration
			plt.plot(rnd[0], rnd[1], "-o")
		
		# for all the nodes in list
		for node in self.listOfNode:

			if node.parent is not None: 
				
				if node.x or node.y is not None: 
					
					# for all the valid nodes connect lines between current and parent nodes

					# uncomment the below line to show the lines/edge of the nodes
					# plt.plot([node.x, self.listOfNode[node.parent].x], [node.y, self.listOfNode[node.parent].y], "-b")
					
					# plot the nodes on plot
					plt.plot([node.x],[node.y],'ro')

		# draw all obstacles on graph
		for (ox, oy, size) in self.obstacleList:
			plt.plot(ox, oy, "ok", ms = 30 * size)

		# plot start point
		plt.plot(self.start.x, self.start.y, "g^")
		
		# plot goal point
		plt.plot(self.goal.x, self.goal.y, "g^")
		
		# plot axis
		plt.axis([-1, 30, -1, 30])


		# take pause for better visualization
		plt.pause(0.01)
		

class Node():
# //////////////////////////////
# custom datatype: Node
# ////////////////////////////////

	def __init__(self, x, y):
		self.x = x 
		self.y = y
		self.cost = 0.0 
		self.parent = None 


def main():
    print("Denim Patel - 710078010\n")
    print("informed RRT* algorithm \n")
	

    # DEFINED CUSTOM OBSTACLES
    # x co-ord, y co-ord, radius of the obstacle 
    obstacleList = [
        (1, 23, 1),
        (15, 1, 1),
        (5, 20, 1),
        (3, 5, 1),
        (8, 2, 1), 
        (27, 19, 1),
        (10, 10, 1),
        (20, 20, 1),
        (25, 5, 1),
        (15, 25, 1)
    ]  
    show_animation = True 
    # Set Initial parameters
    infRRT = informedRRTPlanner(start = [4, 28], goal = [10, 2],
              randArea = [-2, 30], obstacleList = obstacleList)
    path = infRRT.InformedRRTStarSearch(animation = show_animation)


    # Draw final path
    if show_animation:
        infRRT.visualize()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-g')
        plt.grid(True)
        plt.pause(0.01)  # Need for Mac
        plt.show()


if __name__ == '__main__':
    main()