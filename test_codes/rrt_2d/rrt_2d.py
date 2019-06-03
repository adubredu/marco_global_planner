#!/usr/bin/env python3



# Author: Mark Moll
import numpy as np 
from PIL import Image
from matplotlib import pyplot as plt
try:
	from ompl import base as ob
	from ompl import geometric as og
except ImportError:
 # if the ompl module is not in the PYTHONPATH assume it is installed in a
 # subdirectory of the parent directory called "py-bindings."
	from os.path import abspath, dirname, join
	import sys
	sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
	from ompl import base as ob
	from ompl import geometric as og

'''
img = Image.open('/home/alphonsus/ompl_code/test_codes/rrt_2d/rrt.png')
imageData = np.array(img, dtype='uint8')
print(imageData.shape)
plt.imshow(imageData)
plt.show()
'''

class Plane2dEnvironment:

	def __init__(self):
		img = Image.open('/home/alphonsus/ompl_code/test_codes/rrt_2d/rrt.png')
		self.env = np.array(img, dtype='uint8')
		self.maxWidth = self.env.shape[0] - 1
		self.maxHeight = self.env.shape[1] - 1

		# bounds = ob.RealVectorBounds(2)
		# bounds.setLow(0,0); bounds.setHigh(0,self.env.shape[0])
		# bounds.setLow(1,0); bounds.setHigh(1,self.env.shape[1])
		# self.space = ob.SE2StateSpace()

		self.space = ob.RealVectorStateSpace()
		self.space.addDimension(0.0, self.env.shape[0])
		self.space.addDimension(0.0, self.env.shape[1])

		# self.space.setBounds(bounds)


		self.ss_ = og.SimpleSetup(self.space)
		self.ss_.setStateValidityChecker(ob.StateValidityCheckerFn(self.isStateValid))

		self.space.setup()

		self.ss_.getSpaceInformation().setStateValidityCheckingResolution(1.0/self.space.getMaximumExtent())
		self.ss_.setPlanner(og.RRTConnect(self.ss_.getSpaceInformation()))
		self.ss_.setup()
		

	def plan(self,start_row, start_col, goal_row, goal_col):
		start = ob.State(self.ss_.getStateSpace())
		start()[0] = start_row
		start()[1] = start_col
		# start().setX(start_row)
		# start().setY(start_col)

		goal = ob.State(self.ss_.getStateSpace())
		goal()[0] = goal_row
		goal()[1] = goal_col

		self.ss_.setStartAndGoalStates(start,goal)
		self.ss_.setup()

		solved =  self.ss_.solve()

		if solved:
			print('Found %d solutions' %self.ss_.getProblemDefinition().getSolutionCount())

			if (self.ss_.haveSolutionPath()):
				self.ss_.simplifySolution()
				p = self.ss_.getSolutionPath()
				ps = og.PathSimplifier(self.ss_.getSpaceInformation())
				ps.simplifyMax(p)
				ps.smoothBSpline(p)
				# self.ss_.getPathSimplifier().simplifyMax(p)
				# self.ss_.getPathSimplifier().smoothBSpline(p)

				return True
		return False


	def recordSolution(self):
		if(not self.ss_ or not self.ss_.haveSolutionPath()):
			return False

		p = self.ss_.getSolutionPath()
		p.interpolate()
		print(p)

		for i in range(p.getStateCount()):
			# print (p.getState(i).printAsMatrix())
			w = min(self.maxWidth, int(p.getState(i)[0]))
			h = min(self.maxHeight, int(p.getState(i)[1]))
			self.env[h,w,0] = 255
			self.env[h,w,1] = 0
			self.env[h,w,2] = 0


	def save(self,filename):
		if not self.ss_:
			return
		im = Image.fromarray(self.env)
		im.save(filename)



	def isStateValid(self,state):
		w = min(int(state[0]), self.maxWidth)
		h = min(int(state[1]), self.maxHeight)

		c = self.env[h,w]
		return c[0] > 127 and c[1] > 127 and c[2] > 127



if __name__=='__main__':
	path = Plane2dEnvironment()
	path.plan(0,0,400,400)
	path.recordSolution()
	path.save('soln.png')

