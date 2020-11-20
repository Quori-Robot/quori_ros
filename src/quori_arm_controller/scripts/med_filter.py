#!/usr/bin/env python
import pdb
import math
import numpy as np
from numpy.linalg import inv
from collections import deque 
#Input:


class med_filter():

	def __init__(self,size):
		self.que = deque([0]*size) # [newest, ..., oldest]
		self.inited = False
		self.result = 0.0

	def init_filter(self, val):
		self.que = deque([val]*len(self.que))
		self.inited = True

	def ComputeFrmNew(self,val):
		self.que.rotate(1) # shift values to the right. the oldest is now at [0]
		self.que[0] = val #replace oldest with newest
		self.result = np.median(self.que)
		return self.result