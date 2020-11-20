#!/usr/bin/env python
import pdb
import math
import numpy as np
from numpy.linalg import inv
from collections import deque 
#Input:


class lp_filter():

	def __init__(self):
		self.alpha = 0.25 #default
		self.old = 0.0
		self.new = 0.0
		self.inited = False



	def init_filter(self, val):
		self.new = val
		self.old = val
		self.inited = True

	def set_const(self,lim):
		self.alpha = lim

	def set_new(self, val):
		self.new = val

	def FilterCompute(self):
		self.old = self.old*(1-self.alpha)+self.new*self.alpha
		return self.old

			






