#!/usr/bin/env python

from numpy import *

# ----------------------
# ELEVATOR DOOR POSITION
# ----------------------

class door_position():

	def __init__(self):

		self.x_start = 0
		self.y_start = 1.125
		self.z_start = 1.5

	def location(self, action):

		if (action == "close"):

			self.x = self.x_start + 0.625
			self.y = self.y_start
			self.z = self.z_start

		if (action == "open"):

			self.x = self.x_start - 0.625
			self.y = self.y_start
			self.z = self.z_start

		return self.x, self.y, self.z
