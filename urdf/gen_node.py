#!/usr/bin/env python

import roslib
import rospy
import sys
import os
roslib.load_manifest('hallway_sim')
from numpy import *
import time

import hallway_gen

class gen_node():

	def __init__(self):

		self.dT = 1.0
		self.timenow = time.time()

		file_folder = os.path.dirname(os.path.abspath(__file__))
		my_file = os.path.join(file_folder, 'hallway_gen.py')
		execfile(my_file)
		rospy.logwarn("generated new hallway")

		# create loop
		rospy.Timer(rospy.Duration(self.dT), self.loop, oneshot=False)

	def loop(self, event):

		self.timenow = time.time()

# main function

def main(args):
	rospy.init_node('gen_node', anonymous=True)
	myNode = gen_node()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"

if __name__ == '__main__':
	main(sys.argv)
