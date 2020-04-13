#!/usr/bin/env python

import roslib
import rospy
roslib.load_manifest('hallway_sim')
from numpy import *

import hallway_gen

class gen_node():

	def __init__(self):

        execfile('hallway_gen.py')

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
