#!/usr/bin/env python

# LIDAR ANALYSIS NODE

import roslib
import rospy
roslib.load_manifest('hallway_sim')
from std_msgs.msg import *
from sensor_msgs.msg import *
from numpy import *
import os
import time
import sys

import lidar_compare


class lidar_quad():

	def __init__(self):

		self.dT = 0.1
		self.timenow = time.time()
		self.oldtime = self.timenow

		self.obst_size = 3;         # number of consecutive dots
		self.safe_range = 1.2;      	# search ranges for obstacles

		self.distances = zeros(360)
		self.angle_parameters = [0.0, 0.0, 0.0]

		self.old_action = "stand"
		self.old_direction = "left"

		self.analyze = lidar_compare.lidar_compare()

		# subscribe to rplidar node
		self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.scancallback)
		self.FSM_action = rospy.Subscriber('/action', String, self.actioncallback)
		self.FSM_direction = rospy.Subscriber('/direction', String, self.directioncallback)

		# publish array of booleans if an obstacle exists in each quadrant
		self.FSM_action = rospy.Publisher('/action', String, queue_size=1)
		self.FSM_direction = rospy.Publisher('/direction', String, queue_size=1)

		# create loop
		rospy.Timer(rospy.Duration(self.dT), self.loop, oneshot=False)

		# create text file for storing navigation parameters
		self.start_time = rospy.get_param("/start_time")
		self.this_folder = os.path.dirname(__file__)
		self.data_file = os.path.join(self.this_folder, '..', 'eval', '{}_nav.txt'.format(self.start_time))
		with open(self.data_file, 'w+') as output:
			output.write('obstacle_size {0}\n'.format(self.obst_size))
			output.write('safe_range {0}\n'.format(self.safe_range))
			output.write('update_rate {0}\n'.format(self.dT))


	def loop(self, event):

		self.timenow = time.time()
		self.oldtime = self.timenow

		self.command_history = [self.old_action, self.old_direction]
		self.action, self.direction = self.analyze.find_optimal_action(self.distances, self.angle_parameters, self.obst_size, self.safe_range, self.command_history)

		self.action_msg = String()
		self.action_msg.data = self.action

		self.direction_msg = String()
		self.direction_msg.data = self.direction

		self.FSM_action.publish(self.action_msg)
		self.FSM_direction.publish(self.direction_msg)


	def scancallback(self,data):

		self.distances = array(data.ranges)
		self.angle_parameters = [data.angle_min, data.angle_max, data.angle_increment]

	def actioncallback(self,data):

		self.old_action = data.data

	def directioncallback(self,data):

		self.old_direction = data.data


# main function

def main(args):
	rospy.init_node('lidar_quad_node', anonymous=True)
	myNode = lidar_quad()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"

if __name__ == '__main__':
	main(sys.argv)
