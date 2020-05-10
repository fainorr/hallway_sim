#!/usr/bin/env python

# ELEVATOR CONTROL NODE

import roslib
import rospy
roslib.load_manifest('hallway_sim')
from std_msgs.msg import *
from numpy import *
import time

import door_position

class elevator_controller():

	def __init__(self):

		rospy.logwarn("started elevator controller")

		self.dT = 0.1
		self.timenow = time.time()
		self.starttime = time.time()
		self.time_elapsed = 0.0

		# subscribe to action and button state
		self.FSM_action = rospy.Subscriber('/door_action', String, self.actioncallback)

		# publish door position
		self.door_location = rospy.Publisher('/elevator/door_to_ground_joint_position_controller/command', Float64, queue_size=1)
		self.action = 'close'

		self.x = 0.0
		self.y = 0.0
		self.z = 0.0

		# create loop
		rospy.Timer(rospy.Duration(self.dT), self.loop, oneshot=False)


	def loop(self, event):

		self.DP = door_position.door_position()
		self.x, self.y, self.z = self.DP.location(self.action)

		self.door_location.publish(self.x)

		self.timenow = time.time()
		self.oldtime = self.timenow


	def actioncallback(self,data):

		self.action = data.data

def main(args):
	rospy.init_node('elevator_control_node', anonymous=True)
	myNode = elevator_controller()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting Down"

if __name__ == '__main__':
	main(sys.argv)
