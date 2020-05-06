#!/usr/bin/env python

# GAZEBO DRIVE NODE

import roslib
import rospy
roslib.load_manifest('hallway_sim')
from std_msgs.msg import *
from geometry_msgs.msg import *
from numpy import *
import time

class gazebo_drive():

	def __init__(self):

		self.dT = 0.01;
		self.timenow = time.time()
		self.oldtime = self.timenow

		self.action = "stand"
		self.direction = "left"

		self.FSM_action = rospy.Subscriber('/action', String, self.actioncallback)
		self.FSM_direction = rospy.Subscriber('/direction', String, self.directioncallback)

		self.robot_velocity = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

		# create loop
		rospy.Timer(rospy.Duration(self.dT), self.loop, oneshot=False)

	def loop(self, event):

		self.twist = Twist()
		if self.action == "stand":
			self.twist.linear.x = 0.0;
			self.twist.linear.y = 0.0;
			self.twist.linear.z = 0.0;
			self.twist.angular.x = 0.0;
			self.twist.angular.y = 0.0;
			self.twist.angular.z = 0.0;

		if self.action == "forward":
			self.twist.linear.x = 1.0;
			self.twist.linear.y = 0.0;
			self.twist.linear.z = 0.0;
			self.twist.angular.x = 0.0;
			self.twist.angular.y = 0.0;
			self.twist.angular.z = 0.0;

		if self.action == "turn":
			if self.direction == "left":
				self.twist.linear.x = 0.0;
				self.twist.linear.y = 0.0;
				self.twist.linear.z = 0.0;
				self.twist.angular.x = 0.0;
				self.twist.angular.y = 0.0;
				self.twist.angular.z = pi/4.0;
			else:
				self.twist.linear.x = 0.0;
				self.twist.linear.y = 0.0;
				self.twist.linear.z = 0.0;
				self.twist.angular.x = 0.0;
				self.twist.angular.y = 0.0;
				self.twist.angular.z = -pi/4.0;

		self.robot_velocity.publish(self.twist)


	def actioncallback(self,data):

		self.action = data.data

	def directioncallback(self,data):

		self.direction = data.data


# main function

def main(args):
	rospy.init_node('gazebo_drive_node', anonymous=True)
	myNode = gazebo_drive()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"

if __name__ == '__main__':
	main(sys.argv)
