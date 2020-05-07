#!/usr/bin/env python

# ROBOT POSITION SUBSCRIBER NODE

import roslib
import rospy
roslib.load_manifest('hallway_sim')
from std_msgs.msg import *
from geometry_msgs.msg import *
from gazebo_msgs.msg import ModelStates, ContactsState, ContactState
from numpy import *
import os
import time
import sys


class robot_sub():

	def __init__(self):

		self.dT = 0.1
		self.timenow = time.time()

		self.robot_name = 'floatinghead' # declared in launch file when spawning model
		self.model_i = 0

		self.model_names = String()
		self.model_poses = Pose()

		self.bumper_states = ContactState()
		self.collision_name = String()

		self.model_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.modelcallback)
		self.bumper_sub = rospy.Subscriber('/bumper_vals', ContactsState, self.bumpercallback)

		# create loop
		rospy.Timer(rospy.Duration(self.dT), self.loop, oneshot=False)

		# create text file for storing robot position data
		self.start_time = rospy.get_param("/start_time")
		self.this_folder = os.path.dirname(__file__)
		self.data_file = os.path.join(self.this_folder, '..', 'eval', '{}_robot.txt'.format(self.start_time))
		with open(self.data_file, 'w+') as output:
			output.write('x_pos y_pos collision\n')


	def loop(self, event):

		for i in range(0,len(self.model_names)):
			if self.model_names[i] == self.robot_name:
				self.model_i = i


		# find robot x and y position
		self.robot_pose = self.model_poses[self.model_i]

		self.robot_x = self.robot_pose.position.x
		self.robot_y = self.robot_pose.position.y

		# parse through bumper data to find relevant collisions
		collisions = len(self.bumper_states)
		self.collision_val = False

		for i in range(0,collisions):
			self.collision_name = self.bumper_states[i].collision2_name
			if "random_hall::base::base_fixed" in self.collision_name:
				self.collision_val = True

		# push positions and collision value to text file for post-processing
		with open(self.data_file, 'a') as output:
			output.write('{0} {1} {2}\n'.format(self.robot_x, self.robot_y, self.collision_val))


	def modelcallback(self,data):

		self.model_names = data.name
		self.model_poses = data.pose

	def bumpercallback(self,data):

		self.bumper_states = data.states


# main function

def main(args):
	rospy.init_node('robot_sub_node', anonymous=True)
	myNode = robot_sub()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"

if __name__ == '__main__':
	main(sys.argv)
