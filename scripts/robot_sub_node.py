
#!/usr/bin/env python

# ROBOT POSITION SUBSCRIBER NODE

import roslib
import rospy
roslib.load_manifest('hallway_sim')
from std_msgs.msg import *
from geometry_msgs.msg import *
from gazebo_msgs import ModelStates
from numpy import *
import time


class robot_sub():

	def __init__(self):

		self.dT = 1;
		self.timenow = time.time()

		self.robot_name = 'floatinghead' # declared in launch file when spawning model
		self.model_i = 0

		self.model_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.modelcallback)

		# create loop
		rospy.Timer(rospy.Duration(self.dT), self.loop, oneshot=False)

		# create text file for storing robot position data
		self.start_time = $1
		self.this_folder = os.path.dirname(__file__)
		self.data_file = os.path.join(this_folder, '..', 'eval', 'robot_{}.txt'.format(self.start_time))
		with open(self.data_file, 'w+') as output:
			output.write('x_pos y_pos\n')


	def loop(self, event):

		self.timenow = time.time()

	def modelcallback(self,data):

		self.model_names = data.name

		for i in range(0,len(model_names)):
			if self.model_names[i] == self.robot_name:
				self.model_i = i

		robot_pose = data.pose[i]

		self.robot_x = robot_pose.position.x
		self.robot_y = robot_pose.position.y

		# push positions to text file for post-processing
		with open(self.data_file, 'w+') as output:
			output.write('{0} {1}\n'.format(self.robot_x, self.robot_y))


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
