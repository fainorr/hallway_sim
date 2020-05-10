#!/usr/bin/env python

import roslib
import rospy
roslib.load_manifest('hallway_sim')
from std_msgs.msg import *
from geometry_msgs.msg import *
from numpy import *
import time

# -------------------
# PERSON CONTROL NODE
# -------------------

# this node publishes linear and angular velocities for each "person" in the
# gazebo world; the number of publishers must match the number of people spawned

class person_drive():

	def __init__(self):

		self.dT = 3;
		self.timenow = time.time()
		self.oldtime = self.timenow

		self.velocity_person1 = rospy.Publisher('/vel_person1', Twist, queue_size=1)
		self.velocity_person2 = rospy.Publisher('/vel_person2', Twist, queue_size=1)
		self.velocity_person3 = rospy.Publisher('/vel_person3', Twist, queue_size=1)

		# create loop
		rospy.Timer(rospy.Duration(self.dT), self.loop, oneshot=False)

	def loop(self, event):

		self.v1 = Twist()
		self.v2 = Twist()
		self.v3 = Twist()

		self.velocity = random.normal(0.8,0.2)
		self.angle_deg = random.randint(1,360)
		self.angle = float(self.angle_deg)*pi/180

		self.v1.linear.x = self.velocity*cos(self.angle)
		self.v1.linear.y = self.velocity*sin(self.angle)
		self.v1.linear.z = 0.0
		self.v1.angular.x = 0.0
		self.v1.angular.y = 0.0
		self.v1.angular.z = 0.0

		self.velocity = random.normal(0.8,0.2)
		self.angle_deg = random.randint(1,360)
		self.angle = float(self.angle_deg)*pi/180

		self.v2.linear.x = self.velocity*cos(self.angle)
		self.v2.linear.y = self.velocity*sin(self.angle)
		self.v2.linear.z = 0.0
		self.v2.angular.x = 0.0
		self.v2.angular.y = 0.0
		self.v2.angular.z = 0.0

		self.velocity = random.normal(0.8,0.2)
		self.angle_deg = random.randint(1,360)
		self.angle = float(self.angle_deg)*pi/180

		self.v3.linear.x = self.velocity*cos(self.angle)
		self.v3.linear.y = self.velocity*sin(self.angle)
		self.v3.linear.z = 0.0
		self.v3.angular.x = 0.0
		self.v3.angular.y = 0.0
		self.v3.angular.z = 0.0

		self.velocity_person1.publish(self.v1)
		self.velocity_person2.publish(self.v2)
		self.velocity_person3.publish(self.v3)



# main function

def main(args):
	rospy.init_node('person_control_node', anonymous=True)
	myNode = person_drive()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"

if __name__ == '__main__':
	main(sys.argv)
