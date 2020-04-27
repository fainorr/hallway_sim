#!/usr/bin/env python

# PERSON CONTROL NODE

import roslib
import rospy
roslib.load_manifest('hallway_sim')
from std_msgs.msg import *
from geometry_msgs.msg import *
from numpy import *
import random
import time


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

        velocity = random.normal(1.4,0.2)
        angle_deg = random.randint(1,360)
        angle = float(angle_deg)*pi/180

        self.v1.linear.x = velocity*cos(angle);
        self.v1.linear.y = velocity*sin(angle);
        self.v1.linear.z = 0.0;
        self.v1.angular.x = 0.0;
        self.v1.angular.y = 0.0;
        self.v1.angular.z = 0.0;

        velocity = random.normal(1.4,0.2)
        angle_deg = random.randint(1,360)
        angle = float(angle_deg)*pi/180

        self.v2.linear.x = velocity*cos(angle);
        self.v2.linear.y = velocity*sin(angle);
        self.v2.linear.z = 0.0;
        self.v2.angular.x = 0.0;
        self.v2.angular.y = 0.0;
        self.v2.angular.z = 0.0;

        velocity = random.normal(1.4,0.2)
        angle_deg = random.randint(1,360)
        angle = float(angle_deg)*pi/180

        self.v3.linear.x = velocity*cos(angle);
        self.v3.linear.y = velocity*sin(angle);
        self.v3.linear.z = 0.0;
        self.v3.angular.x = 0.0;
        self.v3.angular.y = 0.0;
        self.v3.angular.z = 0.0;


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
