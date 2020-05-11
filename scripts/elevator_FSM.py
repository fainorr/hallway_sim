#!/usr/bin/env python

# Elevator FSM Node

import roslib
import rospy
roslib.load_manifest('hallway_sim')
from std_msgs.msg import *
from gazebo_msgs.msg import ContactsState, ContactState
from numpy import *
import time


class Elevator_FSM():
	def __init__(self):

		self.dT = 0.1
		self.timenow = time.time()
		self.oldtime = self.timenow

		self.timenow = rospy.Time.now()

		# subscribe to button contact sensors
		self.outer_button_sub = rospy.Subscriber('/Outer_Button_State', ContactsState, self.outer_callback)
		self.inner_button_sub = rospy.Subscriber('/Inner_Button_State', ContactsState, self.inner_callback)

		self.door_action = rospy.Publisher('/door_action', String, queue_size=1)

		#Create loop
		rospy.Timer(rospy.Duration(self.dT), self.loop, oneshot=False)

		self.Ready = 1
		self.Enter = 0
		self.Moving = 0
		self.Exit = 0

		self.T0_EN = 0
		self.T0 = 0
		self.T1_EN = 0
		self.T1 = 0

		self.wait_time0 = 1
		self.timing_time0 = 0
		self.wait_time1 = 1
		self.timing_time1 = 0

		self.A_time0 = 0
		self.B_time0 = 0
		self.C_time0 = 0
		self.D_time0 = 0
		self.A_time1 = 0
		self.B_time1 = 0
		self.C_time1 = 0
		self.D_time1 = 0

		self.delta_t0 = 0
		self.Start_time0 = 0
		self.delta_t1 = 0
		self.Start_time1 = 0

		self.outer_button_states = ContactState()
		self.inner_button_states = ContactState()

		self.A = 0
		self.B = 0
		self.C = 0
		self.D = 0
		self.E = 0
		self.F = 0
		self.G = 0
		self.H = 0

	def loop(self, event):

		# Block 1

		self.T0_EN = self.Moving
		self.T1_EN = self.Exit

		#--------------------TIMER_0-------------------------
		self.A_time0 = self.wait_time0 and self.T0_EN
		self.B_time0 = self.wait_time0 and not self.T0_EN
		self.C_time0 = self.timing_time0 and not self.T0_EN
		self.D_time0 = self.timing_time0 and self.T0_EN

		self.wait_time0 = self.B_time0 or self.C_time0
		self.timing_time0  =self.A_time0 or self.D_time0

		if(self.A_time0):
			self.Start_time0 = time.time()

		if(self.timing_time0):
			self.delta_t0 = time.time() - self.Start_time0

		else:
			self.delta_t0 = 0

		self.T0 = self.delta_t0 > 5
		#----------------------------------------------------

		#--------------------TIMER_1-------------------------
		self.A_time1 = self.wait_time1 and self.T1_EN
		self.B_time1 = self.wait_time1 and not self.T1_EN
		self.C_time1 = self.timing_time1 and not self.T1_EN
		self.D_time1 = self.timing_time1 and self.T1_EN

		self.wait_time1 = self.B_time1 or self.C_time1
		self.timing_time1  =self.A_time1 or self.D_time1

		if(self.A_time1):
			self.Start_time1 = time.time()

		if(self.timing_time1):
			self.delta_t1 = time.time() - self.Start_time1

		else:
			self.delta_t1 = 0

		self.T1 = self.delta_t1 > 15
		#----------------------------------------------------

		# button presses

		if len(self.outer_button_states) > 0:
			self.outer_button_press = True
		else:
			self.outer_button_press = False

		if len(self.inner_button_states) > 0:
			self.inner_button_press = True
		else:
			self.inner_button_press = False

		# Block 2

		self.A = self.Ready and not self.outer_button_press
		self.B = self.Ready and self.outer_button_press
		self.C = self.Enter and not self.inner_button_press
		self.D = self.Enter and self.inner_button_press
		self.E = self.Moving and not self.T0
		self.F = self.Moving and self.T0
		self.G = self.Exit and not self.T1
		self.H = self.Exit and self.T1

		# Block 3

		self.Ready = self.A or self.H
		self.Enter = self.B or self.C
		self.Moving = self.D or self.E
		self.Exit = self.F or self.G


		# Block 4
		if self.Enter or self.Exit:
			action = 'open'

		if self.Ready or self.Moving:
			action = 'close'

		self.door_action.publish(action)


	def outer_callback(self,data):
		self.outer_button_states = data.states

	def inner_callback(self,data):
		self.inner_button_states = data.states


# main function

def main(args):
	rospy.init_node('elevator_FSM', anonymous=True)
	myNode = Elevator_FSM()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"

if __name__ == '__main__':
	main(sys.argv)