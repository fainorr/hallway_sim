#!/usr/bin/env python

# Elevator FSM Node

import roslib
import rospy
roslib.load_manifest('hallway_sim')
from std_msds.msg import *
from numpy import *
import time



class Elevator_FSM():
	def __init__(self):

		self.dT = 0.005
		self.timenow = time.time()
		self.oldtime = self.timenow

		self.timenow = rospy.Time.now()

		# Publishers here



		#Create loop
		rospy.Timer(rospy.Duration(self.dT), self.loop, oneshot=False)

		self.Ready = 1
		self.Open = 0
		self.Close = 0
		self.Moving = 0
		self.Reopen = 0
		self.Reclose = 0

		self.T0_EN = 0
		self.T0 = 0
		self.T1_EN = 0
		self.T1 = 0
		self.T2_EN = 0
		self.T2 = 0

		self.wait_time0 = 1
		self.timing_time0 = 0
		self.wait_time1 = 1
		self.timing_time1 = 0
		self.wait_time2 = 1
		self.timing_time2 = 0

		self.A_time0 = 0
		self.B_time0 = 0
		self.C_time0 = 0
		self.D_time0 = 0
		self.A_time1 = 0
		self.B_time1 = 0
		self.C_time1 = 0
		self.D_time1 = 0
		self.A_time2 = 0
		self.B_time2 = 0
		self.C_time2 = 0
		self.D_time2 = 0

		self.delta_t0 = 0
		self.Start_time0 = 0
		self.delta_t1 = 0
		self.Start_time1 = 0
		self.delta_t2 = 0
		self.Start_time2 = 0

		self.A = 0
		self.B = 0
		self.C = 0
		self.D = 0
		self.E = 0
		self.F = 0
		self.G = 0
		self.H = 0
		self.I = 0
		self.J = 0
		self.K = 0
		self.L = 0

	def loop(self, event):

		# Block 1

		self.T0_EN = self.Open
		self.T1_EN = self.Moving
		self.T2_EN = self.Reopen

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

		#--------------------TIMER_0-------------------------
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

		self.T1 = self.delta_t1 > 5
		#----------------------------------------------------

		#--------------------TIMER_0-------------------------
		self.A_time2 = self.wait_time2 and self.T2_EN
		self.B_time2 = self.wait_time2 and not self.T2_EN
		self.C_time2 = self.timing_time2 and not self.T2_EN
		self.D_time2 = self.timing_time2 and self.T2_EN

		self.wait_time2 = self.B_time2 or self.C_time2
		self.timing_time2  =self.A_time2 or self.D_time2

		if(self.A_time2):
			self.Start_time2 = time.time()

		if(self.timing_time2):
			self.delta_t2 = time.time() - self.Start_time2

		else:
			self.delta_t2 = 0

		self.T2 = self.delta_t2 > 5
		#----------------------------------------------------

		# Block 2

		self.A = self.Ready and not self.button_press
		self.B = self.Ready and self.button_press
		self.C = self.Open and not self.T0
		self.D = self.Open and self.T0
		self.E = self.Close and not self.closed
		self.F = self.Close and self.closed
		self.G = self.Moving and not self.T1
		self.H = self.Moving and self.T1
		self.I = self.Reopen and not self.T2
		self.J = self.Reopen and self.T2
		self.K = self.Reclose and not self.closed
		self.L = self.Reclose and self.closed

		# Block 3

		self.Ready = self.A or self.L
		self.Open = self.B or self.C
		self.Close = self.D or self.E
		self.Moving = self.F or self.G
		self.Reopen = self.H or self.I
		self.Reclose = self.J or self.K