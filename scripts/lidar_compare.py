#!/usr/bin/env python

from numpy import *
import rospy
import roslib
from quad_analysis_methods import *

# -------------
# LIDAR COMPARE
# -------------

# this script assists the lidar_quad_node.py in processing the LIDAR scan data
# using the methods outlined in quad_analysis_methods.py

class lidar_compare():

	def __init__(self):

		action = "stand"
		direction = "left"

	def find_optimal_action(self, r_pos, angle_parameters, obst_size, safe_range, old_commands):

		# this function finds the optimal action and direction for the robot

		action = "stand"
		direction = "left"

		# create angles vector from the minimum angle and angle increment

		angles = zeros(len(r_pos))
		angle_min = angle_parameters[0]
		angle_incr = angle_parameters[2]

		for i in range(0,len(r_pos)):
			angles[i] = angle_min + angle_incr*i


		# divide scan into four quadrants, and perform analysis on each
		# results for gazebo: [back, right, front, left]

		quad_obstacles =[0.,0.,0.,0.]
		obst_percent = [0.,0.,0.,0.]
		obst_intensity = [0.,0.,0.,0.]

		# for analysis, reorder values into appropriate four quadrants

		distances = zeros(len(r_pos))
		in_range = zeros(len(r_pos))

		distances[0:45] = r_pos[315:360]
		distances[45:360] = r_pos[0:315]


		# create vector "in_range" of 1's and 0's whether points lie within the
		# specified safe_range parameter

		for i in range(0,len(distances)):
			if distances[i] > safe_range: in_range[i] = 0
			else: in_range[i] = 1


		# --- ANALYZE SCAN ---

		# method = "QUADRANT"
		quad_obstacles = analyze_quadrant(obst_size, in_range)

		# method = "PERCENT"
		obst_percent = analyze_percent(in_range)

		# method = "INTENSITY"
		obst_intensity = analyze_intensity(distances)


		# --- FINDING ACTION AND DIRECTION ---

		# this is the obstacle avoidance logic, which takes the analysis results
		# and determines the best reaction for the robot

		if quad_obstacles[2] == 0:
			action = "forward"
			direction = "left"

		elif quad_obstacles[2] == 1:

			if quad_obstacles[3] == 0 and quad_obstacles[1] == 1: # left = 0, right = 1
				action = "turn"
				direction = "left"
			elif quad_obstacles[3] == 1 and quad_obstacles[1] == 0: # left = 1, right = 0
				action = "turn"
				direction = "right"

			else:
				if obst_intensity[3] < obst_intensity[1]:
					action = "turn"
					direction = "left"
				if obst_intensity[3] >= obst_intensity[1]:
					action = "turn"
					direction = "right"

		return action, direction
