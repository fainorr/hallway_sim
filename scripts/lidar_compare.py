#!/usr/bin/env python

from numpy import *
import rospy
import roslib
from quad_analysis_methods import *

# -------------
# LIDAR COMPARE
# -------------

class lidar_compare():

	def __init__(self):

		action = "stand"
		direction = "left"

	def find_optimal_action(self, r_pos, angle_parameters, obst_size, safe_range, old_commands):

		action = "stand"
		direction = "left"

		# create angles vector
		angles = zeros(len(r_pos))
		angle_min = angle_parameters[0]
		angle_incr = angle_parameters[2]

		for i in range(0,len(r_pos)):
			angles[i] = angle_min + angle_incr*i

		x_pos = zeros(len(r_pos))
		y_pos = zeros(len(r_pos))

		for i in range(0,len(r_pos)):
			x_pos[i] = r_pos[i]*cos(angles[i])
			y_pos[i] = r_pos[i]*sin(angles[i])


		# --- ANALYZE SCAN ---
		# GAZEBO: [back, right, front, left]

		quad_obstacles =[0.,0.,0.,0.]
		obst_percent = [0.,0.,0.,0.]
		obst_intensity = [0.,0.,0.,0.]

		# for analysis, reorder values into four quadrants

		distances = zeros(len(r_pos))
		in_range = zeros(len(r_pos))

		distances[0:45] = r_pos[315:360]
		distances[45:360] = r_pos[0:315]

		for i in range(0,len(distances)):
			if distances[i] > safe_range: in_range[i] = 0
			else: in_range[i] = 1

		# method = "QUADRANT"
		quad_obstacles = analyze_quadrant(obst_size, in_range)

		# method = "PERCENT"
		obst_percent = analyze_percent(in_range)

		# method = "INTENSITY"
		obst_intensity = analyze_intensity(distances)

		# method = "CLOSEST POINT"
		closest_angle = analyze_closest(r_pos,angles)

		# FINDING ACTION AND DIRECTION

		# if quad_obstacles[2] == 0:
		# 	action = "forward"
		# 	direction = "left"
		#
		# elif quad_obstacles[2] == 1:
		#
		# 	if quad_obstacles[3] == 0 and quad_obstacles[1] == 1: # left = 0, right = 1
		# 		action = "turn"
		# 		direction = "left"
		# 	elif quad_obstacles[3] == 1 and quad_obstacles[1] == 0: # left = 1, right = 0
		# 		action = "turn"
		# 		direction = "right"
		#
		# 	else:
		# 		if obst_intensity[3] < obst_intensity[1]:
		# 			action = "turn"
		# 			direction = "left"
		# 		if obst_intensity[3] >= obst_intensity[1]:
		# 			action = "turn"
		# 			direction = "right"

		if closest_angle > 0:
			if closest_angle > pi/3 and closest_angle < 2*pi/3:
				action = "forward"
				direction = "left"
			else:
				action = "turn"
				direction = "right"

		elif closest_angle <= 0:
			if closest_angle < -pi/3 and closest_angle > -2*pi/3:
				action = "forward"
				direction = "left"
			else:
				action = "turn"
				direction = "left"

		return action, direction
