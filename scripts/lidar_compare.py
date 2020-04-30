#!/usr/bin/env python

from numpy import *
import rospy
import roslib

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
		# [left, back, right, front]

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

		# METHOD = "QUADRANT"

		for quad in range(0,4):
			quad_values = zeros((90-obst_size,1))

			for i in range(90*quad, 90*(quad+1) - obst_size):
				scan_obst_size = 0

				for k in range(0,obst_size):
					if in_range[i+k] == 1: scan_obst_size = scan_obst_size + 1

				if scan_obst_size == obst_size: quad_values[i-90*quad] = 1

			if sum(quad_values >= 1): quad_obstacles[quad] = 1


		# METHOD = "PERCENT"

		quad_points = [0.,0.,0.,0.]

		for quad in range(0,4):
			for i in range(90*quad, 90*(quad+1)):
				if in_range[i] == 1: quad_points[quad] = quad_points[quad] + 1

		if sum(quad_points) == 0:
			obst_percent = 0.0
		else:
			obst_percent = quad_points/sum(quad_points)


		# METHOD = "INTENSITY"

		quad_points = [0.,0.,0.,0.]

		for quad in range(0,4):
			for i in range(90*quad, 90*(quad+1)):
				if distances[i] != inf:
					quad_points[quad] = quad_points[quad] + distances[i]**2

		for quad in range(0,4):
			if quad_points[quad] > 0:
				obst_intensity[quad] = sum(quad_points)/quad_points[quad]
			else:
				obst_intensity[quad] = inf

		total_obst_intensity = sum(obst_intensity)

		for quad in range(0,4):
			if total_obst_intensity == 0.0:
				obst_intensity[quad] = 0.0
			else:
				obst_intensity[quad] = obst_intensity[quad]/total_obst_intensity


		# FINDING ACTION AND DIRECTION

		if quad_obstacles[3] == 0:
			action = "forward"
			direction = "left"

		elif quad_obstacles[3] == 1:

			if quad_obstacles[0] == 0 and quad_obstacles[2] == 1: # left = 0, right = 1
				action = "turn"
				direction = "left"
			elif quad_obstacles[0] == 1 and quad_obstacles[2] == 0: # left = 1, right = 0
				action = "turn"
				direction = "right"

			else:
				if obst_intensity[0] < obst_intensity[2]:
					if old_commands[1] == "left":
						action = "turn"
						direction = "left"
					elif old_commands[1] == "right":
						action = "turn"
						direction = "right"
				if obst_intensity[0] >= obst_intensity[2]:
					if old_commands[1] == "right":
						action = "turn"
						direction = "right"
					elif old_commands[1] == "left":
						action = "turn"
						direction = "left"

		return action, direction
