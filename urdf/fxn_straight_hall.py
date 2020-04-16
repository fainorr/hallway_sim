#!/usr/bin/env python

from random import *
from numpy import *
from math import *

# -------------------------
# STRAIGHT HALLWAY FUNCTION - to generate hallway in chunks
# -------------------------

# given general hallway specs, this function will develop lists describing each
# link that can be passed to the urdf generator

def straight_hall(element, orientation, n_chunks, chunk_length, chunk_width, wall_specs, room_center, room_side):

	# orientation = direction of hallway in [+-x, +-y]
	# chunk_length = [avg, std, min]
	# chunk_width = [avg, std, min]
	# wall_specs = [height, thickness]
	# room_center = center point of room [x, y, z]
	# room_side = side length of intersection room


	# --- find starting point for hallway ---
	hall_start = [room_center[0]+orientation[0]*(room_side/2+wall_specs[1]), \
				  room_center[1]+orientation[1]*(room_side/2+wall_specs[1])]


	# --- randomly define hallway specs for each chunk ---

	lengths = zeros(n_chunks)
	widths = zeros(n_chunks)

	for i in range(0,n_chunks):
		if (i == 0) or (i == n_chunks-1): # first and last chunk always set size
			lengths[i] = round(chunk_length[0],2)
			widths[i] = round(chunk_width[0]+2.0*chunk_width[1],2)

		else: # chunks between generate as random sizes
			lengths[i] = round(gauss(chunk_length[0],chunk_length[1]),2)
			if lengths[i] < chunk_length[2]:
				lengths[i] = chunk_length[2]

			widths[i] = round(gauss(chunk_width[0],chunk_width[1]),2)
			if widths[i] < chunk_width[2]:
				widths[i] = chunk_width[2]


	# --- find next room center ---

	hall_length = sum(lengths)
	room_center = [round(room_center[0] + orientation[0]*(hall_length + room_side+2*wall_specs[1]),3), \
				   round(room_center[1] + orientation[1]*(hall_length + room_side+2*wall_specs[1]),3)]


	# --- define necessary walls for each hallway chunk ---
	# (includes walls on sides and connectors to next chunk)
	# (defined in global x, y, z coordinate system)

	# sends the urdf generator lists of the chunk number, xyz origin, size,
	# and part name of each link

	xyz_list = []
	size_list = []
	part_list = []

	for i in range(0,n_chunks):

		sidewall_pos_x = hall_start[0] + orientation[0]*(round((lengths[i]/2.0) + sum(lengths[0:i]),3)) + \
										 orientation[1]*(widths[i]/2.0 + wall_specs[1]/2.0)
		sidewall_pos_y = hall_start[1] + orientation[1]*(round((lengths[i]/2.0) + sum(lengths[0:i]),3)) + \
										 orientation[0]*(widths[i]/2.0 + wall_specs[1]/2.0)
		sidewall_pos_z = wall_specs[0]/2.0

		sidewall_size_x = orientation[0]*lengths[i] + orientation[1]*wall_specs[1]
		sidewall_size_y = orientation[1]*lengths[i] + orientation[0]*wall_specs[1]
		sidewall_size_z = wall_specs[0]

		if i < n_chunks-1:
			conn_pos_x = hall_start[0] + orientation[0]*sum(lengths[0:i+1]) + orientation[1]*((widths[i]+widths[i+1])/4 + wall_specs[1]/2)
			conn_pos_y = hall_start[1] + orientation[1]*sum(lengths[0:i+1]) + orientation[0]*((widths[i]+widths[i+1])/4 + wall_specs[1]/2)
			conn_pos_z = wall_specs[0]/2.0

			conn_size_x = orientation[0]*wall_specs[1] + orientation[1]*(abs(widths[i+1]-widths[i])/2 + wall_specs[1])
			conn_size_y = orientation[1]*wall_specs[1] + orientation[0]*(abs(widths[i+1]-widths[i])/2 + wall_specs[1])
			conn_size_z = wall_specs[0]


		# --- set link xyz and size in lists ---

		for link in range(0,4):
			if link == 0: # left wall
				link_xyz = '{} {} {}'.format(sidewall_pos_x, sidewall_pos_y, sidewall_pos_z)
				link_size = '{} {} {}'.format(sidewall_size_x, sidewall_size_y, sidewall_size_z)
				part = '{}_sec_{}_left_wall'.format(element,i+1)
			elif link == 1: # right wall
				link_xyz = '{} {} {}'.format(orientation[0]*sidewall_pos_x - orientation[1]*sidewall_pos_x, \
											 orientation[1]*sidewall_pos_y - orientation[0]*sidewall_pos_y, sidewall_pos_z)
				link_size = '{} {} {}'.format(sidewall_size_x, sidewall_size_y, sidewall_size_z)
				part = '{}_sec_{}_right_wall'.format(element,i+1)
			elif link == 2: # left connector to next chunk
				if i+1 < n_chunks:
					link_xyz = '{} {} {}'.format(conn_pos_x, conn_pos_y, conn_pos_z)
					link_size = '{} {} {}'.format(conn_size_x, conn_size_y, conn_size_z)
					part = '{}_sec_{}_left_conn'.format(element,i+1)
				else:
					break
			elif link == 3: # right connector to next chunk
				if i+1 < n_chunks:
					link_xyz = '{} {} {}'.format(conn_pos_x, -conn_pos_y, conn_pos_z)
					link_size = '{} {} {}'.format(conn_size_x, conn_size_y, conn_size_z)
					part = '{}_sec_{}_right_conn'.format(element,i+1)
				else:
					break

			xyz_list.append(link_xyz)
			size_list.append(link_size)
			part_list.append(part)


	return xyz_list, size_list, part_list, room_center
