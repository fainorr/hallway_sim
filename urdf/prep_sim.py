#!/usr/bin/env python

import rospy
from numpy import *
from math import *
from hallway_urdf_gen import URDF_generator
import os
import time
import sys

# ------------------
# PREPARE SIMULATION
# ------------------

# store start_time from launch file argument

start_time = rospy.get_param("/start_time")


# first randomize urdf for simulation

urdf_gen = URDF_generator()
xyz_list, size_list, part_list = urdf_gen.write_urdf()


# make vectors of x and y positions for each link

link_xpos = []
link_ypos = []

for item in xyz_list:
	link_xyz = item.split(' ')
	link_xpos.append(float(link_xyz[0]))
	link_ypos.append(float(link_xyz[1]))

# make vectors of x and y sizes for each link

link_xsize = []
link_ysize = []

for item in size_list:
	link_xyz = item.split(' ')
	link_xsize.append(float(link_xyz[0]))
	link_ysize.append(float(link_xyz[1]))


# push list of links to text file for post-processing

this_folder = os.path.dirname(__file__)
data_file = os.path.join(this_folder, '..', 'eval', 'hall_{}.txt'.format(start_time))

with open(data_file, 'w+') as output:
	output.write('x_pos y_pos x_size y_size\n')

	for i in range(0,len(link_xpos)):
		output.write('{0} {1} {2} {3}\n'.format(link_xpos[i], link_ypos[i], link_xsize[i], link_ysize[i]))
