#!/usr/bin/env python

from numpy import *
from math import *
from matplotlib import pyplot as plt
from matplotlib import patches
from matplotlib import _color_data

# -------------------
# EVALUATE SIMULATION (in post-processing)
# -------------------

# plot hallway links with robot path to evaluation navigation simulations

timestamp = '20-04-30-193118'


# extract hallway data from text file

with open('eval/hall_{}.txt'.format(timestamp), 'r') as file:
	hall_full = file.readlines()

link_xpos = []
link_ypos = []
link_xsize = []
link_ysize = []

for y in range(0,len(hall_full)):
	row = hall_full[y].split(' ')

	if y != 0:
		link_xpos.append(float(row[0]))
		link_ypos.append(float(row[1]))
		link_xsize.append(float(row[2]))
		link_ysize.append(float(row[3].strip('\n')))


# extract robot data from text file

with open('eval/robot_{}.txt'.format(timestamp), 'r') as file:
	robot_full = file.readlines()

robot_xpos = []
robot_ypos = []

for y in range(0,len(robot_full)):
	row = robot_full[y].split(' ')

	if y != 0:
		robot_xpos.append(float(row[0]))
		robot_ypos.append(float(row[1]))


# prepare for plot

# find axes max and min limits
ax_max = max(max(link_xpos)+2, max(link_ypos)+2)
ax_min = max(min(link_xpos)-2, min(link_ypos)-2)


# plot in x, y plane

fig = plt.figure(figsize=(6,6))
fig.patch.set_facecolor('w')
ax = plt.axes(xlim=(ax_min, ax_max),ylim=(ax_min, ax_max))


# plot hallway

for i in range(0,len(link_xpos)):

	rect_x = link_xpos[i] - link_xsize[i]/2.0
	rect_y = link_ypos[i] - link_ysize[i]/2.0
	rect_len = link_xsize[i]
	rect_height = link_ysize[i]

	box = plt.Rectangle((rect_x, rect_y), rect_len, rect_height, fc='xkcd:goldenrod')
	plt.gca().add_patch(box)


# plot robot path

for i in range(0,len(robot_xpos)):

	scatter = ax.scatter(robot_xpos[i], robot_ypos[i], s=3, c='xkcd:blue green')


plt.show()
