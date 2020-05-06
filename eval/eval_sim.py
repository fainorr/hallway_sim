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

timestamp = '20-05-05-183508'


# extract hallway data from text file

with open('eval/{}_hall.txt'.format(timestamp), 'r') as file:
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

with open('eval/{}_robot.txt'.format(timestamp), 'r') as file:
	robot_full = file.readlines()

robot_xpos = []
robot_ypos = []
robot_collision = []

for y in range(0,len(robot_full)):
	row = robot_full[y].split(' ')

	if y != 0:
		robot_xpos.append(float(row[0]))
		robot_ypos.append(float(row[1]))
		robot_collision.append(row[2].strip('\n'))


# ---------------
# plot simulation
# ---------------

# find axes max and min limits
ax_max = max(max(link_xpos)+2, max(link_ypos)+2)
ax_min = max(min(link_xpos)-2, min(link_ypos)-2)


# set up x,y plane for figure
fig = plt.figure(figsize=(6,6))
fig.patch.set_facecolor('w')
ax = plt.axes(xlim=(ax_min, ax_max),ylim=(ax_min, ax_max))


# plot hallway

for i in range(0,len(link_xpos)):

	rect_x = link_xpos[i] - link_xsize[i]/2.0
	rect_y = link_ypos[i] - link_ysize[i]/2.0
	rect_len = link_xsize[i]
	rect_height = link_ysize[i]

	box = plt.Rectangle((rect_x, rect_y), rect_len, rect_height, fc='xkcd:charcoal')
	plt.gca().add_patch(box)


# plot robot path, with collisions in a separate color

for i in range(0,len(robot_xpos)):

	if robot_collision[i] == 'False':
		scatter = ax.scatter(robot_xpos[i], robot_ypos[i], s=4, c='xkcd:cadet blue')
	else:
		scatter = ax.scatter(robot_xpos[i], robot_ypos[i], s=4, c='xkcd:neon blue')


# add collision annotation

bumper_text = 'Bumper Test: PASS'
n_collisions = 0

for i in range(0,len(robot_collision)):
	if robot_collision[i] == 'True':
		bumper_text = 'Bumper Test: FAIL'
		n_collisions += 1

plt.annotate(bumper_text, xy=(0, 0), xytext=(0.05, 0.95), textcoords='axes fraction',
        horizontalalignment='left', verticalalignment='top', fontsize=9)

collisions_text = 'Collisions: {0}/{1}'.format(n_collisions, len(robot_collision))
plt.annotate(collisions_text, xy=(0, 0), xytext=(0.05, 0.9), textcoords='axes fraction',
		horizontalalignment='left', verticalalignment='top', fontsize=9)

plt.show()
