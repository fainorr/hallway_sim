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

# extract data from text file

file_name = 'eval/hall_1588256612.txt'

with open(file_name, 'r') as file:
	full = file.readlines()

link_xpos = []
link_ypos = []
link_xsize = []
link_ysize = []

for y in range(0,len(full)):
	row = full[y].split(' ')

	if y != 0:
		link_xpos.append(float(row[0]))
		link_ypos.append(float(row[1]))
		link_xsize.append(float(row[2]))
		link_ysize.append(float(row[3].strip('\n')))


# plot hall in x, y plane

fig = plt.figure(figsize=(6,6))
fig.patch.set_facecolor('w')
ax = plt.axes(xlim=(min(link_xpos)-5,max(link_xpos)+5),ylim=(min(link_ypos)-5,max(link_ypos)+5))


for i in range(0,len(link_xpos)):

	rect_x = link_xpos[i] - link_xsize[i]/2.0
	rect_y = link_ypos[i] - link_ysize[i]/2.0
	rect_len = link_xsize[i]
	rect_height = link_ysize[i]

	box = plt.Rectangle((rect_x, rect_y), rect_len, rect_height, fc='xkcd:goldenrod')
	plt.gca().add_patch(box)

	# box = ax.add_patch(patches.Rectangle((rect_x, rect_y), rect_len, rect_height, fc='xkcd:blue green'))

	# scatter = ax.scatter(link_xpos[i], link_ypos[i], s=4, fc='xkcd:blue green')

plt.show()
