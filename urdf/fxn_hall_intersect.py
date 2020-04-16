#!/usr/bin/env python

from random import *
from numpy import *
from math import *

# -----------------------------
# HALLWAY INTERSECTION FUNCTION - to generate intersection room between hallways
# -----------------------------

# given general hallway specs, this function will develop lists describing each
# link that can be passed to the urdf generator

def hall_intersect(element, openings, chunk_width, wall_specs, room_center, room_side):

    # openings = boolean, sides of room where hallways exist [-x, +y, +x, -y]
    # chunk_width = [avg, std, min]
    # wall_specs = [height, thickness]
    # room_center = center point of room [x, y, z]
    # room_side = side length of intersection room


    # --- convert "openings" to x,y offsets ---
    side_offsets = [[-1, 0], [0, 1], [1, 0], [0, -1]]

    # --- define walls depending on where openings vs. closed walls exist ---
    # (positions are calculated globally based on the room center)

    xyz_list = []
    size_list = []
    part_list = []

    for side in range(0,len(openings)):

        wall = [1,-1]
        parts = ['{}_int_side_{}_left'.format(element, side), \
                 '{}_int_side_{}_right'.format(element, side), \
                 '{}_int_side_{}'.format(element, side)]

        if openings[side] == True: # add opening for hallway
            for i in range(0,2):

                wall_pos_x = room_center[0] + side_offsets[side][0]*(room_side/2 + wall_specs[1]/2) \
                                + wall[i]*side_offsets[side][1]*(chunk_width[0]/2+(chunk_width[1]+wall_specs[1])/2)
                wall_pos_y = room_center[1] + side_offsets[side][1]*(room_side/2 + wall_specs[1]/2) \
                                + wall[i]*side_offsets[side][0]*(chunk_width[0]/2+(chunk_width[1]+wall_specs[1])/2)
                wall_pos_z = wall_specs[0]/2

                wall_size_x = abs(side_offsets[side][0]*wall_specs[1] + side_offsets[side][1]*(chunk_width[1]+2*wall_specs[1]))
                wall_size_y = abs(side_offsets[side][1]*wall_specs[1] + side_offsets[side][0]*(chunk_width[1]+2*wall_specs[1]))
                wall_size_z = wall_specs[0]

                link_xyz = '{} {} {}'.format(wall_pos_x, wall_pos_y, wall_pos_z)
                link_size = '{} {} {}'.format(wall_size_x, wall_size_y, wall_size_z)
                part = parts[i]

                xyz_list.append(link_xyz)
                size_list.append(link_size)
                part_list.append(part)


        elif openings[side] == False: # make closed wall

            wall_pos_x = room_center[0] + side_offsets[side][0]*(room_side/2 + wall_specs[1]/2)
            wall_pos_y = room_center[1] + side_offsets[side][1]*(room_side/2 + wall_specs[1]/2)
            wall_pos_z = wall_specs[0]/2

            wall_size_x = abs(side_offsets[side][0]*wall_specs[1] + side_offsets[side][1]*(room_side+2*wall_specs[1]))
            wall_size_y = abs(side_offsets[side][1]*wall_specs[1] + side_offsets[side][0]*(room_side+2*wall_specs[1]))
            wall_size_z = wall_specs[0]

            link_xyz = '{} {} {}'.format(wall_pos_x, wall_pos_y, wall_pos_z)
            link_size = '{} {} {}'.format(wall_size_x, wall_size_y, wall_size_z)
            part = parts[2]

            xyz_list.append(link_xyz)
            size_list.append(link_size)
            part_list.append(part)


    return xyz_list, size_list, part_list, room_center
