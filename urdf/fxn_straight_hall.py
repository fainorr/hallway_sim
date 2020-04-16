#!/usr/bin/env python

from random import *
from numpy import *
from math import *

# -------------------------
# STRAIGHT HALLWAY FUNCTION - to generate hallway in chunks
# -------------------------

# given general hallway specs, this function will develop lists describing each
# link that can be passed to the urdf generator

def straight_hall(orientation, n_chunks, chunk_length, chunk_width, wall_specs):

    # chunk_length = [avg, std, min]
    # chunk_width = [avg, std, min]
    # wall_specs = [height, thickness]


    # --- randomly define hallway specs for each chunk ---

    lengths = zeros(n_chunks)
    widths = zeros(n_chunks)

    for i in range(0,n_chunks):
        lengths[i] = round(gauss(chunk_length[0],chunk_length[1]),2)
        if lengths[i] < chunk_length[2]:
            lengths[i] = chunk_length[2]

        widths[i] = round(gauss(chunk_width[0],chunk_width[1]),2)
        if widths[i] < chunk_width[2]:
            widths[i] = chunk_width[2]


    # --- find overall hallway specs ---
    hall_length = sum(lengths)
    hall_max_width = max(widths)

    if orientation == "x":
        start_xyz = [hall_length+start_xyz[0], start_xyz[1], start_xyz[2]]

    elif orientation == "y":
        start_xyz = [start_xyz[0], hall_max_width+start_xyz[1], start_xyz[2]]


    # --- define necessary walls for each hallway chunk ---
    # (includes walls on sides and connectors to next chunk)

    # sends the urdf generator lists of the chunk number, xyz origin, size,
    # and part name of each link
    chunk_list = []
    xyz_list = []
    size_list = []
    part_list = []

    # defined in a 1, 2, 3 coordinate system
    for i in range(0,n_chunks):
        sidewall_pos_x = round((lengths[i]/2.0) + sum(lengths[0:i]),3)
        sidewall_pos_y = widths[i]/2.0 + wall_specs[1]/2.0
        sidewall_pos_z = wall_specs[0]/2.0

        sidewall_size_x = lengths[i]
        sidewall_size_y = wall_specs[1]
        sidewall_size_z = wall_specs[0]

        if i+1 < n_chunks:
            conn_pos_x = sum(lengths[0:i+1])
            conn_pos_y = (widths[i]+widths[i+1])/4 + wall_specs[1]/2
            conn_pos_z = wall_specs[0]/2.0

            conn_size_x = wall_specs[1]
            conn_size_y = abs(widths[i+1]-widths[i])/2 + wall_specs[1]
            conn_size_z = wall_specs[0]


        # --- set link xyz and size in lists ---

        for link in range(0,4):
            if link == 0: # left wall
                link_xyz = '{} {} {}'.format(sidewall_pos_x, sidewall_pos_y, sidewall_pos_z)
                link_size = '{} {} {}'.format(sidewall_size_x, sidewall_size_y, sidewall_size_z)
                part = 'left_wall'
            elif link == 1: # right wall
                link_xyz = '{} {} {}'.format(sidewall_pos_x, -sidewall_pos_y, sidewall_pos_z)
                link_size = '{} {} {}'.format(sidewall_size_x, sidewall_size_y, sidewall_size_z)
                part = 'right_wall'
            elif link == 2: # left connector to next chunk
                if i+1 < n_chunks:
                    link_xyz = '{} {} {}'.format(conn_pos_x, conn_pos_y, conn_pos_z)
                    link_size = '{} {} {}'.format(conn_size_x, conn_size_y, conn_size_z)
                    part = 'left_conn'
                else:
                    break
            elif link == 3: # right connector to next chunk
                if i+1 < n_chunks:
                    link_xyz = '{} {} {}'.format(conn_pos_x, -conn_pos_y, conn_pos_z)
                    link_size = '{} {} {}'.format(conn_size_x, conn_size_y, conn_size_z)
                    part = 'right_conn'
                else:
                    break

            chunk_list.append(i+1)
            xyz_list.append(link_xyz)
            size_list.append(link_size)
            part_list.append(part)


    return chunk_list, xyz_list, size_list, part_list, hall_length, hall_max_width, start_xyz
