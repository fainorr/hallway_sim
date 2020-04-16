#!/usr/bin/env python

from lxml import etree
from random import *
from numpy import *
from math import *
from fxn_straight_hall import *
from fxn_hall_intersect import *

# ------------------------------------
# RANDOMLY GENERATE HALLWAY WITH TURNS
# ------------------------------------

# to install lxml library, run in terminal:
# pip install lxml


# --- define hallway specs ---

chunk_length = [5.0, 1.0, 0.5]
chunk_width = [4.0, 0.5, 2.0]
wall_specs = [3.0, 0.2]
room_side = chunk_width[0]+3.0*chunk_width[1]

xyz_list = []
size_list = []
part_list = []


# --- build environment as combination of hallways ---

# starting room
element = 1
room_center = [0,0]
openings = [False, False, True, False]
xyz, sizes, parts, room_center = hall_intersect(element, openings, chunk_width, wall_specs, room_center, room_side)
xyz_list = xyz_list + xyz
size_list = size_list + sizes
part_list = part_list + parts

# hallway
element = 2
orientation = [1,0]
n_chunks = 10
xyz, sizes, parts, room_center = straight_hall(element, orientation, n_chunks, chunk_length, chunk_width, wall_specs, room_center, room_side)
xyz_list = xyz_list + xyz
size_list = size_list + sizes
part_list = part_list + parts

# intersection room
element = 3
openings = [True, True, False, False]
xyz, sizes, parts, room_center = hall_intersect(element, openings, chunk_width, wall_specs, room_center, room_side)
xyz_list = xyz_list + xyz
size_list = size_list + sizes
part_list = part_list + parts

# hallway
element = 4
orientation = [0,1]
n_chunks = 4
xyz, sizes, parts, room_center = straight_hall(element, orientation, n_chunks, chunk_length, chunk_width, wall_specs, room_center, room_side)
xyz_list = xyz_list + xyz
size_list = size_list + sizes
part_list = part_list + parts

# intersection room
element = 5
openings = [True, False, False, True]
xyz, sizes, parts, room_center = hall_intersect(element, openings, chunk_width, wall_specs, room_center, room_side)
xyz_list = xyz_list + xyz
size_list = size_list + sizes
part_list = part_list + parts

# hallway
element = 6
orientation = [-1,0]
n_chunks = 5
xyz, sizes, parts, room_center = straight_hall(element, orientation, n_chunks, chunk_length, chunk_width, wall_specs, room_center, room_side)
xyz_list = xyz_list + xyz
size_list = size_list + sizes
part_list = part_list + parts


# --- base specs ---

base_xyz = '{} {} {}'.format(0, 0, -wall_specs[1]/2.0)
base_size = '{} {} {}'.format(2*room_center[0], 2*room_center[1], wall_specs[1])
base_inertia = ["50.0", "0.0", "0.0", "50.0", "0.0", "50.0"] # [ixx, ixy, ixz, iyy, iyz ,izz]


# ---------------
# WRITE URDF FILE
# ---------------

root = etree.Element('robot')
root.set("name","hallway")

# create base layer
base = etree.SubElement(root,"link")
base.set("name","base")
for tag in range(0,2):
    if tag == 0:
        tag_parent = etree.SubElement(base,"visual")
    else:
        tag_parent = etree.SubElement(base,"collision")
    origin = etree.SubElement(tag_parent,"origin")
    origin.set("xyz",base_xyz)
    origin.set("rpy",'0 0 0')
    geometry = etree.SubElement(tag_parent,"geometry")
    box = etree.SubElement(geometry,"box")
    box.set("size",base_size)
inertial = etree.SubElement(base,"inertial")
mass = etree.SubElement(inertial,"mass")
mass.set("value",base_inertia[0])
origin = etree.SubElement(inertial,"origin")
origin.set("xyz",base_xyz)
inertia = etree.SubElement(inertial,"inertia")
inertia.set("ixx",base_inertia[0])
inertia.set("ixy",base_inertia[1])
inertia.set("ixz",base_inertia[2])
inertia.set("iyy",base_inertia[3])
inertia.set("iyz",base_inertia[4])
inertia.set("izz",base_inertia[5])


# create walls for each chunk
n_links = len(xyz_list)
for n in range(0,n_links):
    link = etree.SubElement(root,"link")
    link.set("name", part_list[n])

    for tag in range(0,2):
        if tag == 0:
            tag_parent = etree.SubElement(link,"visual")
        else:
            tag_parent = etree.SubElement(link,"collision")

        origin = etree.SubElement(tag_parent,"origin")
        origin.set("xyz",xyz_list[n])
        origin.set("rpy",'0 0 0')

        geometry = etree.SubElement(tag_parent,"geometry")

        box = etree.SubElement(geometry,"box")
        box.set("size",size_list[n])

    # define joints
    joint = etree.SubElement(root,"joint")
    joint.set("name","joint_" + part_list[n])
    joint.set("type","fixed")

    parent = etree.SubElement(joint,"parent")
    parent.set("link","base")

    child = etree.SubElement(joint,"child")
    child.set("link", part_list[n])

    origin = etree.SubElement(joint,"origin")
    origin.set("xyz",'0 0 0')
    origin.set("rpy",'0 0 0')


# make hallway static
gazebo_link = etree.SubElement(root,"gazebo")
static = etree.SubElement(gazebo_link,"static")
static.text = "true"


etree.ElementTree(root).write("random_hall.urdf", pretty_print=True)
